use super::compound::CompoundShape;
use super::edge::Edge;
use super::face::Face;
use super::ffi;
use crate::common::boolean::Boolean;
use crate::common::error::Error;
use crate::traits::{ProfileOrient, SolidStruct, Transform};
use glam::DVec3;
use std::sync::{Mutex, OnceLock};

// OCCT の BRepOffsetAPI_ThruSections は内部で global state (おそらく
// BSplCLib のキャッシュや GeomFill_AppSurf の作業バッファ) を使うため、
// 複数スレッドから同時に呼び出すと heap corruption を起こす。
// 並列テスト実行下で再現する症状で、loft 呼び出し全体を Mutex で
// serialize すれば回避できる。性能劣化はあるが loft は重い操作なので
// ロック粒度の粗さは現実的に問題にならない。
static LOFT_LOCK: Mutex<()> = Mutex::new(());

/// Encode `ProfileOrient` into FFI arguments: (kind, ux, uy, uz, aux_spine_edges).
fn encode_orient(orient: ProfileOrient) -> (u32, f64, f64, f64, cxx::UniquePtr<cxx::CxxVector<ffi::TopoDS_Edge>>) {
	let mut aux_vec = ffi::edge_vec_new();
	let (kind, ux, uy, uz) = match orient {
		ProfileOrient::Fixed => (0u32, 0.0, 0.0, 0.0),
		ProfileOrient::Torsion => (1u32, 0.0, 0.0, 0.0),
		ProfileOrient::Up(v) => (2u32, v.x, v.y, v.z),
		ProfileOrient::Auxiliary(edges) => {
			for e in edges {
				ffi::edge_vec_push(aux_vec.pin_mut(), &e.inner);
			}
			(3u32, 0.0, 0.0, 0.0)
		}
	};
	(kind, ux, uy, uz, aux_vec)
}

#[cfg(feature = "color")]
fn remap_colormap_by_order(old_inner: &ffi::TopoDS_Shape, new_inner: &ffi::TopoDS_Shape, old_colormap: &std::collections::HashMap<u64, crate::common::color::Color>) -> std::collections::HashMap<u64, crate::common::color::Color> {
	let mut colormap = std::collections::HashMap::new();
	let old_faces = ffi::shape_faces(old_inner);
	let new_faces = ffi::shape_faces(new_inner);
	for (old_face, new_face) in old_faces.iter().zip(new_faces.iter()) {
		if let Some(&color) = old_colormap.get(&ffi::face_tshape_id(old_face)) {
			colormap.insert(ffi::face_tshape_id(new_face), color);
		}
	}
	// The solid's own colour is keyed by its TShape id, which these ops change
	// (they rebuild topology), so it needs the same remap the faces get.
	if let Some(&color) = old_colormap.get(&ffi::shape_tshape_id(old_inner)) {
		colormap.insert(ffi::shape_tshape_id(new_inner), color);
	}
	colormap
}

/// A single solid topology shape wrapping a `TopoDS_Shape` guaranteed to be `TopAbs_SOLID`.
///
/// `inner` is private to prevent external mutation that could break the solid invariant.
/// Use the provided methods to query and transform the solid.
///
/// `edges` / `faces` are lazy `OnceLock` caches populated on first `iter_edge` /
/// `iter_face` call. Since topology-changing ops consume `self` and construct
/// a new `Solid` via `Solid::new`, these caches are invalidated automatically
/// (new instance → fresh `OnceLock::new()`). See
/// `notes/20260420-OCCTトポロジ不変性と設計含意.md`.
pub struct Solid {
	inner: cxx::UniquePtr<ffi::TopoDS_Shape>,
	edges: OnceLock<Vec<Edge>>,
	faces: OnceLock<Vec<Face>>,
	/// Keyed by a face's TShape id, or by `Solid::id()` for the solid as a whole; a face
	/// colour wins over the solid's. Other solids' keys may be present (`decompose`).
	#[cfg(feature = "color")]
	colormap: std::collections::HashMap<u64, crate::common::color::Color>,
	/// Face-derivation history from the most recent boolean operation.
	///
	/// Flat `[post_id, src_id, post_id, src_id, ...]` pairs:
	/// - `post_id` is the TShape* of a face in this Solid. The legacy list is
	///   filtered when a compound result is decomposed.
	/// - `src_id` is the TShape* of the originating face in either
	///   boolean input (a or b — distinction is intentionally lost;
	///   TShape* is globally unique so callers filter by membership).
	///
	/// Empty for primitives, source-edge builders (extrude/sweep/loft), I/O
	/// reads, and after scale/mirror/explicit deep copy. Preserved across
	/// translate/rotate/color. New consumers should prefer `topology_history`.
	history: Vec<u64>,
	/// Complete operation-local correspondence for the most recent
	/// topology-changing operation. Unlike `history`, this is ordinal-based,
	/// operand-aware, and covers faces, edges, and vertices.
	topology_history: TopologyHistory,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EdgeBlendKind {
	Fillet,
	Chamfer,
}

pub struct EdgeBlendSession {
	source: Solid,
	edge_indices: Vec<u32>,
}

pub struct ExtrusionSession {
	profile: Vec<Edge>,
}

/// Prepared invariant profile topology for repeated sweep/revolution updates.
pub struct SweepSession {
	profile: Vec<Edge>,
}

pub struct FaceEditSession {
	source: Solid,
	face_index: u32,
	boundary: Vec<Edge>,
}

/// Prepared source topology for repeated shell-thickness updates.
pub struct ShellSession {
	source: Solid,
	open_face_indices: Vec<u32>,
}

/// Exact `BRepCheck_Analyzer` result for one solid occurrence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ValidationReport {
	pub valid: bool,
	pub invalid_faces: u32,
	pub invalid_edges: u32,
	pub invalid_vertices: u32,
}

/// Exact minimum distance and witness points between two topology entities.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TopologyDistance {
	pub distance: f64,
	pub first_point: DVec3,
	pub second_point: DVec3,
}

/// The dimension of an artifact-local topology entity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TopologyKind {
	Face,
	Edge,
	Vertex,
}

/// The way an OCCT builder related an input entity to a result entity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum TopologyRelationKind {
	Unchanged,
	Modified,
	Generated,
}

/// A topology entity in one input operand of an operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct InputTopology {
	pub operand: u32,
	pub kind: TopologyKind,
	pub index: u32,
}

/// A topology entity in the operation result artifact.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct ResultTopology {
	pub kind: TopologyKind,
	pub index: u32,
}

/// One many-to-many operation history relation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TopologyRelation {
	pub result: ResultTopology,
	pub relation: TopologyRelationKind,
	pub source: InputTopology,
}

/// Complete topology history from the most recent operation.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TopologyHistory {
	relations: Vec<TopologyRelation>,
	deleted: Vec<InputTopology>,
	unresolved: Vec<ResultTopology>,
}

impl TopologyHistory {
	pub fn relations(&self) -> &[TopologyRelation] {
		&self.relations
	}

	pub fn deleted(&self) -> &[InputTopology] {
		&self.deleted
	}

	pub fn unresolved(&self) -> &[ResultTopology] {
		&self.unresolved
	}

	pub fn sources_for(&self, result: ResultTopology) -> impl Iterator<Item = &TopologyRelation> {
		self.relations.iter().filter(move |relation| relation.result == result)
	}

	pub(crate) fn remap_result_to(&self, global: &TopologySnapshot, local: &TopologySnapshot) -> Self {
		let remap = |result: ResultTopology| -> Option<ResultTopology> {
			let (global_ids, local_ids) = match result.kind {
				TopologyKind::Face => (global.face_ids(), local.face_ids()),
				TopologyKind::Edge => (global.edge_ids(), local.edge_ids()),
				TopologyKind::Vertex => (global.vertex_ids(), local.vertex_ids()),
			};
			let id = *global_ids.get(result.index as usize)?;
			let index = u32::try_from(local_ids.iter().position(|candidate| *candidate == id)?).ok()?;
			Some(ResultTopology { kind: result.kind, index })
		};

		let mut relations = self.relations.iter().filter_map(|relation| Some(TopologyRelation { result: remap(relation.result)?, ..*relation })).collect::<Vec<_>>();
		relations.sort_unstable_by_key(|item| (item.result.kind as u8, item.result.index, item.source.operand, item.source.kind as u8, item.source.index, item.relation as u8));
		relations.dedup();
		let unresolved = self.unresolved.iter().filter_map(|result| remap(*result)).collect();
		Self { relations, deleted: self.deleted.clone(), unresolved }
	}
}

/// Request flags for an artifact-local topology traversal.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TopologyQueryOptions {
	pub frames: bool,
	pub measurements: bool,
	pub geometry: bool,
}

impl TopologyQueryOptions {
	pub const INTERACTION: Self = Self { frames: true, measurements: false, geometry: true };
	pub const MEASUREMENT: Self = Self { frames: true, measurements: true, geometry: true };

	fn bits(self) -> u32 {
		u32::from(self.frames) | (u32::from(self.measurements) << 1) | (u32::from(self.geometry) << 2)
	}
}

/// An occurrence-aware token scoped to one exact artifact.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TopologyOccurrenceToken {
	pub tshape_id: u64,
	pub location_hash: u64,
	pub orientation: u32,
	pub ordinal: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SurfaceGeometryKind {
	Plane,
	Cylinder,
	Cone,
	Sphere,
	Torus,
	Bezier,
	BSpline,
	Revolution,
	Extrusion,
	Offset,
	Other,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CurveGeometryKind {
	Line,
	Circle,
	Ellipse,
	Hyperbola,
	Parabola,
	Bezier,
	BSpline,
	Offset,
	Other,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FaceTopologyFacts {
	pub token: TopologyOccurrenceToken,
	pub geometry: Option<SurfaceGeometryKind>,
	pub representative_point: Option<[f64; 3]>,
	pub normal: Option<[f64; 3]>,
	pub area: Option<f64>,
	pub closed: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct EdgeTopologyFacts {
	pub token: TopologyOccurrenceToken,
	pub geometry: Option<CurveGeometryKind>,
	pub midpoint: Option<[f64; 3]>,
	pub tangent: Option<[f64; 3]>,
	pub outward_direction: Option<[f64; 3]>,
	pub length: Option<f64>,
	pub vertices: Vec<u32>,
	pub closed: bool,
	pub degenerate: bool,
	pub seam: bool,
	pub manifold: bool,
}

#[derive(Clone, Debug, PartialEq)]
pub struct VertexTopologyFacts {
	pub token: TopologyOccurrenceToken,
	pub point: Option<[f64; 3]>,
}

/// One artifact-local topology index produced in a single OCCT traversal.
#[derive(Clone, Debug, PartialEq)]
pub struct TopologySnapshot {
	face_ids: Vec<u64>,
	edge_ids: Vec<u64>,
	vertex_ids: Vec<u64>,
	face_edges: Vec<Vec<u32>>,
	edge_faces: Vec<Vec<u32>>,
	edge_vertices: Vec<Vec<u32>>,
	face_facts: Vec<FaceTopologyFacts>,
	edge_facts: Vec<EdgeTopologyFacts>,
	vertex_facts: Vec<VertexTopologyFacts>,
}

impl TopologySnapshot {
	pub fn face_ids(&self) -> &[u64] {
		&self.face_ids
	}

	pub fn edge_ids(&self) -> &[u64] {
		&self.edge_ids
	}

	pub fn vertex_ids(&self) -> &[u64] {
		&self.vertex_ids
	}

	pub fn face_edges(&self, face: u32) -> Option<&[u32]> {
		self.face_edges.get(face as usize).map(Vec::as_slice)
	}

	pub fn edge_faces(&self, edge: u32) -> Option<&[u32]> {
		self.edge_faces.get(edge as usize).map(Vec::as_slice)
	}

	pub fn edge_vertices(&self, edge: u32) -> Option<&[u32]> {
		self.edge_vertices.get(edge as usize).map(Vec::as_slice)
	}

	pub fn face_facts(&self, face: u32) -> Option<&FaceTopologyFacts> {
		self.face_facts.get(face as usize)
	}

	pub fn edge_facts(&self, edge: u32) -> Option<&EdgeTopologyFacts> {
		self.edge_facts.get(edge as usize)
	}

	pub fn vertex_facts(&self, vertex: u32) -> Option<&VertexTopologyFacts> {
		self.vertex_facts.get(vertex as usize)
	}
}

impl Solid {
	/// Create a `Solid` from a `TopoDS_Shape`.
	///
	/// # Panics
	/// Panics if `inner` is not `TopAbs_SOLID` (and not null).
	pub(crate) fn new(inner: cxx::UniquePtr<ffi::TopoDS_Shape>, #[cfg(feature = "color")] colormap: std::collections::HashMap<u64, crate::common::color::Color>, history: Vec<u64>) -> Self {
		debug_assert!(ffi::shape_is_null(&inner) || ffi::shape_is_solid(&inner), "Solid::new called with a non-SOLID shape");
		Solid {
			inner,
			edges: OnceLock::new(),
			faces: OnceLock::new(),
			#[cfg(feature = "color")]
			colormap,
			history,
			topology_history: TopologyHistory::default(),
		}
	}

	pub(crate) fn with_topology_history(mut self, topology_history: TopologyHistory) -> Self {
		self.topology_history = topology_history;
		self
	}

	fn from_primitive_ffi(inner: cxx::UniquePtr<ffi::TopoDS_Shape>, operation: &'static str) -> Result<Self, Error> {
		if inner.is_null() || !ffi::shape_is_solid(&inner) {
			return Err(ffi::operation_error(Error::InvalidInput(format!("{operation} did not produce a solid")), operation, "occt_build"));
		}
		Ok(Solid::new(
			inner,
			#[cfg(feature = "color")]
			std::collections::HashMap::new(),
			Default::default(),
		))
	}

	pub fn try_cube(corner0: DVec3, corner1: DVec3) -> Result<Self, Error> {
		if !corner0.is_finite() || !corner1.is_finite() || (corner1 - corner0).abs().min_element() <= 0.0 {
			return Err(Error::InvalidInput("box corners must be finite and differ on every axis".into()));
		}
		ffi::begin_operation();
		Self::from_primitive_ffi(ffi::make_box(corner0.x, corner0.y, corner0.z, corner1.x, corner1.y, corner1.z), "build box")
	}

	pub fn try_sphere(radius: f64) -> Result<Self, Error> {
		if !radius.is_finite() || radius <= 0.0 {
			return Err(Error::InvalidInput("sphere radius must be finite and positive".into()));
		}
		ffi::begin_operation();
		Self::from_primitive_ffi(ffi::make_sphere(0.0, 0.0, 0.0, radius), "build sphere")
	}

	pub fn try_cylinder(radius: f64, height: DVec3) -> Result<Self, Error> {
		if !radius.is_finite() || radius <= 0.0 || !height.is_finite() || height == DVec3::ZERO {
			return Err(Error::InvalidInput("cylinder radius and height must be finite and nonzero".into()));
		}
		ffi::begin_operation();
		Self::from_primitive_ffi(ffi::make_cylinder(0.0, 0.0, 0.0, height.x, height.y, height.z, radius, height.length()), "build cylinder")
	}

	pub fn try_cone(radius0: f64, radius1: f64, height: DVec3) -> Result<Self, Error> {
		if !radius0.is_finite() || !radius1.is_finite() || radius0 < 0.0 || radius1 < 0.0 || radius0.max(radius1) <= 0.0 || !height.is_finite() || height == DVec3::ZERO {
			return Err(Error::InvalidInput("cone radii and height must be finite, with a positive radius and nonzero height".into()));
		}
		ffi::begin_operation();
		Self::from_primitive_ffi(ffi::make_cone(0.0, 0.0, 0.0, height.x, height.y, height.z, radius0, radius1, height.length()), "build cone")
	}

	pub fn try_torus(major_radius: f64, minor_radius: f64, axis: DVec3) -> Result<Self, Error> {
		if !major_radius.is_finite() || !minor_radius.is_finite() || minor_radius <= 0.0 || major_radius <= minor_radius || !axis.is_finite() || axis == DVec3::ZERO {
			return Err(Error::InvalidInput("torus radii must be finite with 0 < minor < major and the axis nonzero".into()));
		}
		ffi::begin_operation();
		Self::from_primitive_ffi(ffi::make_torus(0.0, 0.0, 0.0, axis.x, axis.y, axis.z, major_radius, minor_radius), "build torus")
	}

	pub fn try_half_space(plane_origin: DVec3, plane_normal: DVec3) -> Result<Self, Error> {
		if !plane_origin.is_finite() || !plane_normal.is_finite() || plane_normal == DVec3::ZERO {
			return Err(Error::InvalidInput("half-space origin must be finite and its normal finite and nonzero".into()));
		}
		ffi::begin_operation();
		Self::from_primitive_ffi(ffi::make_half_space(plane_origin.x, plane_origin.y, plane_origin.z, plane_normal.x, plane_normal.y, plane_normal.z), "build half space")
	}

	/// Return complete, operand-aware topology correspondence for the most
	/// recent topology-changing operation.
	pub fn topology_history(&self) -> &TopologyHistory {
		&self.topology_history
	}

	pub fn fillet_edges_cancelable<'a>(&self, radius: f64, edges: impl IntoIterator<Item = &'a Edge>, progress: &ffi::CancellationToken) -> Result<Self, Error> {
		if !radius.is_finite() || radius <= 0.0 {
			return Err(Error::InvalidInput("fillet radius must be finite and positive".into()));
		}
		let mut edge_vec = ffi::edge_vec_new();
		for edge in edges {
			ffi::edge_vec_push(edge_vec.pin_mut(), &edge.inner);
		}
		let mut history = Vec::new();
		let mut topology_history = empty_ffi_history();
		ffi::begin_operation();
		let shape = ffi::builder_fillet(&self.inner, &edge_vec, radius, progress, &mut history, &mut topology_history);
		if shape.is_null() {
			return Err(if progress.is_cancelled() { Error::Cancelled } else { ffi::operation_error(Error::FilletFailed, "fillet", "occt_build") });
		}
		let topology_history = decode_topology_history(topology_history)?;
		#[cfg(feature = "color")]
		let colormap = self.remap_colormap(&shape, &history);
		Ok(Solid::new(
			shape,
			#[cfg(feature = "color")]
			colormap,
			history,
		)
		.with_topology_history(topology_history))
	}

	pub fn chamfer_edges_cancelable<'a>(&self, distance: f64, edges: impl IntoIterator<Item = &'a Edge>, progress: &ffi::CancellationToken) -> Result<Self, Error> {
		if !distance.is_finite() || distance <= 0.0 {
			return Err(Error::InvalidInput("chamfer distance must be finite and positive".into()));
		}
		let mut edge_vec = ffi::edge_vec_new();
		for edge in edges {
			ffi::edge_vec_push(edge_vec.pin_mut(), &edge.inner);
		}
		let mut history = Vec::new();
		let mut topology_history = empty_ffi_history();
		ffi::begin_operation();
		let shape = ffi::builder_chamfer(&self.inner, &edge_vec, distance, progress, &mut history, &mut topology_history);
		if shape.is_null() {
			return Err(if progress.is_cancelled() { Error::Cancelled } else { ffi::operation_error(Error::ChamferFailed, "chamfer", "occt_build") });
		}
		let topology_history = decode_topology_history(topology_history)?;
		#[cfg(feature = "color")]
		let colormap = self.remap_colormap(&shape, &history);
		Ok(Solid::new(
			shape,
			#[cfg(feature = "color")]
			colormap,
			history,
		)
		.with_topology_history(topology_history))
	}

	pub fn shell_cancelable<'a>(&self, thickness: f64, open_faces: impl IntoIterator<Item = &'a Face>, progress: &ffi::CancellationToken) -> Result<Self, Error> {
		if !thickness.is_finite() || thickness == 0.0 {
			return Err(Error::InvalidInput("shell thickness must be finite and nonzero".into()));
		}
		let mut face_vec = ffi::face_vec_new();
		for face in open_faces {
			ffi::face_vec_push(face_vec.pin_mut(), &face.inner);
		}
		let mut history = Vec::new();
		let mut topology_history = empty_ffi_history();
		ffi::begin_operation();
		let shape = ffi::builder_thick_solid(&self.inner, &face_vec, thickness, progress, &mut history, &mut topology_history);
		if shape.is_null() {
			return Err(if progress.is_cancelled() { Error::Cancelled } else { ffi::operation_error(Error::ShellFailed, "shell", "occt_build") });
		}
		let topology_history = decode_topology_history(topology_history)?;
		#[cfg(feature = "color")]
		let colormap = self.remap_colormap(&shape, &history);
		Ok(Solid::new(
			shape,
			#[cfg(feature = "color")]
			colormap,
			history,
		)
		.with_topology_history(topology_history))
	}

	pub fn boolean_build_cancelable(b: &Boolean<Self>, progress: &ffi::CancellationToken) -> Result<Vec<Self>, Error> {
		boolean_build_with_progress(b, progress)
	}

	pub fn extrude_cancelable<'a>(profile: impl IntoIterator<Item = &'a Edge>, direction: DVec3, progress: &ffi::CancellationToken) -> Result<Self, Error> {
		if !direction.is_finite() || direction == DVec3::ZERO {
			return Err(Error::InvalidInput("extrusion direction must be finite and nonzero".into()));
		}
		let mut profile_vec = ffi::edge_vec_new();
		for edge in profile {
			ffi::edge_vec_push(profile_vec.pin_mut(), &edge.inner);
		}
		let mut topology_history = empty_ffi_history();
		ffi::begin_operation();
		let shape = ffi::make_extrude(&profile_vec, direction.x, direction.y, direction.z, progress, &mut topology_history);
		if shape.is_null() {
			return Err(if progress.is_cancelled() { Error::Cancelled } else { ffi::operation_error(Error::ExtrudeFailed, "extrude", "occt_build") });
		}
		let topology_history = decode_topology_history(topology_history)?;
		Ok(Solid::new(
			shape,
			#[cfg(feature = "color")]
			std::collections::HashMap::new(),
			Default::default(),
		)
		.with_topology_history(topology_history))
	}

	pub fn sweep_cancelable<'a, 'b, 'c>(profile: impl IntoIterator<Item = &'a Edge>, spine: impl IntoIterator<Item = &'b Edge>, orient: ProfileOrient<'c>, progress: &ffi::CancellationToken) -> Result<Self, Error> {
		match orient {
			ProfileOrient::Up(up) if !up.is_finite() || up == DVec3::ZERO => {
				return Err(Error::InvalidInput("sweep up direction must be finite and nonzero".into()));
			}
			ProfileOrient::Auxiliary([]) => {
				return Err(Error::InvalidInput("an auxiliary sweep needs a spine".into()));
			}
			_ => {}
		}
		let mut profile_vec = ffi::edge_vec_new();
		for edge in profile {
			ffi::edge_vec_push(profile_vec.pin_mut(), &edge.inner);
		}
		let mut spine_vec = ffi::edge_vec_new();
		for edge in spine {
			ffi::edge_vec_push(spine_vec.pin_mut(), &edge.inner);
		}
		let (kind, ux, uy, uz, auxiliary) = encode_orient(orient);
		let mut topology_history = empty_ffi_history();
		ffi::begin_operation();
		let shape = ffi::make_pipe_shell(&profile_vec, &spine_vec, kind, ux, uy, uz, &auxiliary, progress, &mut topology_history);
		if shape.is_null() {
			return Err(if progress.is_cancelled() { Error::Cancelled } else { ffi::operation_error(Error::SweepFailed, "sweep", "occt_build") });
		}
		let topology_history = decode_topology_history(topology_history)?;
		Ok(Solid::new(
			shape,
			#[cfg(feature = "color")]
			std::collections::HashMap::new(),
			Default::default(),
		)
		.with_topology_history(topology_history))
	}

	pub fn loft_cancelable<'a, I: IntoIterator<Item = &'a Edge>, S: IntoIterator<Item = I>>(sections: S, ruled: bool, progress: &ffi::CancellationToken) -> Result<Self, Error>
	where
		Edge: 'a,
	{
		let _guard = LOFT_LOCK.lock().unwrap_or_else(|e| e.into_inner());

		let mut all_edges = ffi::edge_vec_new();
		let mut section_count = 0usize;
		for section in sections {
			if section_count > 0 {
				ffi::edge_vec_push_null(all_edges.pin_mut());
			}
			let mut count = 0u32;
			for edge in section {
				ffi::edge_vec_push(all_edges.pin_mut(), &edge.inner);
				count += 1;
			}
			if count == 0 {
				return Err(Error::LoftFailed(format!("loft: section {section_count} is empty (each section must contain ≥1 edge)")));
			}
			section_count += 1;
		}

		if section_count < 2 {
			return Err(Error::LoftFailed(format!("loft: need ≥2 sections, got {section_count} (a single section has no thickness to skin across)")));
		}

		let mut topology_history = empty_ffi_history();
		ffi::begin_operation();
		let shape = ffi::make_loft(&all_edges, ruled, progress, &mut topology_history);
		if shape.is_null() {
			return Err(if progress.is_cancelled() { Error::Cancelled } else { ffi::operation_error(Error::LoftFailed(format!("loft: OCCT BRepOffsetAPI_ThruSections failed (sections={section_count}, ruled={ruled}). Check that each section forms a valid closed wire and sections are not coplanar.")), "loft", "occt_build") });
		}
		let topology_history = decode_topology_history(topology_history)?;
		Ok(Solid::new(
			shape,
			#[cfg(feature = "color")]
			std::collections::HashMap::new(),
			Default::default(),
		)
		.with_topology_history(topology_history))
	}

	// ==================== Internal accessors ====================

	/// Borrow the underlying `TopoDS_Shape` (crate-internal only).
	pub(crate) fn inner(&self) -> &ffi::TopoDS_Shape {
		&self.inner
	}

	/// Query face/edge identity and adjacency in one OCCT traversal.
	pub fn topology_snapshot(&self) -> Result<TopologySnapshot, Error> {
		topology_snapshot_from_shape(&self.inner)
	}

	/// Query occurrence-aware topology plus the requested immutable facts in one traversal.
	pub fn topology_snapshot_with_options(&self, options: TopologyQueryOptions) -> Result<TopologySnapshot, Error> {
		topology_snapshot_from_shape_with_options(&self.inner, options)
	}

	/// Return the exact closest points between two artifact-local entities.
	pub fn topology_distance(&self, first: ResultTopology, other: &Self, second: ResultTopology) -> Result<TopologyDistance, Error> {
		let kind = |value| match value {
			TopologyKind::Face => 0,
			TopologyKind::Edge => 1,
			TopologyKind::Vertex => 2,
		};
		ffi::begin_operation();
		let result = ffi::topology_distance(&self.inner, kind(first.kind), first.index, &other.inner, kind(second.kind), second.index);
		if !result.success {
			return Err(ffi::operation_error(Error::TopologyQueryFailed, "topology distance", "native"));
		}
		Ok(TopologyDistance { distance: result.distance, first_point: DVec3::new(result.first_x, result.first_y, result.first_z), second_point: DVec3::new(result.second_x, result.second_y, result.second_z) })
	}

	/// Run OCCT's exact B-rep analyzer and report invalid subshape counts.
	pub fn validate(&self) -> Result<ValidationReport, Error> {
		ffi::begin_operation();
		let report = ffi::shape_validation(&self.inner);
		if !report.success {
			return Err(ffi::operation_error(Error::TopologyQueryFailed, "validate shape", "validate_result"));
		}
		Ok(ValidationReport { valid: report.valid, invalid_faces: report.invalid_faces, invalid_edges: report.invalid_edges, invalid_vertices: report.invalid_vertices })
	}

	/// Return process-local `(self, other)` face pairs with the same OCCT
	/// occurrence (TShape and location); these are not persistent identifiers.
	pub fn shared_face_indices(&self, other: &Self) -> Vec<(u32, u32)> {
		ffi::shared_face_indices(&self.inner, &other.inner).chunks_exact(2).map(|pair| (pair[0], pair[1])).collect()
	}

	/// Mesh solids into topology-keyed face and edge chunks suitable for
	/// progressive or incremental presentation.
	pub fn mesh_chunks<'a>(solids: impl IntoIterator<Item = &'a Self>, options: crate::traits::Tessellation) -> Result<crate::common::mesh::MeshChunks, Error> {
		super::io::mesh_chunks(solids, options)
	}

	/// Mesh topology-keyed chunks with cooperative cancellation.
	pub fn mesh_chunks_cancelable<'a>(solids: impl IntoIterator<Item = &'a Self>, options: crate::traits::Tessellation, progress: &ffi::CancellationToken) -> Result<crate::common::mesh::MeshChunks, Error> {
		super::io::mesh_chunks_cancelable(solids, options, progress)
	}

	/// Mesh only selected artifact-local face ordinals so callers can retain
	/// unchanged chunks across a local edit.
	pub fn mesh_face_chunks(&self, face_indices: &[u32], options: crate::traits::Tessellation) -> Result<Vec<crate::common::mesh::FaceMeshChunk>, Error> {
		super::io::mesh_face_chunks(self, face_indices, options)
	}

	/// Mesh selected artifact-local faces with cooperative cancellation.
	pub fn mesh_face_chunks_cancelable(&self, face_indices: &[u32], options: crate::traits::Tessellation, progress: &ffi::CancellationToken) -> Result<Vec<crate::common::mesh::FaceMeshChunk>, Error> {
		super::io::mesh_face_chunks_cancelable(self, face_indices, options, progress)
	}

	/// Discretize only ordered topological edges without surface meshing.
	pub fn edge_polyline_chunks(&self, options: crate::traits::Tessellation) -> Result<Vec<crate::common::mesh::EdgePolylineChunk>, Error> {
		super::io::edge_polyline_chunks(self, options)
	}

	/// Discretize ordered edges with cancellation between edge stages.
	pub fn edge_polyline_chunks_cancelable(&self, options: crate::traits::Tessellation, progress: &ffi::CancellationToken) -> Result<Vec<crate::common::mesh::EdgePolylineChunk>, Error> {
		super::io::edge_polyline_chunks_cancelable(self, options, progress)
	}

	/// Create a shallow occurrence sharing topology and geometry with this solid.
	pub fn shared_copy(&self) -> Self {
		Solid::new(
			ffi::clone_shape_handle(&self.inner),
			#[cfg(feature = "color")]
			self.colormap.clone(),
			self.history.clone(),
		)
		.with_topology_history(self.topology_history.clone())
	}

	/// Create a rigidly located occurrence without deep-copying its topology.
	pub fn located(&self, axis: DVec3, angle: f64, translation: DVec3) -> Self {
		let mut result = self.shared_copy();
		if angle.abs() > f64::EPSILON {
			result = result.rotate(DVec3::ZERO, axis, angle);
		}
		result.translate(translation)
	}

	pub fn prepare_edge_blend(&self, edge_indices: &[u32]) -> Result<EdgeBlendSession, Error> {
		if edge_indices.is_empty() {
			return Err(Error::InvalidEdge("an edge blend needs at least one edge".into()));
		}
		let edge_count = self.iter_edge().count();
		if edge_indices.iter().any(|index| *index as usize >= edge_count) {
			return Err(Error::InvalidEdge("an edge blend index is outside the source topology".into()));
		}
		let source = self.shared_copy();
		// Resolve the source topology once when the session is prepared. Repeated
		// updates then reuse both the live source shape and its cached edge handles.
		source.iter_edge().count();
		Ok(EdgeBlendSession { source, edge_indices: edge_indices.to_vec() })
	}

	pub fn prepare_face_edit(&self, face_index: u32) -> Result<FaceEditSession, Error> {
		let face = self.iter_face().nth(face_index as usize).ok_or_else(|| Error::InvalidEdge("a face edit index is outside the source topology".into()))?;
		let boundary = face.iter_edge().map(Edge::shared_copy).collect::<Vec<_>>();
		if boundary.is_empty() {
			return Err(Error::InvalidEdge("a face edit needs a bounded source face".into()));
		}
		let source = self.shared_copy();
		source.iter_face().count();
		source.iter_edge().count();
		Ok(FaceEditSession { source, face_index, boundary })
	}

	pub fn prepare_shell(&self, open_face_indices: &[u32]) -> Result<ShellSession, Error> {
		let face_count = self.iter_face().count();
		if open_face_indices.iter().any(|index| *index as usize >= face_count) {
			return Err(Error::InvalidInput("a shell opening index is outside the source topology".into()));
		}
		let source = self.shared_copy();
		source.iter_face().count();
		Ok(ShellSession { source, open_face_indices: open_face_indices.to_vec() })
	}

	/// Create an independent full copy with an ordinal source map. This creates
	/// new OCCT topology and is proportional to the complete shape.
	pub fn deep_copy_with_map(&self) -> Result<Self, Error> {
		let source = self.topology_snapshot()?;
		let inner = ffi::deep_copy(&self.inner);
		#[cfg(feature = "color")]
		let colormap = remap_colormap_by_order(&self.inner, &inner, &self.colormap);
		let mut result = Solid::new(
			inner,
			#[cfg(feature = "color")]
			colormap,
			Default::default(),
		);
		let copied = result.topology_snapshot()?;
		if source.face_ids().len() != copied.face_ids().len() || source.edge_ids().len() != copied.edge_ids().len() || source.vertex_ids().len() != copied.vertex_ids().len() {
			return Err(Error::TopologyQueryFailed);
		}
		let mut relations = Vec::new();
		for (kind, count) in [(TopologyKind::Face, source.face_ids().len()), (TopologyKind::Edge, source.edge_ids().len()), (TopologyKind::Vertex, source.vertex_ids().len())] {
			for index in 0..u32::try_from(count).map_err(|_| Error::TopologyQueryFailed)? {
				relations.push(TopologyRelation { result: ResultTopology { kind, index }, relation: TopologyRelationKind::Unchanged, source: InputTopology { operand: 0, kind, index } });
			}
		}
		result.topology_history = TopologyHistory { relations, deleted: Vec::new(), unresolved: Vec::new() };
		Ok(result)
	}

	/// Create a detached full copy for presentation operations that may mutate
	/// shape-local caches.
	pub fn presentation_copy(&self) -> Result<Self, Error> {
		let inner = ffi::deep_copy(&self.inner);
		if inner.is_null() {
			return Err(Error::TopologyQueryFailed);
		}
		#[cfg(feature = "color")]
		let colormap = remap_colormap_by_order(&self.inner, &inner, &self.colormap);
		Ok(Solid::new(
			inner,
			#[cfg(feature = "color")]
			colormap,
			Default::default(),
		))
	}

	// ==================== Color accessors ====================

	/// Read-only access to the per-face colormap.
	#[cfg(feature = "color")]
	pub fn colormap(&self) -> &std::collections::HashMap<u64, crate::common::color::Color> {
		&self.colormap
	}

	/// Mutable access to the per-face colormap.
	#[cfg(feature = "color")]
	pub fn colormap_mut(&mut self) -> &mut std::collections::HashMap<u64, crate::common::color::Color> {
		&mut self.colormap
	}

	/// Carry face colours across `history` `[post_id, src_id]` pairs, and the solid's
	/// own colour onto the new solid (shell/fillet/chamfer/clean).
	#[cfg(feature = "color")]
	fn remap_colormap(&self, new_inner: &ffi::TopoDS_Shape, history: &[u64]) -> std::collections::HashMap<u64, crate::common::color::Color> {
		let mut colormap: std::collections::HashMap<u64, crate::common::color::Color> = history.chunks_exact(2).filter_map(|p| Some((p[0], *self.colormap.get(&p[1])?))).collect();
		// `history` is a face→face relation and has no entry for the solid, whose
		// TShape id these ops change. Carry it across by hand.
		if let Some(&color) = self.colormap.get(&ffi::shape_tshape_id(&self.inner)) {
			colormap.insert(ffi::shape_tshape_id(new_inner), color);
		}
		colormap
	}

	// ==================== Constructors ====================

	/// Returns `true` if this solid wraps a null shape.
	pub fn is_null(&self) -> bool {
		ffi::shape_is_null(&self.inner)
	}
}

impl EdgeBlendSession {
	/// Rebuild this prepared edge blend with a new kind and size.
	///
	/// The source shape and selected edge handles remain live for the session's
	/// lifetime. Only the OCCT builder and result history are recreated.
	pub fn update(&self, kind: EdgeBlendKind, size: f64, progress: &ffi::CancellationToken) -> Result<Solid, Error> {
		let edges = self.edge_indices.iter().map(|index| self.source.iter_edge().nth(*index as usize).ok_or_else(|| Error::InvalidEdge("a prepared edge blend no longer matches its source topology".into()))).collect::<Result<Vec<_>, _>>()?;
		match kind {
			EdgeBlendKind::Fillet => self.source.fillet_edges_cancelable(size, edges, progress),
			EdgeBlendKind::Chamfer => self.source.chamfer_edges_cancelable(size, edges, progress),
		}
	}

	/// Return the process-local source occurrence identity for diagnostics.
	pub fn source_id(&self) -> u64 {
		self.source.id()
	}
}

impl ExtrusionSession {
	/// Prepare a profile once so repeated extrusion updates do not copy or
	/// resolve profile edges for every pointer sample.
	pub fn prepare<'a>(profile: impl IntoIterator<Item = &'a Edge>) -> Result<Self, Error> {
		let profile = profile.into_iter().map(Edge::shared_copy).collect::<Vec<_>>();
		if profile.is_empty() {
			return Err(Error::InvalidEdge("an extrusion profile needs at least one edge".into()));
		}
		Ok(Self { profile })
	}

	/// Rebuild the prepared profile with a new extrusion direction.
	pub fn update(&self, direction: DVec3, progress: &ffi::CancellationToken) -> Result<Solid, Error> {
		Solid::extrude_cancelable(&self.profile, direction, progress)
	}
}

impl SweepSession {
	pub fn prepare<'a>(profile: impl IntoIterator<Item = &'a Edge>) -> Result<Self, Error> {
		let profile = profile.into_iter().map(Edge::shared_copy).collect::<Vec<_>>();
		if profile.is_empty() {
			return Err(Error::InvalidEdge("a sweep profile needs at least one edge".into()));
		}
		Ok(Self { profile })
	}

	pub fn update<'a, 'b>(&self, spine: impl IntoIterator<Item = &'a Edge>, orient: ProfileOrient<'b>, progress: &ffi::CancellationToken) -> Result<Solid, Error> {
		Solid::sweep_cancelable(&self.profile, spine, orient, progress)
	}
}

impl FaceEditSession {
	pub fn source(&self) -> &Solid {
		&self.source
	}

	pub fn face(&self) -> Result<&Face, Error> {
		self.source.iter_face().nth(self.face_index as usize).ok_or(Error::TopologyQueryFailed)
	}

	pub fn boundary(&self) -> &[Edge] {
		&self.boundary
	}

	pub fn source_id(&self) -> u64 {
		self.source.id()
	}
}

impl ShellSession {
	pub fn update(&self, thickness: f64, progress: &ffi::CancellationToken) -> Result<Solid, Error> {
		let faces = self.open_face_indices.iter().map(|index| self.source.iter_face().nth(*index as usize).ok_or(Error::TopologyQueryFailed)).collect::<Result<Vec<_>, _>>()?;
		self.source.shell_cancelable(thickness, faces, progress)
	}

	pub fn source_id(&self) -> u64 {
		self.source.id()
	}
}

pub(crate) fn topology_snapshot_from_shape(shape: &ffi::TopoDS_Shape) -> Result<TopologySnapshot, Error> {
	topology_snapshot_from_shape_with_options(shape, TopologyQueryOptions::default())
}

fn topology_snapshot_from_shape_with_options(shape: &ffi::TopoDS_Shape, options: TopologyQueryOptions) -> Result<TopologySnapshot, Error> {
	ffi::begin_operation();
	let data = ffi::shape_topology(shape, options.bits());
	if !data.success {
		return Err(ffi::operation_error(Error::TopologyQueryFailed, "topology snapshot", "topology_snapshot"));
	}
	let face_edges = decode_adjacency(&data.face_edge_offsets, &data.face_edge_indices, data.face_tshape_ids.len(), data.edge_tshape_ids.len())?;
	let edge_faces = decode_adjacency(&data.edge_face_offsets, &data.edge_face_indices, data.edge_tshape_ids.len(), data.face_tshape_ids.len())?;
	let edge_vertices = decode_adjacency(&data.edge_vertex_offsets, &data.edge_vertex_indices, data.edge_tshape_ids.len(), data.vertex_tshape_ids.len())?;
	let face_tokens = decode_tokens(&data.face_tshape_ids, &data.face_location_hashes, &data.face_orientations)?;
	let edge_tokens = decode_tokens(&data.edge_tshape_ids, &data.edge_location_hashes, &data.edge_orientations)?;
	let vertex_tokens = decode_tokens(&data.vertex_tshape_ids, &data.vertex_location_hashes, &data.vertex_orientations)?;
	let face_facts = decode_face_facts(&data, &face_tokens, options)?;
	let edge_facts = decode_edge_facts(&data, &edge_tokens, &edge_vertices, options)?;
	let vertex_facts = decode_vertex_facts(&data, &vertex_tokens, options)?;
	Ok(TopologySnapshot { face_ids: data.face_tshape_ids, edge_ids: data.edge_tshape_ids, vertex_ids: data.vertex_tshape_ids, face_edges, edge_faces, edge_vertices, face_facts, edge_facts, vertex_facts })
}

fn decode_tokens(ids: &[u64], locations: &[u64], orientations: &[u32]) -> Result<Vec<TopologyOccurrenceToken>, Error> {
	if locations.len() != ids.len() || orientations.len() != ids.len() {
		return Err(Error::TopologyQueryFailed);
	}
	ids.iter().zip(locations).zip(orientations).enumerate().map(|(ordinal, ((tshape_id, location_hash), orientation))| Ok(TopologyOccurrenceToken { tshape_id: *tshape_id, location_hash: *location_hash, orientation: *orientation, ordinal: u32::try_from(ordinal).map_err(|_| Error::TopologyQueryFailed)? })).collect()
}

fn decode_face_facts(data: &ffi::TopologyData, tokens: &[TopologyOccurrenceToken], options: TopologyQueryOptions) -> Result<Vec<FaceTopologyFacts>, Error> {
	let count = tokens.len();
	if (options.frames || options.measurements || options.geometry) && (data.face_geometry_kinds.len() != count || data.face_fact_flags.len() != count || data.face_points.len() != count * 3 || data.face_normals.len() != count * 3 || data.face_areas.len() != count) {
		return Err(Error::TopologyQueryFailed);
	}
	tokens
		.iter()
		.enumerate()
		.map(|(index, token)| {
			let flags = data.face_fact_flags.get(index).copied().unwrap_or(0);
			Ok(FaceTopologyFacts {
				token: *token,
				geometry: data.face_geometry_kinds.get(index).copied().map(decode_surface_kind).transpose()?.flatten(),
				representative_point: ((flags & 1) != 0).then(|| decode_point(&data.face_points, index)).transpose()?,
				normal: ((flags & 1) != 0).then(|| decode_point(&data.face_normals, index)).transpose()?,
				area: ((flags & 2) != 0).then(|| data.face_areas[index]),
				closed: (flags & 4) != 0,
			})
		})
		.collect()
}

fn decode_edge_facts(data: &ffi::TopologyData, tokens: &[TopologyOccurrenceToken], edge_vertices: &[Vec<u32>], options: TopologyQueryOptions) -> Result<Vec<EdgeTopologyFacts>, Error> {
	let count = tokens.len();
	if (options.frames || options.measurements || options.geometry) && (data.edge_geometry_kinds.len() != count || data.edge_fact_flags.len() != count || data.edge_points.len() != count * 3 || data.edge_tangents.len() != count * 3 || data.edge_directions.len() != count * 3 || data.edge_lengths.len() != count) {
		return Err(Error::TopologyQueryFailed);
	}
	tokens
		.iter()
		.enumerate()
		.map(|(index, token)| {
			let flags = data.edge_fact_flags.get(index).copied().unwrap_or(0);
			Ok(EdgeTopologyFacts {
				token: *token,
				geometry: data.edge_geometry_kinds.get(index).copied().map(decode_curve_kind).transpose()?.flatten(),
				midpoint: ((flags & 1) != 0).then(|| decode_point(&data.edge_points, index)).transpose()?,
				tangent: ((flags & 1) != 0).then(|| decode_point(&data.edge_tangents, index)).transpose()?,
				outward_direction: ((flags & 64) != 0).then(|| decode_point(&data.edge_directions, index)).transpose()?,
				length: ((flags & 2) != 0).then(|| data.edge_lengths[index]),
				vertices: edge_vertices[index].clone(),
				closed: (flags & 4) != 0,
				degenerate: (flags & 8) != 0,
				seam: (flags & 16) != 0,
				manifold: (flags & 32) != 0,
			})
		})
		.collect()
}

fn decode_vertex_facts(data: &ffi::TopologyData, tokens: &[TopologyOccurrenceToken], options: TopologyQueryOptions) -> Result<Vec<VertexTopologyFacts>, Error> {
	if options.frames && (data.vertex_fact_flags.len() != tokens.len() || data.vertex_points.len() != tokens.len() * 3) {
		return Err(Error::TopologyQueryFailed);
	}
	tokens
		.iter()
		.enumerate()
		.map(|(index, token)| {
			let flags = data.vertex_fact_flags.get(index).copied().unwrap_or(0);
			Ok(VertexTopologyFacts { token: *token, point: ((flags & 1) != 0).then(|| decode_point(&data.vertex_points, index)).transpose()? })
		})
		.collect()
}

fn decode_point(values: &[f64], index: usize) -> Result<[f64; 3], Error> {
	let start = index.checked_mul(3).ok_or(Error::TopologyQueryFailed)?;
	let point: [f64; 3] = values.get(start..start + 3).ok_or(Error::TopologyQueryFailed)?.try_into().map_err(|_| Error::TopologyQueryFailed)?;
	if point.into_iter().all(f64::is_finite) {
		Ok(point)
	} else {
		Err(Error::TopologyQueryFailed)
	}
}

fn decode_surface_kind(value: u32) -> Result<Option<SurfaceGeometryKind>, Error> {
	Ok(match value {
		0 => None,
		1 => Some(SurfaceGeometryKind::Plane),
		2 => Some(SurfaceGeometryKind::Cylinder),
		3 => Some(SurfaceGeometryKind::Cone),
		4 => Some(SurfaceGeometryKind::Sphere),
		5 => Some(SurfaceGeometryKind::Torus),
		6 => Some(SurfaceGeometryKind::Bezier),
		7 => Some(SurfaceGeometryKind::BSpline),
		8 => Some(SurfaceGeometryKind::Revolution),
		9 => Some(SurfaceGeometryKind::Extrusion),
		10 => Some(SurfaceGeometryKind::Offset),
		11 => Some(SurfaceGeometryKind::Other),
		_ => return Err(Error::TopologyQueryFailed),
	})
}

fn decode_curve_kind(value: u32) -> Result<Option<CurveGeometryKind>, Error> {
	Ok(match value {
		0 => None,
		1 => Some(CurveGeometryKind::Line),
		2 => Some(CurveGeometryKind::Circle),
		3 => Some(CurveGeometryKind::Ellipse),
		4 => Some(CurveGeometryKind::Hyperbola),
		5 => Some(CurveGeometryKind::Parabola),
		6 => Some(CurveGeometryKind::Bezier),
		7 => Some(CurveGeometryKind::BSpline),
		8 => Some(CurveGeometryKind::Offset),
		9 => Some(CurveGeometryKind::Other),
		_ => return Err(Error::TopologyQueryFailed),
	})
}

fn decode_adjacency(offsets: &[u32], indices: &[u32], owner_count: usize, target_count: usize) -> Result<Vec<Vec<u32>>, Error> {
	if offsets.len() != owner_count + 1 || offsets.first() != Some(&0) || offsets.last().copied() != u32::try_from(indices.len()).ok() {
		return Err(Error::TopologyQueryFailed);
	}
	offsets
		.windows(2)
		.map(|bounds| {
			let start = bounds[0] as usize;
			let end = bounds[1] as usize;
			if start > end || end > indices.len() || indices[start..end].iter().any(|index| *index as usize >= target_count) {
				return Err(Error::TopologyQueryFailed);
			}
			Ok(indices[start..end].to_vec())
		})
		.collect()
}

fn decode_topology_kind(value: u32) -> Result<TopologyKind, Error> {
	match value {
		0 => Ok(TopologyKind::Face),
		1 => Ok(TopologyKind::Edge),
		2 => Ok(TopologyKind::Vertex),
		_ => Err(Error::TopologyQueryFailed),
	}
}

fn decode_topology_history(data: ffi::HistoryData) -> Result<TopologyHistory, Error> {
	if !data.success || !data.relations.len().is_multiple_of(6) || !data.deleted.len().is_multiple_of(3) || !data.unresolved.len().is_multiple_of(2) {
		return Err(Error::TopologyQueryFailed);
	}
	let mut relations = data
		.relations
		.chunks_exact(6)
		.map(|values| {
			let relation = match values[2] {
				0 => TopologyRelationKind::Unchanged,
				1 => TopologyRelationKind::Modified,
				2 => TopologyRelationKind::Generated,
				_ => return Err(Error::TopologyQueryFailed),
			};
			Ok(TopologyRelation {
				result: ResultTopology { kind: decode_topology_kind(values[0])?, index: values[1] },
				relation,
				source: InputTopology { operand: values[3], kind: decode_topology_kind(values[4])?, index: values[5] },
			})
		})
		.collect::<Result<Vec<_>, Error>>()?;
	relations.sort_unstable_by_key(|item| (item.result.kind as u8, item.result.index, item.source.operand, item.source.kind as u8, item.source.index, item.relation as u8));
	relations.dedup();

	let mut deleted = data.deleted.chunks_exact(3).map(|values| Ok(InputTopology { operand: values[0], kind: decode_topology_kind(values[1])?, index: values[2] })).collect::<Result<Vec<_>, Error>>()?;
	deleted.sort_unstable_by_key(|item| (item.operand, item.kind as u8, item.index));
	deleted.dedup();

	let mut unresolved = data.unresolved.chunks_exact(2).map(|values| Ok(ResultTopology { kind: decode_topology_kind(values[0])?, index: values[1] })).collect::<Result<Vec<_>, Error>>()?;
	unresolved.sort_unstable_by_key(|item| (item.kind as u8, item.index));
	unresolved.dedup();

	Ok(TopologyHistory { relations, deleted, unresolved })
}

fn empty_ffi_history() -> ffi::HistoryData {
	ffi::HistoryData { relations: Vec::new(), deleted: Vec::new(), unresolved: Vec::new(), success: false }
}

fn boolean_build_with_progress(b: &Boolean<Solid>, progress: &ffi::CancellationToken) -> Result<Vec<Solid>, Error> {
	let (solids, clauses) = (b.solids(), b.clauses());
	if solids.is_empty() || clauses.is_empty() {
		return Err(Error::OneFailed(0));
	}
	debug_assert!(clauses.last() == Some(&0), "clauses must be 0-terminated");

	let mut solid_vec = ffi::shape_vec_new();
	for solid in solids {
		ffi::shape_vec_push(solid_vec.pin_mut(), solid.inner());
	}
	let mut history = Vec::new();
	let mut topology_history = empty_ffi_history();
	ffi::begin_operation();
	let inner = ffi::builder_cells(&solid_vec, clauses, progress, &mut history, &mut topology_history);
	if inner.is_null() {
		return Err(if progress.is_cancelled() { Error::Cancelled } else { ffi::operation_error(Error::BooleanOperationFailed, "boolean", "occt_build") });
	}
	let topology_history = decode_topology_history(topology_history)?;

	#[cfg(feature = "color")]
	let colormap = {
		let mut map = std::collections::HashMap::new();
		for pair in history.chunks_exact(2) {
			for solid in solids {
				if let Some(&color) = solid.colormap.get(&pair[1]) {
					map.entry(pair[0]).or_insert(color);
					break;
				}
			}
		}
		map
	};

	#[cfg(feature = "color")]
	let solid_color = solids[0].colormap.get(&solids[0].id()).copied();
	let compound = CompoundShape::from_raw(
		inner,
		#[cfg(feature = "color")]
		colormap,
		history,
		topology_history,
	);
	#[cfg_attr(not(feature = "color"), allow(unused_mut))]
	let mut output = compound.decompose();
	#[cfg(feature = "color")]
	if let Some(color) = solid_color {
		for solid in &mut output {
			let id = solid.id();
			solid.colormap_mut().insert(id, color);
		}
	}
	Ok(output)
}

impl SolidStruct for Solid {
	type Edge = Edge;
	type Face = Face;

	// ==================== Identity ====================

	fn id(&self) -> u64 {
		ffi::shape_tshape_id(&self.inner)
	}

	// ==================== Constructors ====================

	fn cube(corner0: DVec3, corner1: DVec3) -> Solid {
		Solid::try_cube(corner0, corner1).expect("Solid::cube requires finite non-degenerate corners")
	}

	fn cylinder(r: f64, height: DVec3) -> Solid {
		Solid::try_cylinder(r, height).expect("Solid::cylinder requires a positive radius and finite nonzero height")
	}

	fn sphere(radius: f64) -> Solid {
		Solid::try_sphere(radius).expect("Solid::sphere requires a finite positive radius")
	}

	fn cone(r1: f64, r2: f64, height: DVec3) -> Solid {
		Solid::try_cone(r1, r2, height).expect("Solid::cone requires valid radii and finite nonzero height")
	}

	fn torus(r1: f64, r2: f64, axis: DVec3) -> Solid {
		Solid::try_torus(r1, r2, axis).expect("Solid::torus requires 0 < minor radius < major radius and a nonzero axis")
	}

	fn half_space(plane_origin: DVec3, plane_normal: DVec3) -> Solid {
		Solid::try_half_space(plane_origin, plane_normal).expect("Solid::half_space requires a finite origin and nonzero normal")
	}

	// ==================== Topology iteration ====================
	//
	// `iter_edge` / `iter_face` lazily populate `OnceLock<Vec<T>>` caches on
	// first call. Subsequent calls return the cached vector's slice iter.
	// Topology-changing ops construct a fresh `Solid` via `Solid::new` so
	// these caches are invalidated automatically (new instance → fresh
	// `OnceLock::new()`). See `notes/20260420-OCCTトポロジ不変性と設計含意.md`.

	fn iter_edge(&self) -> impl Iterator<Item = &Edge> + '_ {
		self.edges
			.get_or_init(|| {
				ffi::shape_edges(&self.inner)
					.iter()
					.map(|e_ref| {
						let owned = ffi::clone_edge_handle(e_ref);
						Edge::try_from_ffi(owned, "shape_edges: null".into()).expect("shape_edges: unexpected null (this is a bug)")
					})
					.collect()
			})
			.iter()
	}

	fn iter_face(&self) -> impl Iterator<Item = &Face> + '_ {
		self.faces.get_or_init(|| ffi::shape_faces(&self.inner).iter().map(|f_ref| Face::new(ffi::clone_face_handle(f_ref))).collect()).iter()
	}

	fn iter_history(&self) -> impl Iterator<Item = [u64; 2]> + '_ {
		self.history.chunks_exact(2).map(|c| [c[0], c[1]])
	}

	// ==================== Extrude ====================

	fn extrude<'a>(profile: impl IntoIterator<Item = &'a Edge>, dir: DVec3) -> Result<Self, Error> {
		Self::extrude_cancelable(profile, dir, &ffi::CancellationToken::new())
	}

	// ==================== Shell ====================

	fn shell<'a>(&self, thickness: f64, open_faces: impl IntoIterator<Item = &'a Face>) -> Result<Self, Error> {
		self.shell_cancelable(thickness, open_faces, &ffi::CancellationToken::new())
	}

	// ==================== Fillet / Chamfer ====================

	fn fillet_edges<'a>(&self, radius: f64, edges: impl IntoIterator<Item = &'a Edge>) -> Result<Self, Error> {
		self.fillet_edges_cancelable(radius, edges, &ffi::CancellationToken::new())
	}

	fn chamfer_edges<'a>(&self, distance: f64, edges: impl IntoIterator<Item = &'a Edge>) -> Result<Self, Error> {
		self.chamfer_edges_cancelable(distance, edges, &ffi::CancellationToken::new())
	}

	// ==================== Sweep ====================

	fn sweep<'a, 'b, 'c>(profile: impl IntoIterator<Item = &'a Edge>, spine: impl IntoIterator<Item = &'b Edge>, orient: ProfileOrient<'c>) -> Result<Self, Error> {
		Self::sweep_cancelable(profile, spine, orient, &ffi::CancellationToken::new())
	}

	// ==================== Loft / ThruSections ====================

	fn loft<'a, I: IntoIterator<Item = &'a Edge>, S: IntoIterator<Item = I>>(sections: S, ruled: bool) -> Result<Self, Error>
	where
		Edge: 'a,
	{
		Self::loft_cancelable(sections, ruled, &ffi::CancellationToken::new())
	}

	// ==================== Sew ====================

	fn sew<'a>(faces: impl IntoIterator<Item = &'a Face>, tolerance: f64) -> Result<Self, Error>
	where
		Face: 'a,
	{
		let mut face_vec = ffi::face_vec_new();
		let mut count = 0usize;
		for f in faces {
			ffi::face_vec_push(face_vec.pin_mut(), &f.inner);
			count += 1;
		}
		if count == 0 {
			return Err(Error::SewFailed("sew: no faces given (need a face set forming one closed shell)".into()));
		}
		let shape = ffi::make_sewn_solid(&face_vec, tolerance);
		if shape.is_null() {
			return Err(Error::SewFailed(format!(
				"sew: {} faces do not form exactly one closed shell within tolerance {} \
				 (gaps, overlaps, multiple shells, or stray faces)",
				count, tolerance
			)));
		}
		Ok(Solid::new(
			shape,
			#[cfg(feature = "color")]
			std::collections::HashMap::new(),
			Default::default(),
		))
	}

	// ==================== Offset surface ====================

	fn offset_surface(&self, offset: f64, tolerance: f64) -> Result<Self, Error> {
		let shape = ffi::make_offset_shape(&self.inner, offset, tolerance);
		if shape.is_null() {
			return Err(Error::OffsetFailed(format!(
				"offset_surface: OCCT BRepOffsetAPI_MakeOffsetShape failed (offset={}, tolerance={}). \
				 Thin walls/slots whose local thickness is ≤ 2|offset| self-intersect and are rejected.",
				offset, tolerance
			)));
		}
		Ok(Solid::new(
			shape,
			#[cfg(feature = "color")]
			std::collections::HashMap::new(),
			Default::default(),
		))
	}

	// ==================== Bspline ====================

	fn bspline(u: usize, v: usize, u_periodic: bool, point: impl Fn(usize, usize) -> DVec3) -> Result<Self, Error> {
		if u < 2 || v < 3 {
			return Err(Error::BsplineFailed(format!("grid must be at least 2x3 (u={}, v={})", u, v)));
		}

		let mut coords = Vec::with_capacity(3 * u * v);
		for i in 0..u {
			for j in 0..v {
				let p = point(i, j);
				coords.push(p.x);
				coords.push(p.y);
				coords.push(p.z);
			}
		}

		let shape = ffi::make_bspline_solid(&coords, u as u32, v as u32, u_periodic);
		if shape.is_null() {
			return Err(Error::BsplineFailed(format!("OCCT construction failed (u={}, v={}, u_periodic={})", u, v, u_periodic)));
		}
		Ok(Solid::new(
			shape,
			#[cfg(feature = "color")]
			std::collections::HashMap::new(),
			Default::default(),
		))
	}

	// ==================== Clean ====================

	fn clean(&self) -> Result<Self, Error> {
		let mut history: Vec<u64> = Default::default();
		let mut topology_history = empty_ffi_history();
		let inner = ffi::builder_clean(&self.inner, &mut history, &mut topology_history);
		if inner.is_null() {
			return Err(Error::CleanFailed);
		}
		let topology_history = decode_topology_history(topology_history)?;
		#[cfg(feature = "color")]
		let colormap = self.remap_colormap(&inner, &history);
		Ok(Solid::new(
			inner,
			#[cfg(feature = "color")]
			colormap,
			history,
		)
		.with_topology_history(topology_history))
	}

	// ==================== Boolean primitive ====================

	fn boolean<'a>(solids: impl IntoIterator<Item = &'a Self>, clauses: impl IntoIterator<Item = i64>) -> Boolean<Self>
	where
		Self: 'a,
	{
		// TShape* を共有する shallow copy で Boolean を組む。Solid::clone() (=
		// BRepBuilderAPI_Copy) と違い、各 face の id() が元と一致するため
		// boolean 結果の history (post_id, src_id) を呼び出し側の face id と
		// 照合できる。
		let solids: Vec<Solid> = solids
			.into_iter()
			.map(|s| Solid {
				inner: ffi::clone_shape_handle(&s.inner),
				edges: OnceLock::new(),
				faces: OnceLock::new(),
				#[cfg(feature = "color")]
				colormap: s.colormap.clone(),
				history: s.history.clone(),
				topology_history: s.topology_history.clone(),
			})
			.collect();
		Boolean::from_parts(solids, clauses.into_iter().collect())
	}
	fn boolean_build(b: &Boolean<Self>) -> Result<Vec<Self>, Error> {
		boolean_build_with_progress(b, &ffi::CancellationToken::new())
	}

	// --- I/O (delegates to super::io helpers) ---

	fn read_step<R: std::io::Read>(reader: &mut R) -> Result<Vec<Self>, Error> {
		super::io::read_step(reader)
	}

	fn read_brep<R: std::io::Read>(reader: &mut R) -> Result<Vec<Self>, Error> {
		super::io::read_brep(reader)
	}

	fn write_step<'a, W: std::io::Write>(solids: impl IntoIterator<Item = &'a Self>, writer: &mut W) -> Result<(), Error>
	where
		Self: 'a,
	{
		super::io::write_step(solids, writer)
	}

	fn write_brep<'a, W: std::io::Write>(solids: impl IntoIterator<Item = &'a Self>, writer: &mut W) -> Result<(), Error>
	where
		Self: 'a,
	{
		super::io::write_brep(solids, writer)
	}

	fn mesh<'a>(solids: impl IntoIterator<Item = &'a Self>, options: crate::traits::Tessellation) -> Result<crate::common::mesh::Mesh, Error>
	where
		Self: 'a,
	{
		super::io::mesh(solids, options)
	}

	// ==================== Queries ====================

	fn volume(&self) -> f64 {
		ffi::shape_volume(&self.inner)
	}

	fn area(&self) -> f64 {
		ffi::shape_surface_area(&self.inner)
	}

	fn center(&self) -> DVec3 {
		let (mut x, mut y, mut z) = (0.0_f64, 0.0_f64, 0.0_f64);
		ffi::shape_center_of_mass(&self.inner, &mut x, &mut y, &mut z);
		DVec3::new(x, y, z)
	}

	fn inertia(&self) -> glam::DMat3 {
		let (mut m00, mut m01, mut m02) = (0.0_f64, 0.0_f64, 0.0_f64);
		let (mut m10, mut m11, mut m12) = (0.0_f64, 0.0_f64, 0.0_f64);
		let (mut m20, mut m21, mut m22) = (0.0_f64, 0.0_f64, 0.0_f64);
		ffi::shape_inertia_tensor(&self.inner, &mut m00, &mut m01, &mut m02, &mut m10, &mut m11, &mut m12, &mut m20, &mut m21, &mut m22);
		// OCCT fills row-major; DMat3::from_cols_array is column-major so
		// transpose when handing the components over.
		glam::DMat3::from_cols_array(&[m00, m10, m20, m01, m11, m21, m02, m12, m22])
	}

	fn contains(&self, point: DVec3) -> bool {
		ffi::shape_contains_point(&self.inner, point.x, point.y, point.z)
	}

	fn bounding_box(&self) -> [DVec3; 2] {
		let (mut xmin, mut ymin, mut zmin) = (0.0_f64, 0.0_f64, 0.0_f64);
		let (mut xmax, mut ymax, mut zmax) = (0.0_f64, 0.0_f64, 0.0_f64);
		ffi::shape_bounding_box(&self.inner, &mut xmin, &mut ymin, &mut zmin, &mut xmax, &mut ymax, &mut zmax);
		[DVec3::new(xmin, ymin, zmin), DVec3::new(xmax, ymax, zmax)]
	}

	// ==================== Color ====================

	#[cfg(feature = "color")]
	fn color(self, color: impl Into<crate::common::color::Color>) -> Self {
		let c = color.into();
		// Existing face colours are dropped: painting the whole solid is a statement
		// about the whole solid.
		let colormap = std::collections::HashMap::from([(ffi::shape_tshape_id(&self.inner), c)]);
		Self::new(self.inner, colormap, self.history).with_topology_history(self.topology_history)
	}

	#[cfg(feature = "color")]
	fn color_clear(self) -> Self {
		Self::new(self.inner, std::collections::HashMap::new(), self.history).with_topology_history(self.topology_history)
	}
}

// ==================== impl Transform for Solid ====================

impl Transform for Solid {
	fn translate(self, translation: DVec3) -> Self {
		let inner = ffi::transform_translate(&self.inner, translation.x, translation.y, translation.z);
		// translate/rotate use shape.Moved() — TShape is shared but Location
		// changes, so cached edges/faces (which embed Location) would go stale.
		// Solid::new gives a fresh OnceLock::new() cache matching the new Location.
		// `history` is preserved because TShape* (= post_id) is unchanged.
		Solid::new(
			inner,
			#[cfg(feature = "color")]
			self.colormap,
			self.history,
		)
		.with_topology_history(self.topology_history)
	}

	fn rotate(self, axis_origin: DVec3, axis_direction: DVec3, angle: f64) -> Self {
		let inner = ffi::transform_rotate(&self.inner, axis_origin.x, axis_origin.y, axis_origin.z, axis_direction.x, axis_direction.y, axis_direction.z, angle);
		Solid::new(
			inner,
			#[cfg(feature = "color")]
			self.colormap,
			self.history,
		)
		.with_topology_history(self.topology_history)
	}

	// scale/mirror consume self for API consistency, but internally clone the geometry.
	// Unlike translate/rotate which use gp_Trsf + shape.Moved() (preserving TShape),
	// scale/mirror cannot use Moved(): since OCCT Fix 0027457 (v7.6), TopLoc_Location
	// rejects gp_Trsf with scale != 1 or negative determinant, because downstream
	// algorithms (meshing, booleans) break on non-rigid transforms in locations.
	// Therefore BRepBuilderAPI_Transform is required, which rebuilds topology.
	// C++ impl: cpp/wrapper.cpp transform_scale() / transform_mirror()
	// See: https://dev.opencascade.org/content/how-scale-or-mirror-shape
	//      BRepBuilderAPI_Transform.cxx:48-49 (myUseModif branch)

	fn scale(self, center: DVec3, factor: f64) -> Self {
		let mut topology_history = empty_ffi_history();
		let inner = ffi::transform_scale(&self.inner, center.x, center.y, center.z, factor, &mut topology_history);
		#[cfg(feature = "color")]
		let colormap = remap_colormap_by_order(&self.inner, &inner, &self.colormap);
		let topology_history = decode_topology_history(topology_history).unwrap_or_default();
		Solid::new(
			inner,
			#[cfg(feature = "color")]
			colormap,
			Default::default(),
		)
		.with_topology_history(topology_history)
	}

	fn mirror(self, plane_origin: DVec3, plane_normal: DVec3) -> Self {
		let mut topology_history = empty_ffi_history();
		let inner = ffi::transform_mirror(&self.inner, plane_origin.x, plane_origin.y, plane_origin.z, plane_normal.x, plane_normal.y, plane_normal.z, &mut topology_history);
		#[cfg(feature = "color")]
		let colormap = remap_colormap_by_order(&self.inner, &inner, &self.colormap);
		let topology_history = decode_topology_history(topology_history).unwrap_or_default();
		Solid::new(
			inner,
			#[cfg(feature = "color")]
			colormap,
			Default::default(),
		)
		.with_topology_history(topology_history)
	}
}
