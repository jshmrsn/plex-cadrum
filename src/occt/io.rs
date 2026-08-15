//! I/O helpers for `Solid`. Exposed via `impl SolidStruct for Solid` in
//! `super::solid` (e.g. `Solid::read_step`, `Solid::write_step`, `Solid::mesh`).

use super::compound::CompoundShape;
use super::ffi;
use super::ffi::{RustReader, RustWriter};
use super::solid::Solid;
use crate::common::error::Error;
use std::io::{Read, Write};

#[cfg(feature = "color")]
use crate::common::color::Color;

// ==================== Color trailer ====================
// Appended past the BinTools payload, which BinTools::Read stops at and ignores:
// `[b"CDCL"][u32 count][count x (u32 trailer_ids index, f32 r, f32 g, f32 b)]`, LE.

#[cfg(feature = "color")]
const COLOR_TRAILER_MAGIC: &[u8; 4] = b"CDCL";

/// `tail` is `&buf[consumed..]`, the bytes the BRep parser did not take. Anything that
/// is not our trailer yields an empty map — the geometry is valid either way.
#[cfg(feature = "color")]
fn read_color_trailer(tail: &[u8]) -> std::collections::HashMap<u32, Color> {
	let mut colormap = std::collections::HashMap::new();
	if tail.len() < 8 || &tail[..4] != COLOR_TRAILER_MAGIC {
		return colormap;
	}
	let count = u32::from_le_bytes(tail[4..8].try_into().unwrap()) as usize;
	// `count` comes from the file, and `usize` is 32-bit on wasm32.
	let Some(end) = count.checked_mul(16).and_then(|n| n.checked_add(8)) else {
		return colormap;
	};
	// `<`, not `!=`: the count self-delimits, so bytes appended after us are not an error.
	if tail.len() < end {
		return colormap;
	}
	for e in tail[8..end].chunks_exact(16) {
		let idx = u32::from_le_bytes(e[0..4].try_into().unwrap());
		let r = f32::from_le_bytes(e[4..8].try_into().unwrap());
		let g = f32::from_le_bytes(e[8..12].try_into().unwrap());
		let b = f32::from_le_bytes(e[12..16].try_into().unwrap());
		colormap.insert(idx, Color { r, g, b });
	}
	colormap
}

/// STEP cannot index like this — `try_sew_orphan_faces` shifts every index, so it
/// carries explicit ids instead.
#[cfg(feature = "color")]
fn trailer_ids(shape: &ffi::TopoDS_Shape) -> Vec<u64> {
	// Bound to locals: both are `UniquePtr<CxxVector<..>>` that the iterators borrow.
	let solids = ffi::decompose_into_solids(shape);
	let faces = ffi::shape_faces(shape);
	solids.iter().map(ffi::shape_tshape_id).chain(faces.iter().map(ffi::face_tshape_id)).collect()
}

#[cfg(feature = "color")]
fn write_color_trailer<W: Write>(compound: &CompoundShape, writer: &mut W) -> Result<(), Error> {
	let id_to_index: std::collections::HashMap<u64, u32> = trailer_ids(compound.inner()).into_iter().enumerate().map(|(i, id)| (id, i as u32)).collect();
	// `CompoundShape::decompose` gives every solid a clone of the merged colormap, so
	// a solid carries its siblings' keys too; those have no index and drop out here.
	let mut entries: Vec<(u32, f32, f32, f32)> = compound.colormap().iter().filter_map(|(id, rgb)| id_to_index.get(id).map(|&idx| (idx, rgb.r, rgb.g, rgb.b))).collect();
	if entries.is_empty() {
		return Ok(());
	}
	entries.sort_by_key(|e| e.0);

	let mut out = Vec::with_capacity(8 + entries.len() * 16);
	out.extend_from_slice(COLOR_TRAILER_MAGIC);
	out.extend_from_slice(&(entries.len() as u32).to_le_bytes());
	for (idx, r, g, b) in &entries {
		out.extend_from_slice(&idx.to_le_bytes());
		out.extend_from_slice(&r.to_le_bytes());
		out.extend_from_slice(&g.to_le_bytes());
		out.extend_from_slice(&b.to_le_bytes());
	}
	writer.write_all(&out).map_err(|_| Error::BrepWriteFailed)
}

// ==================== Reader / writer / mesh helpers ====================
//
// Each function is invoked by the matching `SolidStruct` method in
// `super::solid::Solid`. Kept module-private (`pub(super)`) so the public
// surface lives entirely on `Solid`.

pub(super) fn read_step<R: Read>(reader: &mut R) -> Result<Vec<Solid>, Error> {
	#[cfg(feature = "color")]
	{
		let mut rust_reader = RustReader::from_ref(reader);
		let mut ids: Vec<u64> = Default::default();
		let mut rgb: Vec<f32> = Default::default();
		ffi::begin_operation();
		let inner = ffi::read_step_color_stream(&mut rust_reader, &mut ids, &mut rgb);
		if inner.is_null() {
			return Err(ffi::operation_error(Error::StepReadFailed, "read STEP", "read"));
		}
		let colormap: std::collections::HashMap<u64, Color> = ids.into_iter().zip(rgb.chunks_exact(3)).map(|(id, c)| (id, Color { r: c[0], g: c[1], b: c[2] })).collect();
		Ok(CompoundShape::from_raw(inner, colormap, Default::default(), Default::default()).decompose())
	}
	#[cfg(not(feature = "color"))]
	{
		let mut rust_reader = RustReader::from_ref(reader);
		ffi::begin_operation();
		let inner = ffi::read_step_stream(&mut rust_reader);
		if inner.is_null() {
			return Err(ffi::operation_error(Error::StepReadFailed, "read STEP", "read"));
		}
		Ok(CompoundShape::from_raw(inner, Default::default(), Default::default()).decompose())
	}
}

pub(super) fn read_brep<R: Read>(reader: &mut R) -> Result<Vec<Solid>, Error> {
	// Buffered whole: `BinTools::Read` seeks backwards to resolve shared sub-shape
	// references, so it cannot run off a sequential stream.
	let mut buf = Vec::new();
	reader.read_to_end(&mut buf).map_err(|_| Error::BrepReadFailed)?;

	// Payload length — where a trailer would begin. Unwritten, and unread, on null.
	let mut consumed = 0usize;
	ffi::begin_operation();
	let inner = ffi::read_brep_stream(&buf, &mut consumed);
	if inner.is_null() {
		return Err(ffi::operation_error(Error::BrepReadFailed, "read B-rep", "read"));
	}

	#[cfg(feature = "color")]
	{
		let ids = trailer_ids(&inner);
		let colormap = read_color_trailer(buf.get(consumed..).unwrap_or_default()).into_iter().filter_map(|(idx, color)| ids.get(idx as usize).map(|&id| (id, color))).collect();
		Ok(CompoundShape::from_raw(inner, colormap, Default::default(), Default::default()).decompose())
	}
	#[cfg(not(feature = "color"))]
	{
		Ok(CompoundShape::from_raw(inner, Default::default(), Default::default()).decompose())
	}
}

/// Write solids to a STEP stream.
///
/// With the `color` feature enabled, face colors are automatically embedded
/// in the STEP file (XDE / AP214 styled items).
pub(super) fn write_step<'a, W: Write>(solids: impl IntoIterator<Item = &'a Solid>, writer: &mut W) -> Result<(), Error> {
	let compound = CompoundShape::new(solids);
	#[cfg(feature = "color")]
	{
		let colormap = compound.colormap();
		let mut ids: Vec<u64> = Vec::with_capacity(colormap.len());
		let mut rgb: Vec<f32> = Vec::with_capacity(colormap.len() * 3);
		for (&id, c) in colormap {
			ids.push(id);
			rgb.extend_from_slice(&[c.r, c.g, c.b]);
		}
		let mut rust_writer = RustWriter::from_ref(writer);
		ffi::begin_operation();
		if ffi::write_step_color_stream(compound.inner(), &ids, &rgb, &mut rust_writer) {
			Ok(())
		} else {
			Err(ffi::operation_error(Error::StepWriteFailed, "write STEP", "write"))
		}
	}
	#[cfg(not(feature = "color"))]
	{
		let mut rust_writer = RustWriter::from_ref(writer);
		ffi::begin_operation();
		if ffi::write_step_stream(compound.inner(), &mut rust_writer) {
			Ok(())
		} else {
			Err(ffi::operation_error(Error::StepWriteFailed, "write STEP", "write"))
		}
	}
}

pub(super) fn write_brep<'a, W: Write>(solids: impl IntoIterator<Item = &'a Solid>, writer: &mut W) -> Result<(), Error> {
	let compound = CompoundShape::new(solids);
	{
		// Scoped: the streambuf flushes on drop, so the payload lands before the trailer.
		let mut rust_writer = RustWriter::from_ref(writer);
		ffi::begin_operation();
		if !ffi::write_brep_stream(compound.inner(), &mut rust_writer) {
			return Err(ffi::operation_error(Error::BrepWriteFailed, "write B-rep", "write"));
		}
	}
	#[cfg(feature = "color")]
	write_color_trailer(&compound, writer)?;
	Ok(())
}

pub(super) fn mesh<'a>(solids: impl IntoIterator<Item = &'a Solid>, options: crate::traits::Tessellation) -> Result<crate::common::mesh::Mesh, Error> {
	use crate::common::mesh::Mesh;
	use glam::DVec3;

	#[cfg(feature = "color")]
	let solids: Vec<&Solid> = solids.into_iter().collect();
	// `Mesh` has only a face level, so a solid-level colour is expanded onto its faces
	// here. STEP and the BRep trailer keep the distinction; the renderers cannot.
	#[cfg(feature = "color")]
	let face_colors = {
		let mut map = std::collections::HashMap::new();
		for s in &solids {
			if let Some(&c) = s.colormap().get(&s.id()) {
				for f in ffi::shape_faces(s.inner()).iter() {
					map.insert(ffi::face_tshape_id(f), c);
				}
			}
			// Face colours are the more specific style and win over the solid's.
			map.extend(s.colormap().iter().map(|(&k, &v)| (k, v)));
		}
		map
	};

	let compound = CompoundShape::new(solids);
	let progress = ffi::CancellationToken::new();
	ffi::begin_operation();
	let data = ffi::mesh_shape(compound.inner(), options.deflection_linear, options.deflection_angular, options.relative_linear, options.parallel, &progress);
	if !data.success {
		return Err(ffi::operation_error(Error::TriangulationFailed, "mesh shape", "mesh"));
	}
	let vertex_count = data.vertices.len() / 3;
	let vertices: Vec<DVec3> = (0..vertex_count).map(|i| DVec3::new(data.vertices[i * 3], data.vertices[i * 3 + 1], data.vertices[i * 3 + 2])).collect();
	let normals: Vec<DVec3> = (0..vertex_count).map(|i| DVec3::new(data.normals[i * 3], data.normals[i * 3 + 1], data.normals[i * 3 + 2])).collect();
	let indices: Vec<usize> = data.indices.iter().map(|&i| i as usize).collect();
	let face_ids = data.face_tshape_ids;

	// Topological edge polylines, NaN-separated. Reuses the existing edge
	// discretizer (GCPnts_TangentialDeflection). `relative_linear` applies to
	// surface triangulation only; edges use `deflection_linear` as an absolute
	// chord here.
	let mut edges: Vec<DVec3> = Vec::new();
	if options.include_edges {
		for e in ffi::shape_edges(compound.inner()).iter() {
			let segs = ffi::edge_approximation_segments(e, options.deflection_linear, options.deflection_angular, options.relative_linear);
			if segs.len() < 6 {
				continue;
			}
			if !edges.is_empty() {
				edges.push(DVec3::NAN);
			}
			for c in segs.chunks_exact(3) {
				edges.push(DVec3::new(c[0], c[1], c[2]));
			}
		}
	}

	#[cfg(feature = "color")]
	let colormap = {
		let mut map = std::collections::HashMap::new();
		for &fid in &face_ids {
			if let Some(&color) = face_colors.get(&fid) {
				map.insert(fid, color);
			}
		}
		map
	};

	Ok(Mesh {
		vertices,
		normals,
		indices,
		face_ids,
		#[cfg(feature = "color")]
		colormap,
		edges,
	})
}

pub(super) fn mesh_chunks<'a>(solids: impl IntoIterator<Item = &'a Solid>, options: crate::traits::Tessellation) -> Result<crate::common::mesh::MeshChunks, Error> {
	mesh_chunks_cancelable(solids, options, &ffi::CancellationToken::new())
}

pub(super) fn mesh_chunks_cancelable<'a>(solids: impl IntoIterator<Item = &'a Solid>, options: crate::traits::Tessellation, progress: &ffi::CancellationToken) -> Result<crate::common::mesh::MeshChunks, Error> {
	let solids = solids.into_iter().collect::<Vec<_>>();
	let compound = CompoundShape::new(solids.iter().copied());
	ffi::begin_operation();
	let data = ffi::mesh_shape(compound.inner(), options.deflection_linear, options.deflection_angular, options.relative_linear, options.parallel, progress);
	if progress.is_cancelled() {
		return Err(Error::Cancelled);
	}
	decode_mesh_chunks(compound.inner(), data, options)
}

pub(super) fn mesh_face_chunks(solid: &Solid, face_indices: &[u32], options: crate::traits::Tessellation) -> Result<Vec<crate::common::mesh::FaceMeshChunk>, Error> {
	mesh_face_chunks_cancelable(solid, face_indices, options, &ffi::CancellationToken::new())
}

pub(super) fn mesh_face_chunks_cancelable(solid: &Solid, face_indices: &[u32], options: crate::traits::Tessellation, progress: &ffi::CancellationToken) -> Result<Vec<crate::common::mesh::FaceMeshChunk>, Error> {
	ffi::begin_operation();
	let data = ffi::mesh_shape_faces(solid.inner(), face_indices, options.deflection_linear, options.deflection_angular, options.relative_linear, options.parallel, progress);
	if progress.is_cancelled() {
		return Err(Error::Cancelled);
	}
	Ok(decode_mesh_chunks(solid.inner(), data, crate::traits::Tessellation { include_edges: false, ..options })?.faces)
}

pub(super) fn edge_polyline_chunks(solid: &Solid, options: crate::traits::Tessellation) -> Result<Vec<crate::common::mesh::EdgePolylineChunk>, Error> {
	edge_polyline_chunks_cancelable(solid, options, &ffi::CancellationToken::new())
}

pub(super) fn edge_polyline_chunks_cancelable(solid: &Solid, options: crate::traits::Tessellation, progress: &ffi::CancellationToken) -> Result<Vec<crate::common::mesh::EdgePolylineChunk>, Error> {
	use crate::common::mesh::EdgePolylineChunk;
	use glam::DVec3;

	ffi::shape_edges(solid.inner())
		.iter()
		.enumerate()
		.map(|(edge_index, edge)| {
			if progress.is_cancelled() {
				return Err(Error::Cancelled);
			}
			let points = ffi::edge_approximation_segments(edge, options.deflection_linear, options.deflection_angular, options.relative_linear).chunks_exact(3).map(|point| DVec3::new(point[0], point[1], point[2])).collect::<Vec<_>>();
			if points.len() < 2 {
				return Err(Error::TriangulationFailed);
			}
			Ok(EdgePolylineChunk { edge_index: u32::try_from(edge_index).map_err(|_| Error::TriangulationFailed)?, points })
		})
		.collect()
}

fn decode_mesh_chunks(shape: &ffi::TopoDS_Shape, data: ffi::MeshData, options: crate::traits::Tessellation) -> Result<crate::common::mesh::MeshChunks, Error> {
	use crate::common::mesh::{EdgePolylineChunk, FaceMeshChunk, MeshChunks};
	use glam::DVec3;

	let face_count = data.chunk_face_tshape_ids.len();
	if !data.success || !data.vertices.len().is_multiple_of(3) || data.normals.len() != data.vertices.len() || data.chunk_face_indices.len() != face_count || data.face_vertex_offsets.len() != face_count + 1 || data.face_index_offsets.len() != face_count + 1 {
		return Err(ffi::operation_error(Error::TriangulationFailed, "mesh shape", "mesh"));
	}
	let vertices = data.vertices.chunks_exact(3).map(|point| DVec3::new(point[0], point[1], point[2])).collect::<Vec<_>>();
	let normals = data.normals.chunks_exact(3).map(|normal| DVec3::new(normal[0], normal[1], normal[2])).collect::<Vec<_>>();
	let mut faces = Vec::with_capacity(face_count);
	for face_index in 0..face_count {
		let vertex_start = data.face_vertex_offsets[face_index] as usize;
		let vertex_end = data.face_vertex_offsets[face_index + 1] as usize;
		let index_start = data.face_index_offsets[face_index] as usize;
		let index_end = data.face_index_offsets[face_index + 1] as usize;
		if vertex_start > vertex_end || vertex_end > vertices.len() || index_start > index_end || index_end > data.indices.len() || !(index_end - index_start).is_multiple_of(3) || data.indices[index_start..index_end].iter().any(|index| (*index as usize) < vertex_start || (*index as usize) >= vertex_end) {
			return Err(Error::TriangulationFailed);
		}
		faces.push(FaceMeshChunk {
			face_index: data.chunk_face_indices[face_index],
			vertices: vertices[vertex_start..vertex_end].to_vec(),
			normals: normals[vertex_start..vertex_end].to_vec(),
			indices: data.indices[index_start..index_end].iter().map(|index| index - data.face_vertex_offsets[face_index]).collect(),
		});
	}

	let mut edges = Vec::new();
	if options.include_edges {
		for (edge_index, edge) in ffi::shape_edges(shape).iter().enumerate() {
			let points = ffi::edge_approximation_segments(edge, options.deflection_linear, options.deflection_angular, options.relative_linear).chunks_exact(3).map(|point| DVec3::new(point[0], point[1], point[2])).collect::<Vec<_>>();
			if points.len() >= 2 {
				edges.push(EdgePolylineChunk { edge_index: u32::try_from(edge_index).map_err(|_| Error::TriangulationFailed)?, points });
			}
		}
	}
	Ok(MeshChunks { faces, edges })
}
