use super::ffi;
use super::solid::{topology_snapshot_from_shape, Solid, TopologyHistory};
#[cfg(feature = "color")]
use crate::common::color::Color;

/// A compound shape wrapping multiple solids into a single `TopoDS_Compound`.
///
/// Provides type-safe distinction from individual `Solid` handles.
pub(crate) struct CompoundShape {
	inner: cxx::UniquePtr<ffi::TopoDS_Shape>,
	#[cfg(feature = "color")]
	colormap: std::collections::HashMap<u64, Color>,
	history: Vec<u64>,
	topology_history: TopologyHistory,
}

impl CompoundShape {
	/// Assemble solids into a compound, merging their colormaps.
	///
	/// Inputs' `history` is intentionally dropped — a compound assembled for
	/// a boolean call has no meaningful history of its own; the boolean
	/// result will populate one fresh.
	pub fn new<'a>(solids: impl IntoIterator<Item = &'a Solid>) -> Self {
		let mut inner = ffi::make_empty();
		#[cfg(feature = "color")]
		let mut colormap = std::collections::HashMap::new();
		for s in solids {
			ffi::compound_add(inner.pin_mut(), s.inner());
			#[cfg(feature = "color")]
			colormap.extend(s.colormap().iter().map(|(&k, &v)| (k, v)));
		}
		CompoundShape {
			inner,
			#[cfg(feature = "color")]
			colormap,
			history: Default::default(),
			topology_history: TopologyHistory::default(),
		}
	}

	/// Create a compound from a raw `TopoDS_Shape` (e.g. from I/O or boolean ops).
	pub fn from_raw(inner: cxx::UniquePtr<ffi::TopoDS_Shape>, #[cfg(feature = "color")] colormap: std::collections::HashMap<u64, Color>, history: Vec<u64>, topology_history: TopologyHistory) -> Self {
		CompoundShape {
			inner,
			#[cfg(feature = "color")]
			colormap,
			history,
			topology_history,
		}
	}

	/// Borrow the underlying `TopoDS_Shape`.
	pub fn inner(&self) -> &ffi::TopoDS_Shape {
		&self.inner
	}

	/// Borrow the merged colormap.
	#[cfg(feature = "color")]
	pub fn colormap(&self) -> &std::collections::HashMap<u64, Color> {
		&self.colormap
	}

	/// Decompose into individual solids, consuming the compound.
	///
	/// Result-local topology relations are filtered and re-indexed, so sibling
	/// solids cannot claim provenance for one another.
	pub fn decompose(self) -> Vec<Solid> {
		let solid_shapes = ffi::decompose_into_solids(&self.inner);
		let global_topology = topology_snapshot_from_shape(&self.inner).ok();
		solid_shapes
			.iter()
			.map(|s| {
				let local_topology = topology_snapshot_from_shape(s).ok();
				let history = local_topology.as_ref().map_or_else(
					|| self.history.clone(),
					|topology| {
						let local_faces = topology.face_ids().iter().copied().collect::<std::collections::HashSet<_>>();
						self.history.chunks_exact(2).filter(|pair| local_faces.contains(&pair[0])).flatten().copied().collect()
					},
				);
				let topology_history = match (&global_topology, &local_topology) {
					(Some(global), Some(local)) => self.topology_history.remap_result_to(global, local),
					_ => TopologyHistory::default(),
				};
				Solid::new(
					ffi::clone_shape_handle(s),
					#[cfg(feature = "color")]
					self.colormap.clone(),
					history,
				)
				.with_topology_history(topology_history)
			})
			.collect()
	}
}
