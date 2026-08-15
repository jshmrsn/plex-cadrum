use cadrum::{DVec3, Edge, EdgeBlendKind, Solid, TopologyKind, TopologyQueryOptions, TopologyRelationKind};

fn kind(kind: TopologyKind) -> u8 {
	match kind {
		TopologyKind::Face => 0,
		TopologyKind::Edge => 1,
		TopologyKind::Vertex => 2,
	}
}

fn relation(kind: TopologyRelationKind) -> u8 {
	match kind {
		TopologyRelationKind::Unchanged => 0,
		TopologyRelationKind::Modified => 1,
		TopologyRelationKind::Generated => 2,
	}
}

fn semantic_births(solid: &Solid) -> Vec<(u8, u8, u32, u8, u32)> {
	let mut births = solid.topology_history().relations().iter().map(|item| (kind(item.result.kind), relation(item.relation), item.source.operand, kind(item.source.kind), item.source.index)).collect::<Vec<_>>();
	births.sort_unstable();
	births
}

#[test]
fn edge_blend_parameter_sweep_preserves_semantic_birth_relations() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(20.0));
	let session = cube.prepare_edge_blend(&[0, 1]).expect("prepare blend");
	let cancellation = cadrum::CancellationToken::new();
	let baseline = session.update(EdgeBlendKind::Fillet, 0.5, &cancellation).expect("initial fillet");
	let baseline_births = semantic_births(&baseline);

	for radius in [0.75, 1.0, 1.5, 2.0] {
		let result = session.update(EdgeBlendKind::Fillet, radius, &cancellation).expect("fillet sweep");
		assert_eq!(semantic_births(&result), baseline_births, "radius {radius}");
	}
	let chamfer = session.update(EdgeBlendKind::Chamfer, 1.0, &cancellation).expect("switch to chamfer");
	assert!(semantic_births(&chamfer).iter().any(|birth| birth.1 == 2 && birth.3 == 1));
}

#[test]
fn extrusion_distance_sweep_preserves_profile_generated_relations() {
	let profile = Edge::polygon(&[DVec3::new(-4.0, -3.0, 0.0), DVec3::new(4.0, -3.0, 0.0), DVec3::new(4.0, 3.0, 0.0), DVec3::new(-4.0, 3.0, 0.0)]).expect("profile");
	let session = cadrum::ExtrusionSession::prepare(&profile).expect("prepare extrusion");
	let cancellation = cadrum::CancellationToken::new();
	let baseline = session.update(DVec3::Z, &cancellation).expect("initial extrusion");
	let baseline_births = semantic_births(&baseline);
	for distance in [2.0, 4.0, 8.0, 16.0] {
		let result = session.update(DVec3::Z * distance, &cancellation).expect("extrusion sweep");
		assert_eq!(semantic_births(&result), baseline_births, "distance {distance}");
	}
}

#[test]
fn occurrence_tokens_include_location_and_orientation() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(10.0));
	let original = cube.topology_snapshot_with_options(TopologyQueryOptions::INTERACTION).expect("original topology");
	let located = cube.located(DVec3::Z, 0.0, DVec3::X * 30.0);
	let moved = located.topology_snapshot_with_options(TopologyQueryOptions::INTERACTION).expect("located topology");

	for face in 0..original.face_ids().len() as u32 {
		let before = original.face_facts(face).expect("original face").token;
		let after = moved.face_facts(face).expect("moved face").token;
		assert_eq!(after.tshape_id, before.tshape_id);
		assert_eq!(after.orientation, before.orientation);
		assert_ne!(after.location_hash, before.location_hash);
		assert_eq!(after.ordinal, before.ordinal);
	}
}
