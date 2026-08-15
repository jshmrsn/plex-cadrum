use cadrum::{Boolean, DVec3, Edge, EdgeBlendKind, Solid, TopologyKind, TopologyQueryOptions, TopologyRelationKind};

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

#[test]
fn revolution_sweep_preserves_profile_birth_relations() {
	let profile = Edge::polygon(&[DVec3::new(3.0, 0.0, -1.0), DVec3::new(5.0, 0.0, -1.0), DVec3::new(5.0, 0.0, 1.0), DVec3::new(3.0, 0.0, 1.0)]).expect("profile");
	let session = cadrum::SweepSession::prepare(&profile).expect("prepare revolution");
	let cancellation = cadrum::CancellationToken::new();
	let build = |angle: f64| {
		let spine = Edge::arc_3pts(DVec3::new(4.0, 0.0, 0.0), DVec3::new(4.0 * (angle * 0.5).cos(), 4.0 * (angle * 0.5).sin(), 0.0), DVec3::new(4.0 * angle.cos(), 4.0 * angle.sin(), 0.0)).expect("spine");
		session.update([&spine], cadrum::ProfileOrient::Up(DVec3::Z), &cancellation).expect("revolution")
	};
	let baseline = semantic_births(&build(0.5));
	for angle in [0.75, 1.0, 1.5] {
		let result = build(angle);
		assert!(result.topology_history().unresolved().is_empty(), "angle {angle}: {:?}", result.topology_history());
		assert_eq!(semantic_births(&result), baseline, "angle {angle}");
	}
}

#[test]
fn shell_thickness_sweep_preserves_source_relations() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(20.0));
	let open = cube.iter_face().next().expect("open face");
	let baseline = cube.shell(-0.5, [open]).expect("thin shell");
	let baseline_births = semantic_births(&baseline);
	for thickness in [-0.75, -1.0, -1.5] {
		let result = cube.shell(thickness, [open]).expect("shell sweep");
		assert!(result.topology_history().unresolved().is_empty(), "thickness {thickness}: {:?}", result.topology_history());
		assert_eq!(semantic_births(&result), baseline_births, "thickness {thickness}");
	}
}

#[test]
fn boolean_operand_relations_survive_tool_placement_sweep() {
	let target = Solid::cube(DVec3::new(-10.0, -10.0, 0.0), DVec3::new(10.0, 10.0, 20.0));
	let tool = Solid::cylinder(2.0, DVec3::Z * 20.0);
	let build = |offset: f64| {
		let placed = tool.located(DVec3::Z, 0.0, DVec3::X * offset);
		(Boolean::from(&target) - Boolean::from(&placed)).build_vec().expect("subtract").remove(0)
	};
	let baseline = semantic_births(&build(2.0));
	for offset in [3.0, 4.0, 5.0] {
		let result = build(offset);
		assert!(result.topology_history().unresolved().is_empty(), "offset {offset}");
		assert_eq!(semantic_births(&result), baseline, "offset {offset}");
	}
}

#[test]
fn brep_reload_rebuilds_equivalent_topology_facts_without_reusing_tokens() {
	let source = Solid::cube(DVec3::ZERO, DVec3::new(12.0, 8.0, 5.0));
	let source_facts = source.topology_snapshot_with_options(TopologyQueryOptions::MEASUREMENT).expect("source facts");
	let mut archive = Vec::new();
	Solid::write_brep([&source], &mut archive).expect("archive");
	let restored = Solid::read_brep(&mut archive.as_slice()).expect("reload").remove(0);
	let restored_facts = restored.topology_snapshot_with_options(TopologyQueryOptions::MEASUREMENT).expect("restored facts");

	assert_ne!(source_facts.face_ids(), restored_facts.face_ids());
	let mut source_areas = (0..source_facts.face_ids().len() as u32).map(|face| source_facts.face_facts(face).and_then(|facts| facts.area).expect("source area")).collect::<Vec<_>>();
	let mut restored_areas = (0..restored_facts.face_ids().len() as u32).map(|face| restored_facts.face_facts(face).and_then(|facts| facts.area).expect("restored area")).collect::<Vec<_>>();
	source_areas.sort_by(f64::total_cmp);
	restored_areas.sort_by(f64::total_cmp);
	assert_eq!(source_areas, restored_areas);
}
