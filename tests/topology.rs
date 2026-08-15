use cadrum::{DVec3, Solid};

#[test]
fn cube_topology_snapshot_is_bidirectionally_consistent() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(10.0));
	let topology = cube.topology_snapshot().expect("query cube topology");

	assert_eq!(topology.face_ids().len(), 6);
	assert_eq!(topology.edge_ids().len(), 12);
	for face in 0..topology.face_ids().len() as u32 {
		let edges = topology.face_edges(face).expect("face adjacency");
		assert_eq!(edges.len(), 4);
		for edge in edges {
			assert!(topology.edge_faces(*edge).expect("edge adjacency").contains(&face));
		}
	}
	for edge in 0..topology.edge_ids().len() as u32 {
		assert_eq!(topology.edge_faces(edge).expect("edge adjacency").len(), 2);
	}
}

#[test]
fn rigid_location_preserves_topology_tokens() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(10.0));
	let original = cube.topology_snapshot().expect("query original topology");
	let located = cube.located(DVec3::Z, std::f64::consts::FRAC_PI_2, DVec3::X * 20.0);
	let transformed = located.topology_snapshot().expect("query located topology");

	assert_eq!(transformed.face_ids(), original.face_ids());
	assert_eq!(transformed.edge_ids(), original.edge_ids());
	let bounds = located.bounding_box();
	assert!(bounds[0].distance(DVec3::new(10.0, 0.0, 0.0)) < 1.0e-6);
	assert!(bounds[1].distance(DVec3::new(20.0, 10.0, 10.0)) < 1.0e-6);
}
