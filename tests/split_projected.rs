use cadrum::{Edge, Solid};
use glam::DVec3;

#[test]
fn split_box_face_with_projected_circle() {
	// 20x20x10 box centred at origin; project a radius-4 circle hovering
	// above the top face straight down through the solid.
	let solid = Solid::cube(DVec3::new(-10.0, -10.0, -5.0), DVec3::new(10.0, 10.0, 5.0));
	let base_faces = solid.iter_face().count();

	let circle = Edge::circle(4.0, DVec3::Z).unwrap().translate(DVec3::new(0.0, 0.0, 30.0));
	let split = solid.split_with_projected_edges(std::slice::from_ref(&circle), DVec3::new(0.0, 0.0, -1.0)).unwrap();

	assert_eq!(split.len(), 1, "splitting faces must not divide the solid");
	let split_faces = split[0].iter_face().count();
	// The circle lands on both the top and bottom faces, carving a disc out
	// of each: at least two new faces.
	assert!(split_faces >= base_faces + 2, "expected face split: {base_faces} -> {split_faces}");
	let volume_ratio = split[0].volume() / solid.volume();
	assert!((volume_ratio - 1.0).abs() < 1.0e-9, "split must preserve volume, ratio {volume_ratio}");
}

#[test]
fn split_with_edge_missing_solid_fails() {
	let solid = Solid::cube(DVec3::new(-10.0, -10.0, -5.0), DVec3::new(10.0, 10.0, 5.0));
	// Circle far outside the box footprint: projection misses entirely.
	let circle = Edge::circle(4.0, DVec3::Z).unwrap().translate(DVec3::new(500.0, 0.0, 30.0));
	assert!(solid.split_with_projected_edges(std::slice::from_ref(&circle), DVec3::new(0.0, 0.0, -1.0)).is_err());
}

#[test]
fn ellipse_edge_has_expected_extents() {
	let edge = Edge::ellipse(5.0, 3.0, DVec3::X, DVec3::Z).unwrap();
	assert!(edge.is_closed());
	// Vertex extremes: the exact edge passes through (±5, 0, 0) and (0, ±3, 0).
	for p in [DVec3::new(5.0, 0.0, 0.0), DVec3::new(-5.0, 0.0, 0.0), DVec3::new(0.0, 3.0, 0.0), DVec3::new(0.0, -3.0, 0.0)] {
		let (closest, _) = edge.project(p).unwrap();
		assert!(closest.distance(p) < 1.0e-9, "ellipse must pass through {p:?}, closest {closest:?}");
	}
	assert!(Edge::ellipse(3.0, 5.0, DVec3::X, DVec3::Z).is_err(), "major < minor must be rejected");
}

#[test]
fn project_box_edges_to_plane() {
	let solid = Solid::cube(DVec3::new(-10.0, -10.0, -5.0), DVec3::new(10.0, 10.0, 5.0));
	let edges = solid.project_to_plane(DVec3::new(0.0, 0.0, -40.0), DVec3::Z).unwrap();
	assert!(!edges.is_empty());
	for edge in &edges {
		for p in [edge.start_point(), edge.end_point()] {
			assert!((p.z + 40.0).abs() < 1.0e-9, "projected edge must lie on the plane, got {p:?}");
		}
	}
}
