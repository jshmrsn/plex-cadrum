use cadrum::{CancellationToken, DVec3, Edge, EdgeBlendKind, Error, ExtrusionSession, Solid};

#[test]
fn prepared_edge_blend_reuses_source_for_parameter_updates() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(10.0));
	let session = cube.prepare_edge_blend(&[0]).expect("prepare edge blend");
	assert_eq!(session.source_id(), cube.id());

	let small = session.update(EdgeBlendKind::Fillet, 0.5, &CancellationToken::new()).expect("small fillet");
	let large = session.update(EdgeBlendKind::Fillet, 2.0, &CancellationToken::new()).expect("large fillet");
	assert!((small.volume() - large.volume()).abs() > 1.0e-3);
	assert!(!small.topology_history().relations().is_empty());
	assert!(!large.topology_history().relations().is_empty());
}

#[test]
fn prepared_edge_blend_can_switch_between_fillet_and_chamfer() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(10.0));
	let session = cube.prepare_edge_blend(&[0]).expect("prepare edge blend");
	let fillet = session.update(EdgeBlendKind::Fillet, 1.0, &CancellationToken::new()).expect("fillet");
	let chamfer = session.update(EdgeBlendKind::Chamfer, 1.0, &CancellationToken::new()).expect("chamfer");

	assert!((fillet.volume() - chamfer.volume()).abs() > 1.0e-3);
}

#[test]
fn prepared_extrusion_reuses_profile_and_honors_cancellation() {
	let profile = Edge::polygon(&[DVec3::ZERO, DVec3::X * 10.0, DVec3::new(10.0, 10.0, 0.0), DVec3::Y * 10.0]).expect("profile");
	let session = ExtrusionSession::prepare(&profile).expect("prepare extrusion");
	let short = session.update(DVec3::Z * 2.0, &CancellationToken::new()).expect("short extrusion");
	let tall = session.update(DVec3::Z * 8.0, &CancellationToken::new()).expect("tall extrusion");
	assert!((short.volume() - 200.0).abs() < 1.0e-6);
	assert!((tall.volume() - 800.0).abs() < 1.0e-6);

	let cancelled = CancellationToken::new();
	cancelled.cancel();
	assert!(matches!(session.update(DVec3::Z * 4.0, &cancelled), Err(Error::Cancelled)));
}

#[test]
fn prepared_sessions_reject_empty_or_invalid_topology() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(10.0));
	assert!(matches!(cube.prepare_edge_blend(&[]), Err(Error::InvalidEdge(_))));
	assert!(matches!(cube.prepare_edge_blend(&[u32::MAX]), Err(Error::InvalidEdge(_))));
	assert!(matches!(ExtrusionSession::prepare(std::iter::empty()), Err(Error::InvalidEdge(_))));
}
