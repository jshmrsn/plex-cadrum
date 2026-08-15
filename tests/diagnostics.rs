use cadrum::{DVec3, Error, FailureCategory, ResultTopology, Solid, Tessellation, TopologyKind};

#[test]
fn caught_occt_exceptions_keep_operation_stage_and_native_message() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(10.0));
	let error = Solid::mesh_chunks([&cube], Tessellation { deflection_linear: 0.0, ..Tessellation::default() }).expect_err("zero deflection must fail");

	let Error::OperationFailed(failure) = &error else {
		panic!("expected a structured native exception, got {error:?}");
	};
	assert_eq!(failure.operation, "mesh_shape");
	assert_eq!(failure.stage, "native");
	assert_eq!(failure.category, FailureCategory::AlgorithmFailed);
	assert!(failure.exception_type.as_deref().is_some_and(|kind| !kind.is_empty()));
	assert!(!failure.message.is_empty());
	assert_eq!(error.category(), FailureCategory::AlgorithmFailed);
	assert!(error.may_keep_last_valid_result());
}

#[test]
fn ordinary_failures_also_have_stable_recovery_categories() {
	assert_eq!(Error::Cancelled.category(), FailureCategory::Cancelled);
	assert_eq!(Error::ProjectionFailed("face").category(), FailureCategory::NoSolution);
	assert_eq!(Error::TopologyQueryFailed.category(), FailureCategory::InvalidResult);
	assert_eq!(Error::BrepReadFailed.category(), FailureCategory::Io);
}

#[test]
fn invalid_topology_distance_keeps_input_failure_context() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(10.0));
	let error = cube.topology_distance(ResultTopology { kind: TopologyKind::Face, index: u32::MAX }, &cube, ResultTopology { kind: TopologyKind::Face, index: 0 }).expect_err("an invalid face ordinal must fail");
	let Error::OperationFailed(failure) = error else { panic!("expected structured input failure") };
	assert_eq!(failure.operation, "topology_distance");
	assert_eq!(failure.stage, "resolve_inputs");
	assert_eq!(failure.category, FailureCategory::InvalidInput);
}
