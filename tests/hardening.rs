use cadrum::{BSplineEnd, CancellationToken, DVec3, Edge, Error, FailureCategory, Solid};

fn invalid_input(error: Error) {
	assert_eq!(error.category(), FailureCategory::InvalidInput, "{error:?}");
	assert_eq!(error.stage(), "validate_input");
}

fn must_fail<T>(result: Result<T, Error>, context: &str) -> Error {
	match result {
		Ok(_) => panic!("{context} unexpectedly succeeded"),
		Err(error) => error,
	}
}

#[test]
fn fallible_primitives_reject_nonfinite_and_degenerate_inputs() {
	for error in [must_fail(Solid::try_cube(DVec3::ZERO, DVec3::new(1.0, 1.0, f64::NAN)), "nonfinite box"), must_fail(Solid::try_cube(DVec3::ZERO, DVec3::new(1.0, 0.0, 1.0)), "flat box"), must_fail(Solid::try_sphere(f64::INFINITY), "nonfinite sphere"), must_fail(Solid::try_cylinder(1.0, DVec3::ZERO), "zero cylinder"), must_fail(Solid::try_cone(0.0, 0.0, DVec3::Z), "zero cone"), must_fail(Solid::try_torus(1.0, 2.0, DVec3::Z), "self-intersecting torus"), must_fail(Solid::try_half_space(DVec3::ZERO, DVec3::ZERO), "zero normal")] {
		invalid_input(error);
	}
}

#[test]
fn edge_builders_reject_adversarial_floating_point_inputs() {
	for result in [Edge::line(DVec3::ZERO, DVec3::new(f64::NAN, 0.0, 0.0)).map(|_| ()), Edge::circle(-1.0, DVec3::Z).map(|_| ()), Edge::arc_3pts(DVec3::ZERO, DVec3::X, DVec3::new(f64::INFINITY, 0.0, 0.0)).map(|_| ()), Edge::polygon(&[DVec3::ZERO, DVec3::X, DVec3::new(0.0, f64::NAN, 0.0)]).map(|_| ()), Edge::helix(1.0, 1.0, 5.0, DVec3::Z, DVec3::Z).map(|_| ()), Edge::bspline(&[DVec3::ZERO, DVec3::X], BSplineEnd::Clamped { start: DVec3::ZERO, end: DVec3::X }).map(|_| ())] {
		invalid_input(result.expect_err("malformed edge input must fail"));
	}
}

#[test]
fn iterative_operations_reject_nonfinite_parameters_before_native_work() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(10.0));
	let edge = cube.iter_edge().next().expect("cube edge");
	let face = cube.iter_face().next().expect("cube face");
	let profile = Edge::polygon(&[DVec3::ZERO, DVec3::X, DVec3::new(1.0, 1.0, 0.0), DVec3::Y]).expect("profile");
	let progress = CancellationToken::new();

	for error in [must_fail(cube.fillet_edges_cancelable(f64::NAN, [edge], &progress), "fillet NaN"), must_fail(cube.chamfer_edges_cancelable(f64::INFINITY, [edge], &progress), "chamfer infinity"), must_fail(Solid::extrude_cancelable(&profile, DVec3::new(0.0, 0.0, f64::NAN), &progress), "extrude NaN"), must_fail(cube.shell_cancelable(f64::NAN, [face], &progress), "shell NaN")] {
		invalid_input(error);
	}
}

#[test]
fn malformed_brep_corpus_never_panics_or_produces_solids() {
	let corpus = [Vec::new(), b"DBRep_DrawableShape\n".to_vec(), vec![0xff; 4_096], (0_u32..4_096).map(|value| value.wrapping_mul(73) as u8).collect()];
	for bytes in corpus {
		let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| Solid::read_brep(&mut bytes.as_slice())));
		assert!(result.is_ok(), "malformed B-rep crossed the bridge as a panic");
		assert!(result.expect("checked above").is_err());
	}
}

#[test]
fn prepared_shell_honors_cancellation_without_poisoning_its_source() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(10.0));
	let session = cube.prepare_shell(&[0]).expect("prepare shell");
	let cancellation = CancellationToken::new();
	cancellation.cancel();
	assert!(matches!(session.update(-1.0, &cancellation), Err(Error::Cancelled)));
	let completed = session.update(-1.0, &CancellationToken::new()).expect("later shell");
	assert!(completed.validate().expect("validate shell").valid);
}

#[test]
fn loft_honors_cancellation_and_a_later_build_still_succeeds() {
	let lower = Edge::polygon(&[DVec3::ZERO, DVec3::X * 2.0, DVec3::new(2.0, 2.0, 0.0), DVec3::Y * 2.0]).expect("lower section");
	let upper = lower.iter().map(|edge| edge.shared_copy().translate(DVec3::Z * 5.0)).collect::<Vec<_>>();
	let cancellation = CancellationToken::new();
	cancellation.cancel();
	assert!(matches!(Solid::loft_cancelable([lower.iter(), upper.iter()], false, &cancellation), Err(Error::Cancelled)));
	let completed = Solid::loft_cancelable([lower.iter(), upper.iter()], false, &CancellationToken::new()).expect("later loft");
	assert!(completed.validate().expect("validate loft").valid);
}
