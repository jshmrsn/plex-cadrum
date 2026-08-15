use cadrum::{Boolean, CancellationToken, DVec3, Error, Solid};

#[test]
fn cancelled_fillet_is_distinct_from_an_algorithm_failure() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(10.0));
	let edge = cube.iter_edge().next().expect("cube edge");
	let cancellation = CancellationToken::new();
	cancellation.cancel();

	let result = cube.fillet_edges_cancelable(1.0, [edge], &cancellation);
	assert!(matches!(result, Err(Error::Cancelled)));
}

#[test]
fn cancelled_boolean_does_not_poison_later_occt_work() {
	let left = Solid::cube(DVec3::ZERO, DVec3::splat(10.0));
	let right = Solid::cube(DVec3::splat(5.0), DVec3::splat(15.0));
	let expression = Boolean::from(&left) + Boolean::from(&right);
	let cancellation = CancellationToken::new();
	cancellation.cancel();

	assert!(matches!(Solid::boolean_build_cancelable(&expression, &cancellation), Err(Error::Cancelled)));
	let result = (Boolean::from(&left) + Boolean::from(&right)).build_vec().expect("later boolean");
	assert_eq!(result.len(), 1);
}

#[test]
fn completed_builder_reports_progress() {
	let cube = Solid::cube(DVec3::ZERO, DVec3::splat(10.0));
	let edge = cube.iter_edge().next().expect("cube edge");
	let progress = CancellationToken::new();

	cube.fillet_edges_cancelable(1.0, [edge], &progress).expect("fillet");
	assert!(progress.progress() > 0.0);
	assert!(progress.progress() <= 1.0);
}
