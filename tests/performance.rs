use std::{hint::black_box, time::Instant};

use cadrum::{DVec3, ResultTopology, Solid, Tessellation, TopologyKind, TopologyQueryOptions};

const SAMPLES: usize = 15;

fn benchmark<T>(name: &str, mut operation: impl FnMut() -> T) {
	let mut samples = Vec::with_capacity(SAMPLES);
	for _ in 0..SAMPLES {
		let started = Instant::now();
		black_box(operation());
		samples.push(started.elapsed().as_secs_f64() * 1_000.0);
	}
	samples.sort_by(f64::total_cmp);
	let median = samples[samples.len() / 2];
	let p95 = samples[((samples.len() as f64 * 0.95).ceil() as usize).saturating_sub(1)];
	let maximum = *samples.last().expect("at least one benchmark sample");
	eprintln!("cadrum_benchmark stage={name} samples={SAMPLES} median_ms={median:.3} p95_ms={p95:.3} max_ms={maximum:.3}");
}

/// Manual stage baseline. Run with:
/// `cargo test --release --test performance -- --ignored --nocapture`
#[test]
#[ignore = "manual native performance baseline"]
fn exact_operation_stage_baseline() {
	benchmark("box_build", || Solid::cube(DVec3::ZERO, DVec3::splat(40.0)));

	let source = Solid::cube(DVec3::ZERO, DVec3::splat(40.0));
	let blend = source.prepare_edge_blend(&[0, 1, 2, 3]).expect("prepare blend");
	let cancellation = cadrum::CancellationToken::new();
	benchmark("prepared_fillet_build_and_history", || blend.update(cadrum::EdgeBlendKind::Fillet, 2.0, &cancellation).expect("fillet"));
	benchmark("topology_adjacency", || source.topology_snapshot().expect("topology"));
	benchmark("topology_measurement_facts", || source.topology_snapshot_with_options(TopologyQueryOptions::MEASUREMENT).expect("topology facts"));
	benchmark("topology_exact_distance", || source.topology_distance(ResultTopology { kind: TopologyKind::Face, index: 0 }, &source, ResultTopology { kind: TopologyKind::Face, index: 1 }).expect("exact distance"));
	benchmark("brepcheck_validation", || source.validate().expect("validation"));
	benchmark("mass_properties", || (source.volume(), source.area(), source.center(), source.inertia()));

	let mut archive = Vec::new();
	Solid::write_brep([&source], &mut archive).expect("seed archive");
	benchmark("brep_archive", || {
		let mut output = Vec::new();
		Solid::write_brep([&source], &mut output).expect("archive");
		output
	});
	benchmark("brep_rehydrate", || {
		let mut input = archive.as_slice();
		Solid::read_brep(&mut input).expect("rehydrate")
	});

	let surface_options = Tessellation { include_edges: false, parallel: false, ..Tessellation::default() };
	benchmark("surface_mesh_serial", || Solid::mesh_chunks([&source], surface_options).expect("surface mesh"));
	let parallel_options = Tessellation { parallel: true, ..surface_options };
	benchmark("surface_mesh_parallel", || Solid::mesh_chunks([&source], parallel_options).expect("parallel surface mesh"));
	benchmark("ordered_edge_polylines", || source.edge_polyline_chunks(surface_options).expect("edge polylines"));

	let many_solids = (0..25).map(|index| Solid::cube(DVec3::ZERO, DVec3::splat(10.0)).translate(DVec3::X * f64::from(index) * 15.0)).collect::<Vec<_>>();
	benchmark("150_face_surface_mesh_serial", || Solid::mesh_chunks(many_solids.iter(), surface_options).expect("large serial mesh"));
	benchmark("150_face_surface_mesh_parallel", || Solid::mesh_chunks(many_solids.iter(), parallel_options).expect("large parallel mesh"));
}
