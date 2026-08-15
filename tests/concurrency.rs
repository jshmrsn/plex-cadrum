use std::sync::{Arc, Barrier};

use cadrum::{Boolean, DVec3, Edge, Solid, Tessellation};

#[test]
fn independent_native_pipelines_are_stable_under_parallel_stress() {
	let worker_count = std::thread::available_parallelism().map_or(2, usize::from).clamp(2, 4);
	let barrier = Arc::new(Barrier::new(worker_count));
	let workers = (0..worker_count)
		.map(|worker| {
			let barrier = Arc::clone(&barrier);
			std::thread::spawn(move || {
				barrier.wait();
				for iteration in 0..8 {
					let offset = worker as f64 * 30.0 + iteration as f64 * 0.01;
					let left = Solid::cube(DVec3::new(offset, 0.0, 0.0), DVec3::new(offset + 10.0, 10.0, 10.0));
					let right = Solid::cube(DVec3::new(offset + 5.0, 5.0, 5.0), DVec3::new(offset + 15.0, 15.0, 15.0));
					let result = (Boolean::from(&left) + Boolean::from(&right)).build_vec().expect("parallel Boolean");
					assert_eq!(result.len(), 1);
					let mesh = Solid::mesh_chunks(result.iter(), Tessellation { parallel: true, include_edges: false, ..Tessellation::default() }).expect("parallel mesh");
					assert!(!mesh.faces.is_empty());

					let mut archive = Vec::new();
					Solid::write_brep(result.iter(), &mut archive).expect("parallel archive");
					assert!(!archive.is_empty());

					let lower = Edge::polygon(&[DVec3::new(offset, 0.0, 0.0), DVec3::new(offset + 2.0, 0.0, 0.0), DVec3::new(offset + 2.0, 2.0, 0.0), DVec3::new(offset, 2.0, 0.0)]).expect("lower profile");
					let upper = lower.iter().cloned().map(|edge| edge.translate(DVec3::Z * 3.0)).collect::<Vec<_>>();
					let loft = Solid::loft([lower.iter(), upper.iter()], false).expect("serialized loft");
					assert!(loft.volume() > 0.0);
				}
			})
		})
		.collect::<Vec<_>>();

	for worker in workers {
		worker.join().expect("native worker must not panic");
	}
}
