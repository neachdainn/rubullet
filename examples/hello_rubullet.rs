//! An introduction to the usage of RuBullet.
use std::{thread, time::Duration};

use easy_error::Terminator;
use nalgebra::{Isometry3, Vector3};
use rubullet::*;

fn main() -> Result<(), Terminator>
{
	let mut physics_client = PhysicsClient::connect(Mode::Gui)?;

	physics_client.set_additional_search_path("bullet3/libbullet3/data")?;
	physics_client.set_gravity(Vector3::new(0.0, 0.0, -10.0))?;

	let plane_id = physics_client.load_urdf("plane.urdf", Default::default())?;

	let cube_start_position = Isometry3::translation(0.0, 0.0, 1.0);
	let box_id = physics_client.load_urdf("r2d2.urdf", UrdfOptions { base_transform: cube_start_position, ..Default::default() })?;

	for _ in 0..10000 {
		physics_client.step_simulation()?;
		thread::sleep(Duration::from_micros(4167));
	}

	let cube_transform = physics_client.get_base_transform(box_id)?;
	println!("{}", cube_transform);

	Ok(())
}
