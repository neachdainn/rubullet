use anyhow::Result;
use nalgebra::{Isometry3, Vector3};
use rubullet::*;

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    physics_client.load_urdf(
        "plane.urdf",
        UrdfOptions {
            use_maximal_coordinates: Some(false),
            ..Default::default()
        },
    )?;
    physics_client.load_urdf(
        "cube.urdf",
        UrdfOptions {
            base_transform: Isometry3::translation(0., 0., 1.),
            use_maximal_coordinates: Some(false),
            ..Default::default()
        },
    )?;
    physics_client.set_gravity([0., 3., -10.])?;
    loop {
        physics_client.step_simulation()?;
        let pts = physics_client.get_contact_points(None, None, None, None)?;
        let mut total_normal_force = 0.;
        let mut total_lateral_friction_force = Vector3::zeros();
        for pt in pts.iter() {
            total_normal_force += pt.normal_force.unwrap();
            total_lateral_friction_force += pt.lateral_friction_1 + pt.lateral_friction_2;
        }
        println!("total_normal_force = {}", total_normal_force);
        println!(
            "total_lateral_friction_force = {}",
            total_lateral_friction_force
        );
    }
}
