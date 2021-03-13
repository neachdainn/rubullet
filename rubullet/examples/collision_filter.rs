use anyhow::Result;
use nalgebra::Isometry3;
use rubullet::*;
use std::time::Duration;

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    let plane_id = physics_client.load_urdf(
        "plane.urdf",
        UrdfOptions {
            use_maximal_coordinates: Some(false),
            ..Default::default()
        },
    )?;
    let cube_id = physics_client.load_urdf(
        "cube_collisionfilter.urdf",
        UrdfOptions {
            use_maximal_coordinates: Some(false),
            base_transform: Isometry3::translation(0., 0., 3.),
            ..Default::default()
        },
    )?;

    let collision_filter_group = 0;
    let collision_filter_mask = 0;

    physics_client.set_collision_filter_group_mask(
        cube_id,
        None,
        collision_filter_group,
        collision_filter_mask,
    );

    let enable_collision = true;

    physics_client.set_collision_filter_pair(plane_id, cube_id, None, None, enable_collision);
    physics_client.set_real_time_simulation(true);
    physics_client.set_gravity([0., 0., -10.]);
    loop {
        std::thread::sleep(Duration::from_secs_f64(1. / 240.));
        physics_client.set_gravity([0., 0., -10.]);
    }
}
