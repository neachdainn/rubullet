use anyhow::Result;
use nalgebra::Isometry3;
use rubullet::*;
use std::f64::consts::FRAC_PI_2;
use std::time::Duration;

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    let _plane_id = physics_client.load_urdf(
        "plane.urdf",
        UrdfOptions {
            base_transform: Isometry3::translation(0., 0., -0.3),
            use_fixed_base: true,
            ..Default::default()
        },
    )?;
    let kuka_id = physics_client.load_urdf(
        "kuka_iiwa/model.urdf",
        UrdfOptions {
            use_fixed_base: true,
            ..Default::default()
        },
    )?;

    let num_joints = physics_client.get_num_joints(kuka_id);
    assert_eq!(num_joints, 7);
    let _cube_id = physics_client.load_urdf(
        "cube.urdf",
        UrdfOptions {
            base_transform: Isometry3::translation(2., 2., 5.),
            ..Default::default()
        },
    )?;
    physics_client.load_urdf(
        "cube.urdf",
        UrdfOptions {
            base_transform: Isometry3::translation(-2., -2., 5.),
            ..Default::default()
        },
    )?;
    physics_client.load_urdf(
        "cube.urdf",
        UrdfOptions {
            base_transform: Isometry3::translation(2., -2., 5.),
            ..Default::default()
        },
    )?;
    let rp = [0., 0., 0., FRAC_PI_2, 0., -FRAC_PI_2 * 0.66, 0.];

    for i in 0..num_joints {
        physics_client.reset_joint_state(kuka_id, i, rp[i], None)?;
    }
    physics_client.set_gravity([0., 0., -10.]);

    std::thread::sleep(Duration::from_secs(2));
    physics_client.save_world("kuka_world.py")?;
    physics_client.save_bullet("state1.bullet")?;
    let state_1 = physics_client.save_state()?;

    physics_client.reset_base_transform(kuka_id, Isometry3::translation(2., 0., 0.));
    physics_client.save_bullet("state2.bullet")?;
    std::thread::sleep(Duration::from_secs(2));

    physics_client.restore_state(state_1)?;
    std::thread::sleep(Duration::from_secs(2));

    physics_client.reset_simulation();
    std::thread::sleep(Duration::from_secs(2));

    let _bodies = physics_client.load_bullet("state1.bullet")?;
    std::thread::sleep(Duration::from_secs(2));

    physics_client.restore_state_from_file("state2.bullet")?;
    std::thread::sleep(Duration::from_secs(2));

    Ok(())
}
