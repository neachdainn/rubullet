use anyhow::Result;
use nalgebra::Isometry3;
use rubullet::logging_utils::read_generic_robot_log;
use rubullet::*;
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
    let _kuka_id = physics_client.load_urdf(
        "kuka_iiwa/model.urdf",
        UrdfOptions {
            use_fixed_base: true,
            ..Default::default()
        },
    )?;
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
    let logs = read_generic_robot_log("LOG0001.txt")?;

    let time_step = {
        let last_log = logs.last().unwrap();
        Duration::from_secs_f64(last_log.time_stamp.as_secs_f64() / last_log.step_count as f64)
    };
    for log in logs {
        physics_client.reset_base_transform(log.body, &log.base_pose);
        for i in 0..log.num_joints {
            physics_client.reset_joint_state(
                log.body,
                i,
                log.joint_positions[i],
                log.joint_velocities[i],
            )?;
        }
        std::thread::sleep(time_step);
    }

    Ok(())
}
