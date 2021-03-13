use anyhow::Result;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use rubullet::*;
use std::f64::consts::{FRAC_PI_2, PI};
use std::time::Instant;

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    let plane_id = physics_client.load_urdf(
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
    let kuka_end_effector_index = 6;
    let num_joints = physics_client.get_num_joints(kuka_id);
    assert_eq!(num_joints, 7);
    let cube_id = physics_client.load_urdf(
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
    let jd = [0.1; 7];

    for i in 0..num_joints {
        physics_client.reset_joint_state(kuka_id, i, rp[i], None)?;
    }
    physics_client.set_gravity([0., 0., -10.]);
    physics_client.set_real_time_simulation(true);
    let _log_id_1 = physics_client.start_state_logging(
        LoggingType::GenericRobot,
        "LOG0001.txt",
        StateLoggingOptions {
            object_ids: vec![plane_id, kuka_id, cube_id],
            ..Default::default()
        },
    )?;
    let _log_id_2 = physics_client.start_state_logging(
        LoggingType::ContactPoints,
        "LOG0002.mp3",
        StateLoggingOptions {
            body_a: Some(cube_id),
            ..Default::default()
        },
    )?;
    let start_time = Instant::now();
    let mut has_prev_pose = false;
    let mut prev_pose: Isometry3<f64> = Isometry3::identity();
    let mut prev_pose_1: Isometry3<f64> = Isometry3::identity();
    let trail_duration = 15.;
    loop {
        let dt = (Instant::now() - start_time).as_secs() % 60;
        let t = (dt as f64 / 60.) * 2. * PI;
        let target_pose = Isometry3::from_parts(
            Translation3::new(-0.4, 0.2 * f64::cos(t), 0.2 * f64::sin(t)),
            UnitQuaternion::from_euler_angles(0., -PI, 0.),
        );
        let inverse_kinematics_parameters =
            InverseKinematicsParametersBuilder::new(kuka_end_effector_index, &target_pose)
                .set_joint_damping(&jd)
                .build();
        let joint_poses =
            physics_client.calculate_inverse_kinematics(kuka_id, inverse_kinematics_parameters)?;
        physics_client.set_joint_motor_control_array(
            kuka_id,
            &[0, 1, 2, 3, 4, 5, 6],
            ControlModeArray::PositionsWithPd {
                target_positions: &joint_poses,
                target_velocities: &[0.; 7],
                position_gains: &[0.03; 7],
                velocity_gains: &[1.; 7],
            },
            Some(&[500.; 7]),
        )?;
        let ls = physics_client.get_link_state(kuka_id, kuka_end_effector_index, false, false)?;
        if has_prev_pose {
            physics_client.add_user_debug_line(
                prev_pose.translation.vector,
                target_pose.translation.vector,
                AddDebugLineOptions {
                    line_color_rgb: [0., 0., 0.3],
                    line_width: 1.,
                    life_time: trail_duration,
                    ..Default::default()
                },
            )?;
            physics_client.add_user_debug_line(
                prev_pose_1.translation.vector,
                ls.world_link_frame_pose.translation.vector,
                AddDebugLineOptions {
                    line_color_rgb: [1., 0., 0.],
                    line_width: 1.,
                    life_time: trail_duration,
                    ..Default::default()
                },
            )?;
        }
        prev_pose = target_pose;
        prev_pose_1 = ls.world_link_frame_pose;
        has_prev_pose = true;
    }
}
