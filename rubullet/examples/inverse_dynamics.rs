use std::f64::consts::PI;
use std::time::Duration;

use anyhow::Result;

use rubullet::ControlModeArray::Torques;
use rubullet::*;

fn main() -> Result<()> {
    let delta_t = Duration::from_secs_f64(0.0001);
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    physics_client.set_time_step(delta_t);
    let id_revolute_joints = [0, 3];
    let id_robot = physics_client.load_urdf(
        "TwoJointRobot_w_fixedJoints.urdf",
        UrdfOptions {
            use_fixed_base: true,
            ..Default::default()
        },
    )?;
    physics_client.change_dynamics(
        id_robot,
        None,
        ChangeDynamicsOptions {
            linear_damping: Some(0.),
            angular_damping: Some(0.),
            ..Default::default()
        },
    );
    physics_client.set_joint_motor_control_array(
        id_robot,
        &id_revolute_joints,
        ControlModeArray::Velocities(&[0., 0.]),
        Some(&[0., 0.]),
    )?;
    // Target Positions:
    let start = 0.;
    let end = 1.;

    let steps = ((end - start) / delta_t.as_secs_f64()) as usize;
    let mut t = vec![0.; steps];

    let mut q_pos_desired = vec![vec![0.; steps]; 2];
    let mut q_vel_desired = vec![vec![0.; steps]; 2];
    let mut q_acc_desired = vec![vec![0.; steps]; 2];

    for s in 0..steps {
        t[s] = start + s as f64 * delta_t.as_secs_f64();
        q_pos_desired[0][s] = 1. / (2. * PI) * f64::sin(2. * PI * t[s]) - t[s];
        q_pos_desired[1][s] = -1. / (2. * PI) * f64::sin(2. * PI * t[s]) - 1.;

        q_vel_desired[0][s] = f64::cos(2. * PI * t[s]) - 1.;
        q_vel_desired[1][s] = f64::sin(2. * PI * t[s]);

        q_acc_desired[0][s] = -2. * PI * f64::sin(2. * PI * t[s]);
        q_acc_desired[1][s] = 2. * PI * f64::cos(2. * PI * t[s]);
    }
    let mut q_pos = vec![vec![0.; steps]; 2];
    let mut q_vel = vec![vec![0.; steps]; 2];
    let mut q_tor = vec![vec![0.; steps]; 2];

    for i in 0..t.len() {
        let joint_states = physics_client.get_joint_states(id_robot, &[0, 3])?;
        q_pos[0][1] = joint_states[0].joint_position;
        let a = joint_states[1].joint_position;
        q_pos[1][i] = a;

        q_vel[0][i] = joint_states[0].joint_velocity;
        q_vel[1][i] = joint_states[1].joint_velocity;

        let obj_pos = [q_pos[0][i], q_pos[1][i]];
        let obj_vel = [q_vel[0][i], q_vel[1][i]];
        let obj_acc = [q_acc_desired[0][i], q_acc_desired[1][i]];

        let torque =
            physics_client.calculate_inverse_dynamics(id_robot, &obj_pos, &obj_vel, &obj_acc)?;

        q_tor[0][i] = torque[0];
        q_tor[1][i] = torque[1];

        physics_client.set_joint_motor_control_array(
            id_robot,
            &id_revolute_joints,
            Torques(&torque),
            None,
        )?;
        physics_client.step_simulation()?;
        std::thread::sleep(delta_t);
    }

    Ok(())
}
