use nalgebra::{DMatrix, Isometry3, Vector3};
use rubullet::client::ControlModeArray;
use rubullet::mode::Mode::Direct;
use rubullet::{b3JointInfo, b3JointSensorState, BodyId, PhysicsClient, UrdfOptions};
use std::time::Duration;
use rubullet::client::ControlModeArray::Torques;
use std::f64::consts::PI;

fn slice_compare(a: &[f64], b: &[f64], thresh: f64) {
    assert_eq!(a.len(), b.len());
    for i in 0..a.len() {
        float_compare(a[i], b[i], thresh);
    }
}

fn float_compare(a: f64, b: f64, thresh: f64) {
    assert!((a - b).abs() < thresh);
}

#[test]
fn test_connect() {
    let mut physics_client = PhysicsClient::connect(Direct).unwrap();
}

#[test]
fn test_load_urdf() {
    let mut physics_client = PhysicsClient::connect(Direct).unwrap();
    physics_client
        .set_additional_search_path("bullet3/libbullet3/data")
        .unwrap();
    let plane_id = physics_client
        .load_urdf("plane.urdf", Default::default())
        .unwrap();
}

pub fn set_joint_positions(client: &mut PhysicsClient, robot: BodyId, position: &[f64]) {
    let num_joints = client.get_num_joints(robot);
    assert_eq!(num_joints as usize, position.len());
    let indices = (0..num_joints).into_iter().collect::<Vec<i32>>();
    let zero_vec = vec![0.; num_joints as usize];
    let position_gains = vec![1.; num_joints as usize];
    let velocity_gains = vec![0.3; num_joints as usize];
    client.set_joint_motor_control_array(
        robot,
        indices.as_slice(),
        ControlModeArray::PositionsWithPD {
            target_positions: position,
            target_velocities: zero_vec.as_slice(),
            position_gains: position_gains.as_slice(),
            velocity_gains: velocity_gains.as_slice(),
        },
        None,
    );
}

pub fn get_joint_states(
    client: &mut PhysicsClient,
    robot: BodyId,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let num_joints = client.get_num_joints(robot);
    let indices = (0..num_joints).into_iter().collect::<Vec<i32>>();
    let joint_states = client.get_joint_states(robot, indices.as_slice()).unwrap();
    let pos = joint_states
        .iter()
        .map(|x| x.m_joint_position)
        .collect::<Vec<f64>>();
    let vel = joint_states
        .iter()
        .map(|x| x.m_joint_velocity)
        .collect::<Vec<f64>>();
    let torque = joint_states
        .iter()
        .map(|x| x.m_joint_motor_torque)
        .collect::<Vec<f64>>();
    (pos, vel, torque)
}

pub fn get_motor_joint_states(
    client: &mut PhysicsClient,
    robot: BodyId,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let num_joints = client.get_num_joints(robot);
    let indices = (0..num_joints).into_iter().collect::<Vec<i32>>();
    let joint_states = client.get_joint_states(robot, indices.as_slice()).unwrap();
    let joint_infos: Vec<b3JointInfo> = (0..num_joints)
        .into_iter()
        .map(|y| client.get_joint_info(robot, y))
        .collect::<Vec<b3JointInfo>>();
    let joint_states = joint_states
        .iter()
        .zip(joint_infos.iter())
        .filter(|(j, i)| i.m_q_index > -1)
        .map(|(j, i)| *j)
        .collect::<Vec<b3JointSensorState>>();
    let pos = joint_states
        .iter()
        .map(|x| x.m_joint_position)
        .collect::<Vec<f64>>();
    let vel = joint_states
        .iter()
        .map(|x| x.m_joint_velocity)
        .collect::<Vec<f64>>();
    let torque = joint_states
        .iter()
        .map(|x| x.m_joint_motor_torque)
        .collect::<Vec<f64>>();
    (pos, vel, torque)
}

#[test]
fn test_jacobian() {
    let delta_t = Duration::from_secs_f64(0.001);
    let mut p = PhysicsClient::connect(Direct).unwrap();
    p.set_additional_search_path("bullet3/libbullet3/data")
        .unwrap();
    p.set_time_step(&delta_t);

    let kuka_id = p
        .load_urdf(
            "TwoJointRobot_w_fixedJoints.urdf",
            UrdfOptions {
                use_fixed_base: true,
                ..Default::default()
            },
        )
        .unwrap();

    let num_joints = p.get_num_joints(kuka_id);
    let kuka_end_effector_index = num_joints - 1;
    set_joint_positions(&mut p, kuka_id, vec![0.1; num_joints as usize].as_slice());
    p.step_simulation();

    let (pos, vel, torq) = get_joint_states(&mut p, kuka_id);
    let (mpos, mvel, mtorq) = get_motor_joint_states(&mut p, kuka_id);
    let result = p
        .get_link_state(kuka_id, kuka_end_effector_index, true, true)
        .unwrap();
    println!("{:?}", vel);
    let zero_vec = vec![0.; mpos.len()];
    let jacobian = p
        .calculate_jacobian(
            kuka_id,
            kuka_end_effector_index,
            &result.m_local_inertial_position,
            mpos.as_slice(),
            zero_vec.as_slice(),
            zero_vec.as_slice(),
        )
        .unwrap();
    let target_linear_jacobian = [
        -0.1618321740829912,
        1.9909341219607504,
        0.0,
        -0.13073095940453022,
        0.991417881749755,
        0.0,
    ];
    for (i, j) in jacobian
        .linear_jacobian
        .as_slice()
        .iter()
        .zip(target_linear_jacobian.iter())
    {
        assert!((i - j).abs() < 1e-6);
    }
    let target_angluar_jacobian = [0., 0., 1.0, 0.0, 0., 1.];
    for (i, j) in jacobian
        .angular_jacobian
        .as_slice()
        .iter()
        .zip(target_angluar_jacobian.iter())
    {
        assert!((i - j).abs() < 1e-6);
    }
}

#[test]
fn test_get_link_state() {
    let delta_t = Duration::from_secs_f64(0.001);
    let mut p = PhysicsClient::connect(Direct).unwrap();
    p.set_additional_search_path("bullet3/libbullet3/data")
        .unwrap();
    let gravity_constant = -9.81;
    p.set_time_step(&delta_t);
    // p.set_gravity(Vector3::new(0., 0., gravity_constant)).unwrap();

    let kuka_id = p
        .load_urdf(
            "TwoJointRobot_w_fixedJoints.urdf",
            UrdfOptions {
                use_fixed_base: true,
                ..Default::default()
            },
        )
        .unwrap();

    let num_joints = p.get_num_joints(kuka_id);
    let kuka_end_effector_index = num_joints - 1;
    set_joint_positions(&mut p, kuka_id, vec![0.1; num_joints as usize].as_slice());
    p.step_simulation();

    let result = p
        .get_link_state(kuka_id, kuka_end_effector_index, true, true)
        .unwrap();
    let m_world_position = [1.9909341219607506, 0.1618321740829912, 0.12500000000000003];
    let m_world_orientation = [-0.0, -0.0, 0.06550617623646283, 0.997852163837348];
    let m_local_inertial_position = [0.; 3];
    let m_local_inertial_orientation = [0., 0., 0., 1.];
    let m_world_link_frame_position = [1.990934133529663, 0.16183216869831085, 0.125];
    let m_world_link_frame_orientation = [0.0, 0.0, 0.06550618261098862, 0.9978521466255188];
    let m_world_linear_velocity = [-18.107084901818524, 161.0722445230232, 0.0];
    let m_world_angular_velocity = [0.0, 0.0, 131.10623082146793];
    slice_compare(&result.m_world_position, &m_world_position, 1e-6);
    slice_compare(&result.m_world_orientation, &m_world_orientation, 1e-6);
    slice_compare(
        &result.m_local_inertial_position,
        &m_local_inertial_position,
        1e-6,
    );
    slice_compare(
        &result.m_local_inertial_orientation,
        &m_local_inertial_orientation,
        1e-6,
    );
    slice_compare(
        &result.m_world_link_frame_position,
        &m_world_link_frame_position,
        1e-6,
    );
    slice_compare(
        &result.m_world_link_frame_orientation,
        &m_world_link_frame_orientation,
        1e-6,
    );
    slice_compare(
        &result.m_world_linear_velocity,
        &m_world_linear_velocity,
        1e-5,
    );
    slice_compare(
        &result.m_world_angular_velocity,
        &m_world_angular_velocity,
        1e-6,
    );
}

#[test]
pub fn inverse_dynamics_test() {
    let target_torque = [
        [
            2.789039373397827,
            -4.350228309631348,
            -9.806090354919434,
            -11.364048957824707,
            -8.452873229980469,
            -2.4533324241638184,
            4.292827606201172,
            9.478754043579102,
            11.22978687286377,
            8.697705268859863,
        ],
        [
            1.5324022769927979,
            -0.3981592059135437,
            -2.1556396484375,
            -3.0122971534729004,
            -2.61287522315979,
            -1.1830209493637085,
            0.663445770740509,
            2.273959159851074,
            3.1140832901000977,
            2.8513741493225098,
        ],
    ];
    let delta_t = Duration::from_secs_f64(0.1);
    let mut physics_client = PhysicsClient::connect(Direct).unwrap();
    physics_client.set_additional_search_path("bullet3/libbullet3/data").unwrap();
    physics_client.set_time_step(&delta_t);
    let id_revolute_joints = [0, 3];
    let id_robot = physics_client.load_urdf(
        "TwoJointRobot_w_fixedJoints.urdf",
        UrdfOptions {
            use_fixed_base: true,
            ..Default::default()
        },
    ).unwrap();
    physics_client.change_dynamics_angular_damping(id_robot, 0.);
    physics_client.change_dynamics_linear_damping(id_robot, 0.);
    physics_client.set_joint_motor_control_array(
        id_robot,
        &id_revolute_joints,
        ControlModeArray::Velocities(&[0., 0.]),
        Some([0., 0.].to_vec()),
    ).unwrap();

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
        let joint_states = physics_client.get_joint_states(id_robot, &[0, 3]).unwrap();
        q_pos[0][1] = joint_states[0].m_joint_position;
        let a = joint_states[1].m_joint_position;
        q_pos[1][i] = a;

        q_vel[0][i] = joint_states[0].m_joint_velocity;
        q_vel[1][i] = joint_states[1].m_joint_velocity;

        let obj_pos = [q_pos[0][i], q_pos[1][i]];
        let obj_vel = [q_vel[0][i], q_vel[1][i]];
        let obj_acc = [q_acc_desired[0][i], q_acc_desired[1][i]];

        let torque =
            physics_client.calculate_inverse_dynamics(id_robot, &obj_pos, &obj_vel, &obj_acc).unwrap();

        q_tor[0][i] = torque[0];
        q_tor[1][i] = torque[1];

        physics_client.set_joint_motor_control_array(
            id_robot,
            &id_revolute_joints,
            Torques(&torque),
            None,
        ).unwrap();
        physics_client.step_simulation().unwrap();
    }
    slice_compare(q_tor[0].as_slice(), &target_torque[0], 1e-10);
}

