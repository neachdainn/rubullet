use easy_error::Terminator;
use nalgebra::{DMatrix, Isometry3, Quaternion, Rotation3, Translation3, UnitQuaternion, Vector3};
use rubullet::client::ControlModeArray::Torques;
use rubullet::client::{ControlModeArray, InverseKinematicsNullSpaceParameters, InverseKinematicsParametersBuilder, JointInfo};
use rubullet::mode::Mode::Direct;
use rubullet::{
     b3JointSensorState, BodyId, ControlMode, DebugVisualizerFlag, JointType,
    PhysicsClient, UrdfOptions,
};
use std::f64::consts::PI;
use std::time::Duration;

fn slice_compare(a: &[f64], b: &[f64], thresh: f64) {
    assert_eq!(a.len(), b.len());
    for i in 0..a.len() {
        println!("{} {}", a[i], b[i]);
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
        .set_additional_search_path("../rubullet-ffi/bullet3/libbullet3/data")
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
    let joint_infos: Vec<JointInfo> = (0..num_joints)
        .into_iter()
        .map(|y| client.get_joint_info(robot, y))
        .collect::<Vec<JointInfo>>();
    let joint_states = joint_states
        .iter()
        .zip(joint_infos.iter())
        .filter(|(_j, i)| i.m_q_index > -1)
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
    p.set_additional_search_path("../rubullet-ffi/bullet3/libbullet3/data")
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
    p.step_simulation().unwrap();

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
    p.set_additional_search_path("../rubullet-ffi/bullet3/libbullet3/data")
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
    physics_client
        .set_additional_search_path("../rubullet-ffi/bullet3/libbullet3/data")
        .unwrap();
    physics_client.set_time_step(&delta_t);
    let id_revolute_joints = [0, 3];
    let id_robot = physics_client
        .load_urdf(
            "TwoJointRobot_w_fixedJoints.urdf",
            UrdfOptions {
                use_fixed_base: true,
                ..Default::default()
            },
        )
        .unwrap();
    physics_client.change_dynamics_angular_damping(id_robot, 0.);
    physics_client.change_dynamics_linear_damping(id_robot, 0.);
    physics_client
        .set_joint_motor_control_array(
            id_robot,
            &id_revolute_joints,
            ControlModeArray::Velocities(&[0., 0.]),
            Some([0., 0.].to_vec()),
        )
        .unwrap();

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

        let torque = physics_client
            .calculate_inverse_dynamics(id_robot, &obj_pos, &obj_vel, &obj_acc)
            .unwrap();

        q_tor[0][i] = torque[0];
        q_tor[1][i] = torque[1];

        physics_client
            .set_joint_motor_control_array(id_robot, &id_revolute_joints, Torques(&torque), None)
            .unwrap();
        physics_client.step_simulation().unwrap();
    }
    slice_compare(q_tor[0].as_slice(), &target_torque[0], 1e-10);
}

#[test]
fn test_mass_matrix_and_inverse_kinematics() -> Result<(), Terminator> {
    let mut physics_client = PhysicsClient::connect(Direct)?;
    physics_client.configure_debug_visualizer(DebugVisualizerFlag::COV_ENABLE_Y_AXIS_UP, true);
    physics_client.set_time_step(&Duration::from_secs_f64(1. / 60.));
    physics_client.set_gravity(Vector3::new(0.0, -9.8, 0.))?;

    let time_step = Duration::from_secs_f64(1. / 60.);
    let mut panda = PandaSim::new(&mut physics_client, Vector3::zeros())?;
    panda.step(&mut physics_client);

    Ok(())
}

pub struct PandaSim {
    pub offset: Vector3<f64>,
    pub id: BodyId,
    pub t: Duration,
}

impl PandaSim {
    const INITIAL_JOINT_POSITIONS: [f64; 9] =
        [0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02];
    const PANDA_NUM_DOFS: usize = 7;
    const PANDA_END_EFFECTOR_INDEX: i32 = 11;
    pub fn new(client: &mut PhysicsClient, offset: Vector3<f64>) -> Result<Self, Terminator> {
        client.set_additional_search_path("../rubullet-ffi/bullet3/libbullet3/data")?;
        client.set_additional_search_path("../rubullet-ffi/bullet3/libbullet3/examples/pybullet/gym/pybullet_data")?;
        let cube_start_position = Isometry3::new(
            Vector3::new(0., 0., 0.),
            UnitQuaternion::from_euler_angles(-PI / 2., 0., 0.).scaled_axis(),
        );

        let urdf_options = UrdfOptions {
            use_fixed_base: true,
            base_transform: cube_start_position.clone(),
            enable_cached_graphics_shapes: true,
            ..Default::default()
        };
        let panda_id = client.load_urdf("franka_panda/panda.urdf", urdf_options)?;
        client.change_dynamics_linear_damping(panda_id, 0.);
        client.change_dynamics_angular_damping(panda_id, 0.);
        let mut index = 0;
        for i in 0..client.get_num_joints(panda_id) {
            let info = client.get_joint_info(panda_id, i);
            if info.m_joint_type == JointType::Revolute
                || info.m_joint_type == JointType::Prismatic
            {
                client.reset_joint_state(
                    panda_id,
                    i,
                    Some(PandaSim::INITIAL_JOINT_POSITIONS[index]),
                )?;
                index += 1;
            }
        }
        let t = Duration::new(0, 0);
        Ok(PandaSim {
            offset,
            id: panda_id,
            t,
        })
    }
    pub fn step(&mut self, client: &mut PhysicsClient) {
        let t = self.t.as_secs_f64();
        self.t += Duration::from_secs_f64(1. / 60.);

        let pose = Isometry3::from_parts(
            Translation3::new(
                0.2 * f64::sin(1.5 * t),
                0.044,
                -0.6 + 0.1 * f64::cos(1.5 * t),
            ),
            UnitQuaternion::<f64>::from_euler_angles(PI / 2., 0., 0.),
        );
        let inverse_kinematics_parameters = InverseKinematicsParametersBuilder::new(
            self.id,
            PandaSim::PANDA_END_EFFECTOR_INDEX,
            &pose,
        )
            .set_max_num_iterations(5)
            .build();
        let joint_poses = client
            .calculate_inverse_kinematics(inverse_kinematics_parameters)
            .unwrap();

        for i in 0..PandaSim::PANDA_NUM_DOFS {
            client.set_joint_motor_control_2(
                self.id,
                i as i32,
                ControlMode::Position(joint_poses[i]),
                Some(240. * 5.),
            );
        }
        let target_mass_matrix = [
            1.2851012047449573,
            0.019937918309349063,
            1.099094168104455,
            -0.13992071941819356,
            -0.04530258995812824,
            0.015010766618204326,
            -0.00419273685297444,
            -0.004045701114304651,
            0.004045701114304651,
            0.019937918309349063,
            1.4200408634854977,
            -0.13973846625926817,
            -0.6766534143669977,
            -0.011150292631844208,
            -0.11222097289908575,
            7.963273507305265e-05,
            0.021288781819096075,
            -0.021288781819096075,
            1.099094168104455,
            -0.13973846625926817,
            1.0599485744387636,
            -0.01927738076755258,
            -0.03898176486608705,
            0.0367718209930567,
            -0.0038394168220030464,
            -0.008321730809750807,
            0.008321730809750807,
            -0.13992071941819356,
            -0.6766534143669977,
            -0.01927738076755258,
            0.8883254282752625,
            0.03919655691643165,
            0.13355872722768167,
            0.0005209277856703768,
            -0.04891378328717881,
            0.04891378328717881,
            -0.04530258995812824,
            -0.011150292631844208,
            -0.03898176486608705,
            0.03919655691643165,
            0.028341271915447625,
            -0.0001434455846943853,
            0.0037745850335795,
            1.8885374235014403e-05,
            -1.8885374235014403e-05,
            0.015010766618204326,
            -0.11222097289908575,
            0.0367718209930567,
            0.13355872722768167,
            -0.0001434455846943853,
            0.047402314149303876,
            2.053797643165216e-14,
            -0.018417574890008004,
            0.018417574890008004,
            -0.00419273685297444,
            7.963273507305265e-05,
            -0.0038394168220030464,
            0.0005209277856703768,
            0.0037745850335795,
            2.053797643165216e-14,
            0.004194301009161442,
            0.0,
            0.0,
            -0.004045701114304651,
            0.021288781819096075,
            -0.008321730809750807,
            -0.04891378328717881,
            1.8885374235014403e-05,
            -0.018417574890008004,
            0.0,
            0.1,
            0.0,
            0.004045701114304651,
            -0.021288781819096075,
            0.008321730809750807,
            0.04891378328717881,
            -1.8885374235014403e-05,
            0.018417574890008004,
            0.0,
            0.0,
            0.1,
        ];
        let target_joint_poses = [
            1.1029851000632531,
            0.43354557662855453,
            0.3608104666320187,
            -2.3105861116521096,
            -0.2888395010735958,
            2.6904095021250938,
            2.4711777602235387,
            0.02,
            0.02,
        ];
        let mass = client
            .calculate_mass_matrix(self.id, joint_poses.as_slice())
            .unwrap();
        slice_compare(joint_poses.as_slice(), &target_joint_poses, 1e-6);
        slice_compare(mass.as_slice(), &target_mass_matrix, 1e-6);
    }
}
