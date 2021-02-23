use std::f64::consts::PI;
use std::time::Duration;

use anyhow::Result;
use nalgebra::{Isometry3, Quaternion, Rotation3, Translation3, UnitQuaternion, Vector3};

use rubullet::*;

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    physics_client.set_additional_search_path(
        "../rubullet-sys/bullet3/libbullet3/examples/pybullet/gym/pybullet_data",
    )?;
    physics_client.configure_debug_visualizer(DebugVisualizerFlag::CovEnableYAxisUp, true);
    physics_client.set_time_step(Duration::from_secs_f64(1. / 60.));
    physics_client.set_gravity(Vector3::new(0.0, -9.8, 0.))?;

    let time_step = Duration::from_secs_f64(1. / 60.);
    let mut panda = PandaSim::new(&mut physics_client, Vector3::zeros())?;
    loop {
        panda.step(&mut physics_client);
        physics_client.step_simulation()?;
        std::thread::sleep(time_step);
    }
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
    const LL: [f64; 9] = [-7.; 9];
    const UL: [f64; 9] = [7.; 9];
    const JR: [f64; 9] = [7.; 9];
    const NULL_SPACE_PARAMETERS: InverseKinematicsNullSpaceParameters<'static> =
        InverseKinematicsNullSpaceParameters {
            lower_limits: &PandaSim::LL,
            upper_limits: &PandaSim::UL,
            joint_ranges: &PandaSim::JR,
            rest_poses: &PandaSim::INITIAL_JOINT_POSITIONS,
        };
    pub fn new(client: &mut PhysicsClient, offset: Vector3<f64>) -> Result<Self, Error> {
        let transform = Isometry3::new(
            Vector3::new(0., 0., -0.6) + offset.clone(),
            Rotation3::from(UnitQuaternion::from_quaternion(Quaternion::new(
                0.5, -0.5, -0.5, -0.5,
            )))
            .scaled_axis(),
        );
        let urdf_options = UrdfOptions {
            base_transform: transform.clone(),
            flags: LoadModelFlags::URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
            ..Default::default()
        };
        client.load_urdf("tray/traybox.urdf", urdf_options)?;

        let transform = Isometry3::translation(offset.x + 0.1, offset.y + 0.3, offset.z - 0.5);
        let urdf_options = UrdfOptions {
            base_transform: transform.clone(),
            flags: LoadModelFlags::URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
            ..Default::default()
        };
        client.load_urdf("lego/lego.urdf", urdf_options)?;

        let transform = Isometry3::translation(offset.x - 0.1, offset.y + 0.3, offset.z - 0.5);
        let urdf_options = UrdfOptions {
            base_transform: transform.clone(),
            flags: LoadModelFlags::URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
            ..Default::default()
        };
        client.load_urdf("lego/lego.urdf", urdf_options)?;

        let transform = Isometry3::translation(offset.x + 0.1, offset.y + 0.3, offset.z - 0.7);
        let urdf_options = UrdfOptions {
            base_transform: transform.clone(),
            flags: LoadModelFlags::URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
            ..Default::default()
        };
        client.load_urdf("lego/lego.urdf", urdf_options)?;

        let transform = Isometry3::translation(offset.x, offset.y + 0.3, offset.z - 0.6);
        let urdf_options = UrdfOptions {
            base_transform: transform.clone(),
            flags: LoadModelFlags::URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
            ..Default::default()
        };
        client.load_urdf("sphere_small.urdf", urdf_options)?;

        let transform = Isometry3::translation(offset.x, offset.y + 0.3, offset.z - 0.5);
        let urdf_options = UrdfOptions {
            base_transform: transform.clone(),
            flags: LoadModelFlags::URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
            ..Default::default()
        };
        client.load_urdf("sphere_small.urdf", urdf_options)?;

        let transform = Isometry3::translation(offset.x, offset.y + 0.3, offset.z - 0.7);
        let urdf_options = UrdfOptions {
            base_transform: transform.clone(),
            flags: LoadModelFlags::URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
            ..Default::default()
        };
        client.load_urdf("sphere_small.urdf", urdf_options)?;
        let cube_start_position = Isometry3::new(
            offset.clone(),
            UnitQuaternion::from_euler_angles(-PI / 2., 0., 0.).scaled_axis(),
        );
        let urdf_options = UrdfOptions {
            use_fixed_base: true,
            base_transform: cube_start_position.clone(),
            flags: LoadModelFlags::URDF_ENABLE_CACHED_GRAPHICS_SHAPES,
            ..Default::default()
        };
        let panda_id = client.load_urdf("franka_panda/panda.urdf", urdf_options)?;
        client.change_dynamics_linear_damping(panda_id, 0.);
        client.change_dynamics_angular_damping(panda_id, 0.);
        let mut index = 0;
        for i in 0..client.get_num_joints(panda_id) {
            let info = client.get_joint_info(panda_id, i);
            if info.joint_type == JointType::Revolute || info.joint_type == JointType::Prismatic {
                client.reset_joint_state(
                    panda_id,
                    i,
                    PandaSim::INITIAL_JOINT_POSITIONS[index],
                    None,
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
        .use_null_space(PandaSim::NULL_SPACE_PARAMETERS)
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
    }
}
