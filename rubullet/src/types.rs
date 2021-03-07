//! Custom data types for RuBullet
use crate::Error;
use image::{ImageBuffer, Luma, RgbaImage};
use nalgebra::{
    DVector, Isometry3, Matrix3xX, Matrix6xX, Point3, Quaternion, Translation3, UnitQuaternion,
    Vector3, Vector6, U3,
};
use rubullet_sys::{
    b3BodyInfo, b3ContactPointData, b3DynamicsInfo, b3JointInfo, b3JointSensorState, b3LinkState,
    b3UserConstraint, b3VisualShapeData,
};
use std::convert::TryFrom;
use std::ffi::CStr;

use std::os::raw::c_int;
use std::path::PathBuf;

/// The unique ID for a body within a physics server.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct BodyId(pub(crate) c_int);

/// The unique ID for a Visual Shape
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct VisualId(pub(crate) c_int);
impl VisualId {
    /// Use it to create an object which does not have a visual appearance. It will be just be
    /// the CollisionShape colored in red.
    pub const NONE: VisualId = VisualId(-1);
}
/// The unique ID for a Collision Shape.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct CollisionId(pub(crate) c_int);

impl CollisionId {
    /// Use it to create an object which does not collide with anything.
    pub const NONE: CollisionId = CollisionId(-1);
}

/// The unique ID for a Texture
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct TextureId(pub(crate) c_int);

/// The unique ID for a User Debug Parameter Item
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct ItemId(pub(crate) c_int);

/// The unique ID for a constraint.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct ConstraintId(pub(crate) c_int);

/// An enum to represent different types of joints
#[derive(Debug, PartialEq, Copy, Clone)]
pub enum JointType {
    Revolute = 0,
    Prismatic = 1,
    Spherical = 2,
    Planar = 3,
    Fixed = 4,
    Point2Point = 5,
    Gear = 6,
}

impl TryFrom<i32> for JointType {
    type Error = Error;

    fn try_from(value: i32) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(JointType::Revolute),
            1 => Ok(JointType::Prismatic),
            2 => Ok(JointType::Spherical),
            3 => Ok(JointType::Planar),
            4 => Ok(JointType::Fixed),
            5 => Ok(JointType::Point2Point),
            6 => Ok(JointType::Gear),
            _ => Err(Error::new("could not convert into a valid joint type")),
        }
    }
}

/// Contains basic information about a joint like its type and name. It can be obtained via
/// [`get_joint_info()`](`crate::PhysicsClient::get_joint_info()`)
/// # Example
/// ```rust
/// use rubullet::{PhysicsClient, UrdfOptions};
/// use nalgebra::Isometry3;
/// use rubullet::Mode::Direct;
/// use anyhow::Result;
/// fn main() -> Result<()> {
///
///     let mut client = PhysicsClient::connect(Direct)?;
///     client.set_additional_search_path(
///         "../rubullet-sys/bullet3/libbullet3/examples/pybullet/gym/pybullet_data",
///         )?;
///     let panda_id = client.load_urdf("franka_panda/panda.urdf", UrdfOptions::default())?;
///     let joint_info = client.get_joint_info(panda_id,4);
///     assert_eq!("panda_joint5",joint_info.joint_name);
///     Ok(())
/// }
/// ```
/// # See also
/// * [`JointState`](`crate::types::JointState`) - For information about the current state of the joint.
#[derive(Debug)]
pub struct JointInfo {
    /// the same joint index as the input parameter
    pub joint_index: usize,
    /// the name of the joint, as specified in the URDF (or SDF etc) file
    pub joint_name: String,
    /// type of the joint, this also implies the number of position and velocity variables.
    pub joint_type: JointType,
    /// the first position index in the positional state variables for this body
    pub q_index: i32,
    /// the first velocity index in the velocity state variables for this body
    pub u_index: i32,
    /// reserved
    #[doc(hidden)]
    pub flags: JointInfoFlags,
    /// the joint damping value, as specified in the URDF file
    pub joint_damping: f64,
    /// the joint friction value, as specified in the URDF file
    pub joint_friction: f64,
    /// Positional lower limit for slider and revolute (hinge) joints.
    pub joint_lower_limit: f64,
    /// Positional upper limit for slider and revolute joints. Values ignored in case upper limit <lower limit.
    pub joint_upper_limit: f64,
    /// Maximum force specified in URDF (possibly other file formats) Note that this value is not automatically used. You can use maxForce in 'setJointMotorControl2'.
    pub joint_max_force: f64,
    /// Maximum velocity specified in URDF. Note that the maximum velocity is not used in actual motor control commands at the moment.
    pub joint_max_velocity: f64,
    /// the name of the link, as specified in the URDF (or SDF etc.) file
    pub link_name: String,
    ///joint axis in local frame (ignored for fixed joints)
    pub joint_axis: Vector3<f64>,
    /// joint pose in parent frame
    pub parent_frame_pose: Isometry3<f64>,
    /// parent link index. None means that the base is the parent link
    pub parent_index: Option<usize>,
}
impl From<b3JointInfo> for JointInfo {
    fn from(b3: b3JointInfo) -> Self {
        unsafe {
            let b3JointInfo {
                m_link_name,
                m_joint_name,
                m_joint_type,
                m_q_index,
                m_u_index,
                m_joint_index,
                m_flags,
                m_joint_damping,
                m_joint_friction,
                m_joint_upper_limit,
                m_joint_lower_limit,
                m_joint_max_force,
                m_joint_max_velocity,
                m_parent_frame,
                m_child_frame: _,
                m_joint_axis,
                m_parent_index,
                m_q_size: _,
                m_u_size: _,
            } = b3;
            let parent_index = match m_parent_index {
                -1 => None,
                index => Some(index as usize),
            };

            JointInfo {
                link_name: CStr::from_ptr(m_link_name.as_ptr())
                    .to_string_lossy()
                    .into_owned(),
                joint_name: CStr::from_ptr(m_joint_name.as_ptr())
                    .to_string_lossy()
                    .into_owned(),
                joint_type: JointType::try_from(m_joint_type).unwrap(),
                q_index: m_q_index,
                u_index: m_u_index,
                joint_index: m_joint_index as usize,
                flags: JointInfoFlags::from_bits(m_flags).expect("Could not parse JointInfoFlags"),
                joint_damping: m_joint_damping,
                joint_friction: m_joint_friction,
                joint_upper_limit: m_joint_upper_limit,
                joint_lower_limit: m_joint_lower_limit,
                joint_max_force: m_joint_max_force,
                joint_max_velocity: m_joint_max_velocity,
                parent_frame_pose: Isometry3::<f64>::from_parts(
                    Translation3::from(Vector3::from_column_slice(&m_parent_frame[0..4])),
                    UnitQuaternion::from_quaternion(Quaternion::from_parts(
                        m_parent_frame[6],
                        Vector3::from_column_slice(&m_parent_frame[3..6]),
                    )),
                ),
                joint_axis: m_joint_axis.into(),
                parent_index,
            }
        }
    }
}
/// Parameters for Inverse Kinematics using the Nullspace
pub struct InverseKinematicsNullSpaceParameters<'a> {
    pub lower_limits: &'a [f64],
    pub upper_limits: &'a [f64],
    pub joint_ranges: &'a [f64],
    /// Favor an IK solution closer to a given rest pose
    pub rest_poses: &'a [f64],
}
/// Parameters for the [`calculate_inverse_kinematics()`](`crate::client::PhysicsClient::calculate_inverse_kinematics()`)
/// You can easily create them using the [`InverseKinematicsParametersBuilder`](`InverseKinematicsParametersBuilder`)
pub struct InverseKinematicsParameters<'a> {
    /// end effector link index
    pub end_effector_link_index: usize,
    /// Target position of the end effector (its link coordinate, not center of mass coordinate!).
    /// By default this is in Cartesian world space, unless you provide current_position joint angles.
    pub target_position: Point3<f64>,
    /// Target orientation in Cartesian world space.
    /// If not specified, pure position IK will be used.
    pub target_orientation: Option<UnitQuaternion<f64>>,
    /// Optional null-space IK
    pub limits: Option<InverseKinematicsNullSpaceParameters<'a>>,
    /// joint_damping allows to tune the IK solution using joint damping factors
    pub joint_damping: Option<&'a [f64]>,
    /// Solver which should be used for the Inverse Kinematics
    pub solver: IkSolver,
    /// By default RuBullet uses the joint positions of the body.
    /// If provided, the target_position and target_orientation is in local space!
    pub current_position: Option<&'a [f64]>,
    /// Refine the IK solution until the distance between target and actual end effector position
    /// is below the residual threshold, or the max_num_iterations is reached
    pub max_num_iterations: Option<usize>,
    /// Refine the IK solution until the distance between target and actual end effector position
    /// is below this threshold, or the max_num_iterations is reached
    pub residual_threshold: Option<f64>,
}
/// Specifies which Inverse Kinematics Solver to use in
/// [`calculate_inverse_kinematics()`](`crate::client::PhysicsClient::calculate_inverse_kinematics()`)
pub enum IkSolver {
    /// Damped Least Squares
    Dls = 0,
    /// Selective Damped Least
    Sdls = 1,
}

impl From<IkSolver> for i32 {
    fn from(solver: IkSolver) -> Self {
        solver as i32
    }
}

impl<'a> Default for InverseKinematicsParameters<'a> {
    fn default() -> Self {
        InverseKinematicsParameters {
            end_effector_link_index: 0,
            target_position: Point3::new(0., 0., 0.),
            target_orientation: None,
            limits: None,
            joint_damping: None,
            solver: IkSolver::Dls,
            current_position: None,
            max_num_iterations: None,
            residual_threshold: None,
        }
    }
}

/// creates [`InverseKinematicsParameters`](`InverseKinematicsParameters`) using the Builder Pattern
/// which can then be used in [`calculate_inverse_kinematics()`](`crate::client::PhysicsClient::calculate_inverse_kinematics()`).
/// Use the [build()](`Self::build()`) method to get the parameters.
/// ```rust
/// # use rubullet::{InverseKinematicsParametersBuilder, BodyId, InverseKinematicsNullSpaceParameters, PhysicsClient, UrdfOptions};
/// # use nalgebra::Isometry3;
/// const INITIAL_JOINT_POSITIONS: [f64; 9] =
///     [0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02];
/// const PANDA_NUM_DOFS: usize = 7;
/// const PANDA_END_EFFECTOR_INDEX: usize = 11;
/// const LL: [f64; 9] = [-7.; 9]; // size is 9 = 7 DOF + 2 DOF for the gripper
/// const UL: [f64; 9] = [7.; 9]; // size is 9 = 7 DOF + 2 DOF for the gripper
/// const JR: [f64; 9] = [7.; 9]; // size is 9 = 7 DOF + 2 DOF for the gripper
/// const NULL_SPACE_PARAMETERS: InverseKinematicsNullSpaceParameters<'static> =
///    InverseKinematicsNullSpaceParameters {
///        lower_limits: &LL,
///        upper_limits: &UL,
///        joint_ranges: &JR,
///        rest_poses: &INITIAL_JOINT_POSITIONS,
///    };
/// let inverse_kinematics_parameters = InverseKinematicsParametersBuilder::new(
///             PANDA_END_EFFECTOR_INDEX,
///             &Isometry3::translation(0.3,0.3,0.3),
///         )
///         .set_max_num_iterations(5)
///         .use_null_space(NULL_SPACE_PARAMETERS)
///         .build();
/// ```
pub struct InverseKinematicsParametersBuilder<'a> {
    params: InverseKinematicsParameters<'a>,
}

impl<'a> InverseKinematicsParametersBuilder<'a> {
    /// creates a new InverseKinematicsParametersBuilder
    /// # Arguments
    /// * `end_effector_link_index` -  end effector link index
    /// * `target_pose` - target pose of the end effector in its link coordinate (not CoM).
    /// use [`ignore_orientation()`](`Self::ignore_orientation()`) if you do not want to consider the orientation
    pub fn new(end_effector_link_index: usize, target_pose: &'a Isometry3<f64>) -> Self {
        let target_position: Point3<f64> = target_pose.translation.vector.into();
        let params = InverseKinematicsParameters {
            end_effector_link_index,
            target_position,
            target_orientation: Some(target_pose.rotation),
            ..Default::default()
        };
        InverseKinematicsParametersBuilder { params }
    }
    /// Do not consider the orientation while calculating the IK
    pub fn ignore_orientation(mut self) -> Self {
        self.params.target_orientation = None;
        self
    }
    /// Consider the nullspace when calculating the IK
    pub fn use_null_space(mut self, limits: InverseKinematicsNullSpaceParameters<'a>) -> Self {
        self.params.limits = Some(limits);
        self
    }
    /// Allow to tune the IK solution using joint damping factors
    pub fn set_joint_damping(mut self, joint_damping: &'a [f64]) -> Self {
        self.params.joint_damping = Some(joint_damping);
        self
    }
    /// Use a different IK-Solver. The default is DLS
    pub fn set_ik_solver(mut self, solver: IkSolver) -> Self {
        self.params.solver = solver;
        self
    }
    /// Specify the current joint position if you do not want to use the position of the body.
    /// If you use it the target pose will be in local space!
    pub fn set_current_position(mut self, current_position: &'a [f64]) -> Self {
        self.params.current_position = Some(current_position);
        self
    }
    /// Sets the maximum number of iterations. The default is 20.
    pub fn set_max_num_iterations(mut self, iterations: usize) -> Self {
        self.params.max_num_iterations = Some(iterations);
        self
    }
    /// Recalculate the IK until the distance between target and actual end effector is smaller than
    /// the residual threshold or max_num_iterations is reached.
    pub fn set_residual_threshold(mut self, residual_threshold: f64) -> Self {
        self.params.residual_threshold = Some(residual_threshold);
        self
    }
    /// creates the parameters
    pub fn build(self) -> InverseKinematicsParameters<'a> {
        self.params
    }
}
/// Represents options for [`add_user_debug_text`](`crate::PhysicsClient::add_user_debug_text()`)
pub struct AddDebugTextOptions {
    /// RGB color [Red, Green, Blue] each component in range [0..1]. Default is [1.,1.,1.]
    pub text_color_rgb: [f64; 3],
    /// size of the text. Default is 1.
    pub text_size: f64,
    /// Use 0 for permanent text, or positive time in seconds
    /// (afterwards the line with be removed automatically). Default is 0.
    pub life_time: f64,
    /// If not specified the text will always face the camera (Default behavior).
    /// By specifying a text orientation (quaternion), the orientation will be fixed in world space
    /// or local space (when parent is specified). Note that a different implementation/shader is
    /// used for camera facing text, with different appearance: camera facing text uses bitmap
    /// fonts, text with specified orientation uses TrueType fonts.
    pub text_orientation: Option<UnitQuaternion<f64>>,
    /// If specified the text will be drawn relative to the parents object coordinate system.
    pub parent_object_id: Option<BodyId>,
    /// When using "parent_object_id" you can also define in which link the coordinate system should be.
    /// By default it is the base frame (-1)
    pub parent_link_index: Option<usize>,
    /// replace an existing text item (to avoid flickering of remove/add)
    pub replace_item_id: Option<ItemId>,
}

impl Default for AddDebugTextOptions {
    fn default() -> Self {
        AddDebugTextOptions {
            text_color_rgb: [1.; 3],
            text_size: 1.,
            life_time: 0.,
            text_orientation: None,
            parent_object_id: None,
            parent_link_index: None,
            replace_item_id: None,
        }
    }
}
/// Represents options for [`add_user_debug_line`](`crate::PhysicsClient::add_user_debug_line()`)
pub struct AddDebugLineOptions {
    /// RGB color [Red, Green, Blue] each component in range [0..1]. Default is [1.,1.,1.]
    pub line_color_rgb: [f64; 3],
    /// line width (limited by OpenGL implementation). Default is 1.
    pub line_width: f64,
    /// Use 0 for a permanent line, or positive time in seconds
    /// (afterwards the line with be removed automatically). Default is 0.
    pub life_time: f64,
    /// If specified the line will be drawn relative to the parents object coordinate system.
    pub parent_object_id: Option<BodyId>,
    /// When using "parent_object_id" you can also define in which link the coordinate system should be.
    /// By default it is the base frame (-1)
    pub parent_link_index: Option<usize>,
    /// replace an existing line (to improve performance and to avoid flickering of remove/add)
    pub replace_item_id: Option<ItemId>,
}

impl Default for AddDebugLineOptions {
    fn default() -> Self {
        AddDebugLineOptions {
            line_color_rgb: [1.; 3],
            line_width: 1.,
            life_time: 0.,
            parent_object_id: None,
            parent_link_index: None,
            replace_item_id: None,
        }
    }
}

/// Specifies a jacobian with 6 rows.
/// The jacobian is split into a linear part and an angular part.
/// # Example
/// Jacobian can be multiplied with joint velocities to get a velocity in cartesian coordinates:
/// ```rust
/// # use rubullet::{Velocity, Jacobian};
/// # use nalgebra::{Matrix6xX, DVector};
/// let jacobian = Jacobian{jacobian:Matrix6xX::from_vec(vec![0.;12])};
/// let velocity: Velocity = jacobian * DVector::from_vec(vec![1.;2]);
/// ```
///
/// # See also
/// * [`PhysicsClient::calculate_jacobian()`](`crate::PhysicsClient::calculate_jacobian()`)
#[derive(Debug, Clone)]
pub struct Jacobian {
    pub jacobian: Matrix6xX<f64>,
}

impl<T: Into<DVector<f64>>> std::ops::Mul<T> for Jacobian {
    type Output = Velocity;

    fn mul(self, q_dot: T) -> Self::Output {
        let vel = self.jacobian * q_dot.into();
        Velocity(vel)
    }
}
impl Jacobian {
    /// Linear part of the the jacobian (first 3 rows)
    pub fn get_linear_jacobian(&self) -> Matrix3xX<f64> {
        Matrix3xX::from(self.jacobian.fixed_rows::<U3>(0))
    }
    /// Angular part of the the jacobian (last 3 rows)
    pub fn get_angular_jacobian(&self) -> Matrix3xX<f64> {
        Matrix3xX::from(self.jacobian.fixed_rows::<U3>(3))
    }
}
/// Frame for [`apply_external_torque()`](`crate::PhysicsClient::apply_external_torque()`) and
/// [`apply_external_force()`](`crate::PhysicsClient::apply_external_force()`)
pub enum ExternalForceFrame {
    /// Local Link Coordinates
    LinkFrame = 1,
    /// Cartesian World Coordinates
    WorldFrame = 2,
}
/// Represents a key press Event
#[derive(Debug, Copy, Clone, Default)]
pub struct KeyboardEvent {
    /// specifies which key the event is about.
    pub key: char,
    pub(crate) key_state: i32,
}

impl KeyboardEvent {
    /// is true when the key goes from an "up" to a "down" state.
    pub fn was_triggered(&self) -> bool {
        self.key_state & 2 == 2
    }
    /// is true when the key is currently pressed.
    pub fn is_down(&self) -> bool {
        self.key_state & 1 == 1
    }
    /// is true when the key goes from a "down" to an "up" state.
    pub fn is_released(&self) -> bool {
        self.key_state & 4 == 4
    }
}
/// Mouse Events can either be a "Move" or a "Button" event. A "Move" event is when the mouse is moved
/// in the OpenGL window and a "Button" even is when a mouse button is clicked.
#[derive(Debug, Copy, Clone)]
pub enum MouseEvent {
    /// Contains the mouse position
    Move {
        /// x-coordinate of the mouse pointer
        mouse_pos_x: f32,
        /// y-coordinate of the mouse pointer
        mouse_pos_y: f32,
    },
    /// Specifies Mouse Position and a Button event
    Button {
        /// x-coordinate of the mouse pointer
        mouse_pos_x: f32,
        /// y-coordinate of the mouse pointer
        mouse_pos_y: f32,
        /// button index for left/middle/right mouse button
        button_index: i32,
        /// state of the mouse button
        button_state: MouseButtonState,
    },
}

/// Represents the different possible states of a mouse button
#[derive(Debug, Copy, Clone)]
pub struct MouseButtonState {
    pub(crate) flag: i32,
}
impl MouseButtonState {
    /// is true when the button goes from an "unpressed" to a "pressed" state.
    pub fn was_triggered(&self) -> bool {
        self.flag & 2 == 2
    }
    /// is true when the button is in a "pressed" state.
    pub fn is_pressed(&self) -> bool {
        self.flag & 1 == 1
    }
    /// is true when the button goes from a "pressed" to an "unpressed" state.
    pub fn is_released(&self) -> bool {
        self.flag & 4 == 4
    }
}
/// Represents the current state of a joint. It can be retrieved via [`get_joint_state()`](`crate::PhysicsClient::get_joint_state()`)
/// # Note
/// joint_force_torque will be [0.;6] if the sensor is not enabled via
/// [`enable_joint_torque_sensor()`](`crate::PhysicsClient::enable_joint_torque_sensor()`)
/// # See also
/// * [`JointInfo`](`JointInfo`) - For basic information about a joint
#[derive(Debug, Default, Copy, Clone)]
pub struct JointState {
    /// The position value of this joint.
    pub joint_position: f64,
    /// The velocity value of this joint.
    pub joint_velocity: f64,
    /// These are the joint reaction forces, if a torque sensor is enabled for this joint it is [Fx, Fy, Fz, Mx, My, Mz].
    /// Without torque sensor, it is \[0,0,0,0,0,0\].
    /// This is is NOT the motor torque/force, but the spatial reaction force vector at joint.
    pub joint_force_torque: [f64; 6],
    /// This is the motor torque applied during the last [`step_simulation()`](`crate::PhysicsClient::step_simulation()`).
    /// Note that this only applies in velocity and position control.
    /// If you use torque control then the applied joint motor torque is exactly what you provide,
    /// so there is no need to report it separately.
    pub joint_motor_torque: f64,
}
impl From<b3JointSensorState> for JointState {
    fn from(b3: b3JointSensorState) -> Self {
        let b3JointSensorState {
            m_joint_position,
            m_joint_velocity,
            m_joint_force_torque,
            m_joint_motor_torque,
        } = b3;
        JointState {
            joint_position: m_joint_position,
            joint_velocity: m_joint_velocity,
            joint_force_torque: m_joint_force_torque,
            joint_motor_torque: m_joint_motor_torque,
        }
    }
}

/// Options for loading a URDF into the physics server.
pub struct UrdfOptions {
    /// Creates the base of the object with the given transform.
    pub base_transform: Isometry3<f64>,

    /// Forces the base of the loaded object to be static.
    pub use_fixed_base: bool,
    /// Experimental. By default, the joints in the URDF file are created using the reduced
    /// coordinate method: the joints are simulated using the
    /// Featherstone Articulated Body Algorithm (ABA, btMultiBody in Bullet 2.x).
    /// The use_maximal_coordinates option will create a 6 degree of freedom rigid body for each link,
    /// and constraints between those rigid bodies are used to model joints.
    pub use_maximal_coordinates: Option<bool>,

    /// Flags for loading the model.
    pub flags: LoadModelFlags,
    /// Applies a scale factor to the model.
    pub global_scaling: f64,
}

impl Default for UrdfOptions {
    fn default() -> UrdfOptions {
        UrdfOptions {
            base_transform: Isometry3::identity(),
            use_fixed_base: false,
            use_maximal_coordinates: None,
            global_scaling: -1.0,
            flags: LoadModelFlags::NONE,
        }
    }
}
/// Options for loading models from an SDF file into the physics server.
pub struct SdfOptions {
    /// Experimental. By default, the joints in the URDF file are created using the reduced
    /// coordinate method: the joints are simulated using the
    /// Featherstone Articulated Body Algorithm (ABA, btMultiBody in Bullet 2.x).
    /// The use_maximal_coordinates option will create a 6 degree of freedom rigid body for each link,
    /// and constraints between those rigid bodies are used to model joints.
    pub use_maximal_coordinates: bool,
    /// Applies a scale factor to the model.
    pub global_scaling: f64,
}

impl Default for SdfOptions {
    fn default() -> Self {
        SdfOptions {
            use_maximal_coordinates: false,
            global_scaling: 1.0,
        }
    }
}
/// The Control Mode specifies how the robot should move (Position Control, Velocity Control, Torque Control)
/// Each Control Mode has its own set of Parameters. The Position mode for example takes a desired joint
/// position as input. It can be used in [`set_joint_motor_control()`](`crate::client::PhysicsClient::set_joint_motor_control()`)
///
/// | Mode                    | Implementation | Component                        | Constraint error to be minimized                                                                          |
/// |-------------------------|----------------|----------------------------------|-----------------------------------------------------------------------------------------------------------|
/// | Position,PositionWithPd | constraint     | velocity and position constraint | error = position_gain*(desired_position-actual_position)+velocity_gain*(desired_velocity-actual_velocity) |
/// | Velocity                | constraint     | pure velocity constraint         | error = desired_velocity - actual_velocity                                                                |
/// | Torque                  | External Force |                                  |                                                                                                           |
/// | Pd                      | ???            | ???                              | ???                                                                                                       |
pub enum ControlMode {
    /// Position Control with the desired joint position.
    Position(f64),
    /// Same as Position, but you can set your own gains
    PositionWithPd {
        /// desired target position
        target_position: f64,
        /// desired target velocity
        target_velocity: f64,
        /// position gain
        position_gain: f64,
        /// velocity gain
        velocity_gain: f64,
        /// limits the velocity of a joint
        maximum_velocity: Option<f64>,
    },
    /// Velocity control with the desired joint velocity
    Velocity(f64),
    /// Torque control with the desired joint torque.
    Torque(f64),
    /// PD Control
    Pd {
        /// desired target position
        target_position: f64,
        /// desired target velocity
        target_velocity: f64,
        /// position gain
        position_gain: f64,
        /// velocity gain
        velocity_gain: f64,
        /// limits the velocity of a joint
        maximum_velocity: Option<f64>,
    },
}

impl ControlMode {
    pub(crate) fn get_int(&self) -> i32 {
        match self {
            ControlMode::Position(_) => 2,
            ControlMode::Velocity(_) => 0,
            ControlMode::Torque(_) => 1,
            ControlMode::Pd { .. } => 3,
            ControlMode::PositionWithPd { .. } => 2,
        }
    }
}
/// Can be used in [`set_joint_motor_control_array()`](`crate::client::PhysicsClient::set_joint_motor_control_array()`).
/// It is basically the same as [`ControlMode`](`ControlMode`) but with arrays. See [`ControlMode`](`ControlMode`) for details.
pub enum ControlModeArray<'a> {
    /// Position Control with the desired joint positions.
    Positions(&'a [f64]),
    /// Same as Positions, but you can set your own gains
    PositionsWithPd {
        /// desired target positions
        target_positions: &'a [f64],
        /// desired target velocities
        target_velocities: &'a [f64],
        /// position gains
        position_gains: &'a [f64],
        /// velocity gains
        velocity_gains: &'a [f64],
    },
    /// Velocity control with the desired joint velocities
    Velocities(&'a [f64]),
    /// Torque control with the desired joint torques.
    Torques(&'a [f64]),
    /// PD Control
    Pd {
        /// desired target positions
        target_positions: &'a [f64],
        /// desired target velocities
        target_velocities: &'a [f64],
        /// position gains
        position_gains: &'a [f64],
        /// velocity gains
        velocity_gains: &'a [f64],
    },
}

impl ControlModeArray<'_> {
    pub(crate) fn get_int(&self) -> i32 {
        match self {
            ControlModeArray::Positions(_) => 2,
            ControlModeArray::Velocities(_) => 0,
            ControlModeArray::Torques(_) => 1,
            ControlModeArray::Pd { .. } => 3,
            ControlModeArray::PositionsWithPd { .. } => 2,
        }
    }
}
/// Flags for [`configure_debug_visualizer()`](`crate::PhysicsClient::configure_debug_visualizer`)
pub enum DebugVisualizerFlag {
    CovEnableGui = 1,
    CovEnableShadows,
    CovEnableWireframe,
    CovEnableVrTeleporting,
    CovEnableVrPicking,
    CovEnableVrRenderControllers,
    CovEnableRendering,
    CovEnableSyncRenderingInternal,
    CovEnableKeyboardShortcuts,
    CovEnableMousePicking,
    CovEnableYAxisUp,
    CovEnableTinyRenderer,
    CovEnableRgbBufferPreview,
    CovEnableDepthBufferPreview,
    CovEnableSegmentationMarkPreview,
    CovEnablePlanarReflection,
    CovEnableSingleStepRendering,
}

/// Describes the State of a Link
/// # Kind of Frames
/// * `world_frame` - center of mass
/// * `local_intertial_frame` - offset to the CoM expressed in the URDF link frame
/// * `world_link_frame` - URDF link frame
/// ### Relationships between Frames
/// urdfLinkFrame = comLinkFrame * localInertialFrame.inverse()
/// ```rust
/// use rubullet::{PhysicsClient, UrdfOptions};
/// use nalgebra::Isometry3;
/// use rubullet::Mode::Direct;
/// use anyhow::Result;
/// fn main() -> Result<()> {
///     let mut client = PhysicsClient::connect(Direct)?;
///     client.set_additional_search_path(
///         "../rubullet-sys/bullet3/libbullet3/examples/pybullet/gym/pybullet_data",
///         )?;
///     let panda_id = client.load_urdf("franka_panda/panda.urdf", UrdfOptions::default())?;
///     let link_state = client.get_link_state(panda_id, 11, true, true)?;
///     // urdfLinkFrame = comLinkFrame * localInertialFrame.inverse()
///     let urdf_frame = link_state.world_pose * link_state.local_inertial_pose.inverse();
///     // print both frames to see that they are about the same
///     println!("{}", link_state.world_link_frame_pose);
///     println!("{}", urdf_frame);
///     // as they are both almost the same calculating the difference:
///     // urdfLinkFrame.inverse() * world_link_frame_pose
///     // should return something very close the identity matrix I.
///     let identity = urdf_frame.inverse() * link_state.world_link_frame_pose;
///     assert!(identity.translation.vector.norm() < 1e-7);
///     assert!(identity.rotation.angle() < 1e-7);
///     Ok(())
/// }
/// ```
///
/// # See also
/// * [`get_link_state()`](`crate::client::PhysicsClient::get_link_state()`)
/// * [`get_link_states()`](`crate::client::PhysicsClient::get_link_states()`)
#[derive(Debug)]
pub struct LinkState {
    /// Cartesian pose of the center of mass
    pub world_pose: Isometry3<f64>,
    /// local offset of the inertial frame (center of mass) express in the URDF link frame
    pub local_inertial_pose: Isometry3<f64>,
    /// world pose of the URDF link frame
    pub world_link_frame_pose: Isometry3<f64>,
    /// Cartesian world linear velocity.
    pub world_velocity: Option<Velocity>,
}
impl LinkState {
    /// conveniently returns the linear world velocity or an error if the velocity was not calculated
    /// for the LinkState. Be sure to set `compute_link_velocity` to true in
    /// [`get_link_state()`](`crate::client::PhysicsClient::get_link_state()`)
    pub fn get_linear_world_velocity(&self) -> Result<Vector3<f64>, Error> {
        match &self.world_velocity {
            None => {Err(Error::new("LinkState contains no velocity. You have to set compute_link_velocity to true in get_link_state() to get the velocity"))}
            Some(velocity) => {Ok(velocity.get_linear_velocity())}
        }
    }
    /// conveniently returns the angular world velocity or an error if the velocity was not calculated
    /// for the LinkState. Be sure to set `compute_link_velocity` to true in
    /// [`get_link_state()`](`crate::client::PhysicsClient::get_link_state()`)
    pub fn get_angular_world_velocity(&self) -> Result<Vector3<f64>, Error> {
        match &self.world_velocity {
            None => {Err(Error::new("LinkState contains no velocity. You have to set compute_link_velocity to true in get_link_state() to get the velocity"))}
            Some(velocity) => {Ok(velocity.get_angular_velocity())}
        }
    }
    /// conveniently returns the world velocity or an error if the velocity was not calculated
    /// for the LinkState. Be sure to set `compute_link_velocity` to true in
    /// [`get_link_state()`](`crate::client::PhysicsClient::get_link_state()`)
    pub fn get_world_velocity(&self) -> Result<&Velocity, Error> {
        match &self.world_velocity {
            None => {Err(Error::new("LinkState contains no velocity. You have to set compute_link_velocity to true in get_link_state() to get the velocity"))}
            Some(velocity) => {Ok(velocity)}
        }
    }
    /// conveniently returns the world velocity vector (x,y,z,wx,w,wz) or an error if the velocity was not calculated
    /// for the LinkState. Be sure to set `compute_link_velocity` to true in
    /// [`get_link_state()`](`crate::client::PhysicsClient::get_link_state()`)
    pub fn get_world_velocity_vector(&self) -> Result<Vector6<f64>, Error> {
        match &self.world_velocity {
            None => {Err(Error::new("LinkState contains no velocity. You have to set compute_link_velocity to true in get_link_state() to get the velocity"))}
            Some(velocity) => {Ok(velocity.to_vector())}
        }
    }
}
impl From<(b3LinkState, bool)> for LinkState {
    fn from(b3: (b3LinkState, bool)) -> Self {
        let (
            b3LinkState {
                m_world_position,
                m_world_orientation,
                m_local_inertial_position,
                m_local_inertial_orientation,
                m_world_link_frame_position,
                m_world_link_frame_orientation,
                m_world_linear_velocity,
                m_world_angular_velocity,
                m_world_aabb_min: _,
                m_world_aabb_max: _,
            },
            velocity_valid,
        ) = b3;
        let mut state = LinkState {
            world_pose: position_orientation_to_isometry(m_world_position, m_world_orientation),
            local_inertial_pose: position_orientation_to_isometry(
                m_local_inertial_position,
                m_local_inertial_orientation,
            ),
            world_link_frame_pose: position_orientation_to_isometry(
                m_world_link_frame_position,
                m_world_link_frame_orientation,
            ),
            world_velocity: None,
        };
        if velocity_valid {
            let velocity: [f64; 6] = [
                m_world_linear_velocity[0],
                m_world_linear_velocity[1],
                m_world_linear_velocity[2],
                m_world_angular_velocity[0],
                m_world_angular_velocity[1],
                m_world_angular_velocity[2],
            ];
            state.world_velocity = Some(velocity.into());
        }
        state
    }
}

fn position_orientation_to_isometry(position: [f64; 3], orientation: [f64; 4]) -> Isometry3<f64> {
    Isometry3::<f64>::from_parts(
        Translation3::from(Vector3::from_column_slice(&position)),
        UnitQuaternion::from_quaternion(Quaternion::from_parts(
            orientation[3],
            Vector3::from_column_slice(&orientation[0..3]),
        )),
    )
}
fn combined_position_orientation_array_to_isometry(combined: [f64; 7]) -> Isometry3<f64> {
    let position = [combined[0], combined[1], combined[2]];
    let orientation = [combined[3], combined[4], combined[5], combined[6]];
    position_orientation_to_isometry(position, orientation)
}

/// VisualShape options are for the [create_visual_shape](`crate::PhysicsClient::create_visual_shape`)
/// function to specify additional options like the color.
pub struct VisualShapeOptions {
    /// offset of the shape with respect to the link frame
    pub frame_offset: Isometry3<f64>,
    /// color components for red, green, blue and alpha, each in range \[0,1\]
    pub rgba_colors: [f64; 4],
    /// specular reflection color, red, green, blue components in range \[0,1\]
    pub specular_colors: [f64; 3],
    /// Additional flags. Currently not used
    #[doc(hidden)]
    pub flags: Option<i32>,
}
impl Default for VisualShapeOptions {
    fn default() -> Self {
        VisualShapeOptions {
            frame_offset: Isometry3::translation(0., 0., 0.),
            rgba_colors: [1.; 4],
            specular_colors: [1.; 3],
            flags: None,
        }
    }
}
/// Collision shape which can be put
/// the [create_collision_shape](`crate::PhysicsClient::create_collision_shape`) method
pub enum GeometricCollisionShape {
    /// A Sphere determined by the radius in meter
    Sphere {
        /// radius in meter
        radius: f64,
    },
    /// A Cuboid
    Box {
        /// \[x,y,z\] lengths starting from the middle of the box.
        /// For example Vector3::new(0.5,0.5,0.5) would be a unit cube.
        half_extents: Vector3<f64>,
    },
    /// Like a cylinder but with a half sphere on each end. The total length of a capsule is
    /// length + 2 * radius.
    Capsule {
        /// radius of the cylindric part of the capsule in meter.
        radius: f64,
        /// height of the cylindric part in meter. The half spheres are put on top on that
        height: f64,
    },
    /// A Cylinder
    Cylinder {
        /// radius in meter
        radius: f64,
        /// height in meter
        height: f64,
    },
    /// A Plane.
    Plane {
        /// normal of the plane.
        plane_normal: Vector3<f64>,
    },
    /// Load a .obj (Wavefront) file. Will create convex hulls for each object.
    MeshFile {
        /// Path to the .obj file.
        filename: PathBuf,
        /// Scaling of the Mesh.Use None if you do not want to apply any scaling.
        mesh_scaling: Option<Vector3<f64>>,
        /// Set to 1 if you want to activate have the GEOM_FORCE_CONCAVE_TRIMESH Flag.
        /// this will create a concave static triangle mesh. This should not be used with
        /// dynamic / moving objects, only for static (mass = 0) terrain.
        flags: Option<i32>,
    },
    /// Create your own mesh.
    Mesh {
        /// list of \[x,y,z\] coordinates.
        vertices: Vec<[f64; 3]>,
        /// triangle indices, should be a multiple of 3
        indices: Option<Vec<i32>>,
        /// Scaling of the Mesh. Use [1.;3] for normal scaling.
        mesh_scaling: Option<Vector3<f64>>,
    },
    /// Loads a Heightfield from a file
    HeightfieldFile {
        /// Path to the .obj file.
        filename: PathBuf,
        /// Scaling of the Mesh.Use None if you do not want to apply any scaling.
        mesh_scaling: Option<Vector3<f64>>,
        /// Texture scaling. Use 1. for original scaling.
        texture_scaling: f64,
    },
    /// Create your own Heightfield. See heightfield.rs for an example.
    Heightfield {
        /// Scaling of the Mesh. Use [1.;3] for normal scaling.
        mesh_scaling: Option<Vector3<f64>>,
        /// Texture scaling. Use 1. for normal scaling.
        texture_scaling: f64,
        /// Heightfield data. Should be of size num_rows * num_columns
        data: Vec<f32>,
        /// number of rows in data
        num_rows: usize,
        /// number of columns in data
        num_columns: usize,
        /// replacing an existing heightfield (updating its heights)
        /// (much faster than removing and re-creating a heightfield)
        replace_heightfield: Option<CollisionId>,
    },
}
/// Visual shapes to put into the [create_visual_shape](`crate::PhysicsClient::create_visual_shape`)
/// method together with [VisualShapeOptions](`VisualShapeOptions`)
pub enum GeometricVisualShape {
    /// A Sphere determined by the radius in meter
    Sphere {
        /// radius in meter
        radius: f64,
    },
    /// A Cuboid
    Box {
        /// \[x,y,z\] lengths starting from the middle of the box.
        /// For example Vector3::new(0.5,0.5,0.5) would be a unit cube.
        half_extents: Vector3<f64>,
    },
    /// Like a cylinder but with a half sphere on each end. The total length of a capsule is
    /// length + 2 * radius.
    Capsule {
        /// radius of the cylindric part of the capsule in meter.
        radius: f64,
        /// length of the cylindric part in meter. The half spheres are put on top on that
        length: f64,
    },
    /// A Cylinder
    Cylinder {
        /// radius in meter
        radius: f64,
        /// length in meter
        length: f64,
    },
    /// A flat Plane. Note that you cannot use a Plane VisualShape in combination with a non Plane
    /// CollisionShape. Also it seems like the visual plane is determined by the collision plane and
    /// thus cannot be adapted through the normal of the visual.
    Plane {
        /// Normal of the plane. Seems to have no effect!
        plane_normal: Vector3<f64>,
    },
    /// Loads a .obj (Wavefront) file. Will create convex hulls for each object.
    MeshFile {
        /// Path to the .obj file.
        filename: PathBuf,
        /// Scaling of the Mesh.Use None if you do not want to apply any scaling.
        mesh_scaling: Option<Vector3<f64>>,
    },
    /// Create your own mesh.
    Mesh {
        /// Scaling of the Mesh. Use [1.;3] for normal scaling.
        mesh_scaling: Option<Vector3<f64>>,
        /// list of \[x,y,z\] coordinates.
        vertices: Vec<[f64; 3]>,
        /// triangle indices, should be a multiple of 3
        indices: Vec<i32>,
        /// uv texture coordinates for vertices.
        /// Use [change_visual_shape](`crate::PhysicsClient::change_visual_shape`)
        /// to choose the texture image. The number of uvs should be equal to number of vertices
        uvs: Option<Vec<[f64; 2]>>,
        /// vertex normals, number should be equal to number of vertices.
        normals: Option<Vec<[f64; 3]>>,
    },
}
/// Specifies all options for [create_multi_body](`crate::PhysicsClient::create_multi_body`).
/// Most of the the time you are probably fine using `MultiBodyOptions::default()` or just setting
/// the base_pose and/or mass
pub struct MultiBodyOptions {
    /// mass of the base, in kg (if using SI units)
    pub base_mass: f64,
    /// Cartesian world pose of the base
    pub base_pose: Isometry3<f64>,
    /// Local pose of inertial frame
    pub base_inertial_frame_pose: Isometry3<f64>,
    /// List of the mass values, one for each link.
    pub link_masses: Vec<f64>,
    /// List of the collision shape unique id, one for each link.
    /// Use [`CollisionId::NONE`](`crate::types::CollisionId::NONE`) if you do not want to have a collision shape.
    pub link_collision_shapes: Vec<CollisionId>,
    /// List of the visual shape unique id, one for each link.
    /// Use [`VisualId::NONE`](`crate::types::VisualId::NONE`) if you do not want to set a visual shape.
    pub link_visual_shapes: Vec<VisualId>,
    /// list of local link poses, with respect to parent
    pub link_poses: Vec<Isometry3<f64>>,
    /// list of local inertial frame poses, in the link frame
    pub link_inertial_frame_poses: Vec<Isometry3<f64>>,
    /// Link index of the parent link or 0 for the base.
    pub link_parent_indices: Vec<i32>,
    /// list of joint types, one for each link.
    pub link_joint_types: Vec<JointType>,
    /// List of joint axis in local frame
    pub link_joint_axis: Vec<Vector3<f64>>,
    /// experimental, best to leave it false.
    pub use_maximal_coordinates: bool,
    /// similar to the flags passed in load_urdf, for example URDF_USE_SELF_COLLISION.
    /// See [`LoadModelFlags`](`LoadModelFlags`) for flags explanation.
    pub flags: Option<LoadModelFlags>,
    /// list of base positions, for fast batch creation of many multibodies.
    /// See create_multi_body_batch.rs example.
    pub batch_positions: Option<Vec<Point3<f64>>>,
}
impl Default for MultiBodyOptions {
    fn default() -> Self {
        MultiBodyOptions {
            base_pose: Isometry3::translation(0., 0., 0.),
            base_inertial_frame_pose: Isometry3::translation(0., 0., 0.),
            base_mass: 0.0,
            link_masses: Vec::new(),
            link_collision_shapes: Vec::new(),
            link_visual_shapes: Vec::new(),
            link_poses: Vec::new(),
            link_inertial_frame_poses: Vec::new(),
            link_parent_indices: Vec::new(),
            link_joint_types: Vec::new(),
            link_joint_axis: Vec::new(),
            use_maximal_coordinates: false,
            flags: None,

            batch_positions: None,
        }
    }
}

/// This struct keeps the information to change a visual shape with the
/// [change_visual_shape](`crate::PhysicsClient::change_visual_shape`) method.
pub struct ChangeVisualShapeOptions {
    /// Experimental for internal use, recommended ignore shapeIndex or leave it -1.
    /// Intention is to let you pick a specific shape index to modify, since URDF (and SDF etc)
    pub shape: VisualId,
    /// texture unique id, as returned by [load_texture](`crate::PhysicsClient::load_texture`) method
    pub texture_id: Option<TextureId>,
    /// color components for RED, GREEN, BLUE and ALPHA, each in range [0..1].
    /// Alpha has to be 0 (invisible) or 1 (visible) at the moment.
    /// Note that TinyRenderer doesn't support transparency, but the GUI/EGL OpenGL3 renderer does.
    pub rgba_color: Option<[f64; 4]>,
    /// specular color components, RED, GREEN and BLUE, can be from 0 to large number (>100).
    pub specular_color: Option<[f64; 3]>,
    /// Not yet used anywhere. But it is in the code.
    #[doc(hidden)]
    pub flags: Option<i32>,
}
impl Default for ChangeVisualShapeOptions {
    fn default() -> Self {
        ChangeVisualShapeOptions {
            shape: VisualId(-1),
            texture_id: None,
            rgba_color: None,
            specular_color: None,
            flags: None,
        }
    }
}
/// Contains the body name and base name of a Body. BodyInfo is returned by
/// [get_body_info](`crate::PhysicsClient::get_body_info`)
#[derive(Debug)]
pub struct BodyInfo {
    /// base name (first link) as extracted from the URDF etc.
    pub base_name: String,
    /// body name (robot name) as extracted from the URDF etc.
    pub body_name: String,
}

impl From<b3BodyInfo> for BodyInfo {
    fn from(info: b3BodyInfo) -> Self {
        unsafe {
            BodyInfo {
                base_name: CStr::from_ptr(info.m_baseName.as_ptr())
                    .to_string_lossy()
                    .into_owned(),
                body_name: CStr::from_ptr(info.m_bodyName.as_ptr())
                    .to_string_lossy()
                    .into_owned(),
            }
        }
    }
}
/// Contains information about the visual shape of a body. It is returned by
/// [get_visual_shape_data](`crate::PhysicsClient::get_visual_shape_data`)
#[derive(Debug)]
pub struct VisualShapeData {
    /// same id as in the input of [get_visual_shape_data](`crate::PhysicsClient::get_visual_shape_data`)
    pub body_id: BodyId,
    /// link index or None for the base
    pub link_index: Option<usize>,
    /// visual geometry type (TBD)
    pub visual_geometry_type: i32,
    /// dimensions (size, local scale) of the geometry
    pub dimensions: [f64; 3],
    /// path to the triangle mesh, if any. Typically relative to the URDF, SDF or
    /// MJCF file location, but could be absolute.
    pub mesh_asset_file_name: String,
    /// of local visual frame relative to link/joint frame
    pub local_visual_frame_pose: Isometry3<f64>,
    /// URDF color (if any specified) in red/green/blue/alpha
    pub rgba_color: [f64; 4],
    /// Id of the texture. Is only some when request_texture_id was set to true
    pub texture_id: Option<TextureId>,
}

impl From<b3VisualShapeData> for VisualShapeData {
    fn from(b3: b3VisualShapeData) -> Self {
        unsafe {
            let link_index = match b3.m_linkIndex {
                -1 => None,
                index => Some(index as usize),
            };
            VisualShapeData {
                body_id: BodyId(b3.m_objectUniqueId),
                link_index,
                visual_geometry_type: b3.m_visualGeometryType,
                dimensions: b3.m_dimensions,
                mesh_asset_file_name: CStr::from_ptr(b3.m_meshAssetFileName.as_ptr())
                    .to_string_lossy()
                    .into_owned(),
                local_visual_frame_pose: combined_position_orientation_array_to_isometry(
                    b3.m_localVisualFrame,
                ),
                rgba_color: b3.m_rgbaColor,
                texture_id: None,
            }
        }
    }
}
/// Stores the images from [`get_camera_image()`](`crate::PhysicsClient::get_camera_image()`)
pub struct Images {
    /// RGB image with additional alpha channel
    pub rgba: RgbaImage,
    /// Depth image. Every pixel represents a distance in meters
    pub depth: ImageBuffer<Luma<f32>, Vec<f32>>,
    /// Segmentation image. Every pixel represents a unique [`BodyId`](`crate::types::BodyId`)
    pub segmentation: ImageBuffer<Luma<i32>, Vec<i32>>,
}

/// Contains the cartesian velocity stored as Vector with 6 elements (x,y,z,wx,wy,wz).
/// # Example
/// ```rust
/// use rubullet::Velocity;
/// use nalgebra::Vector6;
/// let vel: Velocity = [0.; 6].into(); // creation from array
/// let vel: Velocity = Vector6::zeros().into(); // creation from vector
/// ```
#[derive(Debug)]
pub struct Velocity(Vector6<f64>);

impl Velocity {
    /// returns the linear velocity (x,y,z)
    pub fn get_linear_velocity(&self) -> Vector3<f64> {
        self.0.fixed_rows::<U3>(0).into()
    }
    /// returns the angular velocity (wx,wy,wz)
    pub fn get_angular_velocity(&self) -> Vector3<f64> {
        self.0.fixed_rows::<U3>(3).into()
    }
    /// converts the velocity to a Vector6 (x,y,z,wx,wy,wz)
    pub fn to_vector(&self) -> Vector6<f64> {
        self.0
    }
}
impl From<[f64; 6]> for Velocity {
    fn from(input: [f64; 6]) -> Self {
        Velocity(input.into())
    }
}
impl From<Vector6<f64>> for Velocity {
    fn from(input: Vector6<f64>) -> Self {
        Velocity(input)
    }
}
bitflags::bitflags! {
    /// Use flag for loading the model. Flags can be combined with the `|`-operator.
    /// Example:
    /// ```rust
    ///# use rubullet::LoadModelFlags;
    /// let flags = LoadModelFlags::URDF_ENABLE_CACHED_GRAPHICS_SHAPES | LoadModelFlags::URDF_PRINT_URDF_INFO;
    /// assert!(flags.contains(LoadModelFlags::URDF_PRINT_URDF_INFO));
    /// ```
    pub struct LoadModelFlags : i32 {
        /// use no flags (Default)
         const NONE = 0;
        /// Use the inertia tensor provided in the URDF.
        ///
        /// By default, Bullet will recompute the inertial tensor based on the mass and volume of the
        /// collision shape. Use this is you can provide a more accurate inertia tensor.
        const URDF_USE_INERTIA_FROM_FILE = 2;
        /// Enables self-collision.
        const URDF_USE_SELF_COLLISION = 8;
        const URDF_USE_SELF_COLLISION_EXCLUDE_PARENT = 16;
        /// will discard self-collisions between a child link and any of its ancestors
        /// (parents, parents of parents, up to the base).
        /// Needs to be used together with [`URDF_USE_SELF_COLLISION`](`Self::URDF_USE_SELF_COLLISION`).
        const URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS = 32;
        const URDF_RESERVED = 64;
        /// will use a smooth implicit cylinder. By default, Bullet will tesselate the cylinder
        /// into a convex hull.
        const URDF_USE_IMPLICIT_CYLINDER = 128;
        const URDF_GLOBAL_VELOCITIES_MB = 256;
        const MJCF_COLORS_FROM_FILE = 512;
        /// Caches as reuses graphics shapes. This will decrease loading times for similar objects
        const URDF_ENABLE_CACHED_GRAPHICS_SHAPES = 1024;
        /// Allow the disabling of simulation after a body hasn't moved for a while.
        ///
        /// Interaction with active bodies will re-enable simulation.
        const URDF_ENABLE_SLEEPING = 2048;
        /// will create triangle meshes for convex shapes. This will improve visualization and also
        /// allow usage of the separating axis test (SAT) instead of GJK/EPA.
        /// Requires to enable_SAT using set_physics_engine_parameter. TODO
        const URDF_INITIALIZE_SAT_FEATURES = 4096;
        /// will enable collision between child and parent, it is disabled by default.
        /// Needs to be used together with [`URDF_USE_SELF_COLLISION`](`Self::URDF_USE_SELF_COLLISION`) flag.
        const URDF_USE_SELF_COLLISION_INCLUDE_PARENT = 8192;
        const URDF_PARSE_SENSORS = 16384;
        /// will use the RGB color from the Wavefront OBJ file, instead of from the URDF file.
        const URDF_USE_MATERIAL_COLORS_FROM_MTL = 32768;
        const URDF_USE_MATERIAL_TRANSPARANCY_FROM_MTL = 65536;
        /// Try to maintain the link order from the URDF file.
        const URDF_MAINTAIN_LINK_ORDER = 131072;
        const URDF_ENABLE_WAKEUP = 262144;
        /// this will remove fixed links from the URDF file and merge the resulting links.
        /// This is good for performance, since various algorithms
        /// (articulated body algorithm, forward kinematics etc) have linear complexity
        /// in the number of joints, including fixed joints.
        const URDF_MERGE_FIXED_LINKS = 1 << 19;
        const URDF_IGNORE_VISUAL_SHAPES = 1 << 20;
        const URDF_IGNORE_COLLISION_SHAPES = 1 << 21;
        const URDF_PRINT_URDF_INFO = 1 << 22;
        const URDF_GOOGLEY_UNDEFINED_COLORS = 1 << 23;
    }
}

impl Default for LoadModelFlags {
    fn default() -> Self {
        LoadModelFlags::NONE
    }
}
bitflags::bitflags! {
    #[doc(hidden)]
    pub struct JointInfoFlags : i32 {
        const NONE = 0;
        const JOINT_CHANGE_MAX_FORCE = 1;
        const JOINT_CHANGE_CHILD_FRAME_POSITION = 2;
        const JOINT_CHANGE_CHILD_FRAME_ORIENTATION = 4;
    }
}

impl Default for JointInfoFlags {
    fn default() -> Self {
        JointInfoFlags::NONE
    }
}

/// contains the parameters for [`change_constraint`](`crate::PhysicsClient::change_constraint`) method.
#[derive(Default)]
pub struct ChangeConstraintOptions {
    /// updated child pivot, see [`create_constraint`](`crate::PhysicsClient::create_constraint`)
    pub joint_child_pivot: Option<Vector3<f64>>,
    /// updated child frame orientation as quaternion
    pub joint_child_frame_orientation: Option<UnitQuaternion<f64>>,
    /// maximum force that constraint can apply
    pub max_force: Option<f64>,
    /// the ratio between the rates at which the two gears rotate
    pub gear_ratio: Option<f64>,
    /// In some cases, such as a differential drive, a third (auxiliary) link is used as reference pose.
    pub gear_aux_link: Option<usize>,
    /// the relative position target offset between two gears
    pub relative_position_target: Option<f64>,
    /// constraint error reduction parameter
    pub erp: Option<f64>,
}

/// contains the parameters for [`change_constraint`](`crate::PhysicsClient::change_constraint`) method.
#[derive(Debug)]
pub struct ConstraintInfo {
    /// the constraint for which this info is generated
    pub id: ConstraintId,
    /// parent body unique id
    pub parent_body: BodyId,
    /// parent body link index or `None` for base link.
    pub parent_link_index: Option<usize>,
    /// child body unique id or `None`or no body (specify a non-dynamic child frame in world coordinates)
    pub child_body: Option<BodyId>,
    /// child body link index or `None` for base link.
    pub child_link_index: Option<usize>,
    /// The [`JointType`](`crate::types::JointType`) for the constraint
    pub constraint_type: JointType,
    /// joint axis, in child link frame
    pub joint_axis: Vector3<f64>,
    /// pose of the joint frame relative to parent center of mass frame.
    pub joint_parent_frame_pose: Isometry3<f64>,
    /// updated child pose, see [`create_constraint`](`crate::PhysicsClient::create_constraint`)
    pub joint_child_frame_pose: Isometry3<f64>,
    /// maximum force that constraint can apply
    pub max_applied_force: f64,
    /// the ratio between the rates at which the two gears rotate
    pub gear_ratio: f64,
    /// In some cases, such as a differential drive, a third (auxiliary) link is used as reference pose.
    pub gear_aux_link: Option<usize>,
    /// the relative position target offset between two gears
    pub relative_position_target: f64,
    /// constraint error reduction parameter
    pub erp: f64,
}
impl From<b3UserConstraint> for ConstraintInfo {
    fn from(b3: b3UserConstraint) -> Self {
        #[allow(non_snake_case)]
        let b3UserConstraint {
            m_parentBodyIndex,
            m_parentJointIndex,
            m_childBodyIndex,
            m_childJointIndex,
            m_parentFrame,
            m_childFrame,
            m_jointAxis,
            m_jointType,
            m_maxAppliedForce,
            m_userConstraintUniqueId,
            m_gearRatio,
            m_gearAuxLink,
            m_relativePositionTarget,
            m_erp,
        } = b3;
        let parent_joint_index = {
            if m_parentJointIndex >= 0 {
                Some(m_parentJointIndex as usize)
            } else {
                None
            }
        };
        let child_link_index = {
            if m_childJointIndex >= 0 {
                Some(m_childJointIndex as usize)
            } else {
                None
            }
        };
        let gear_aux_link = {
            if m_gearAuxLink >= 0 {
                Some(m_gearAuxLink as usize)
            } else {
                None
            }
        };
        let child_body = {
            if m_childBodyIndex >= 0 {
                Some(BodyId(m_childBodyIndex))
            } else {
                None
            }
        };
        ConstraintInfo {
            id: ConstraintId(m_userConstraintUniqueId),
            parent_body: BodyId(m_parentBodyIndex),
            parent_link_index: parent_joint_index,
            child_body,
            child_link_index,
            constraint_type: JointType::try_from(m_jointType).unwrap(),
            joint_axis: m_jointAxis.into(),
            joint_parent_frame_pose: combined_position_orientation_array_to_isometry(m_parentFrame),
            joint_child_frame_pose: combined_position_orientation_array_to_isometry(m_childFrame),
            max_applied_force: m_maxAppliedForce,
            gear_ratio: m_gearRatio,
            gear_aux_link,
            relative_position_target: m_relativePositionTarget,
            erp: m_erp,
        }
    }
}
bitflags::bitflags! {
    pub struct ActivationState : i32 {
        const ENABLE_SLEEPING = 1;
        const DISABLE_SLEEPING = 2;
        const WAKE_UP = 4;
        const SLEEP = 8;
        const ENABLE_WAKEUP = 16;
        const DISABLE_WAKEUP = 32;
    }
}
/// Dynamics options for the [`change_dynamics`](`crate::PhysicsClient::`change_dynamics`) method.
/// Some options do not depend on the given link and apply to the whole body. These options are:
///
/// * `linear_damping`
/// * `angular_damping`
/// * `activation_state`
/// * `max_joint_velocity` - PyBullet claims that you can set it per joint, but that is not true
/// * `collision_margin`
#[derive(Default, Debug, Clone)]
pub struct ChangeDynamicsOptions {
    /// change the mass of the link
    pub mass: Option<f64>,
    /// lateral (linear) contact friction
    pub lateral_friction: Option<f64>,
    /// torsional friction around the contact normal
    pub spinning_friction: Option<f64>,
    /// torsional friction orthogonal to contact normal (keep this value very close to zero,
    /// otherwise the simulation can become very unrealistic
    pub rolling_friction: Option<f64>,
    /// bouncyness of contact. Keep it a bit less than 1, preferably closer to 0.
    pub restitution: Option<f64>,
    /// linear damping of the link (0.04 by default)
    pub linear_damping: Option<f64>,
    /// angular damping of the link (0.04 by default)
    pub angular_damping: Option<f64>,
    /// The contact stiffness and contact damping of the link encoded as tuple (contact_stiffness, contact_damping)
    /// This overrides the value if it was specified in the URDF file in the contact section.
    pub contact_stiffness_and_damping: Option<(f64, f64)>,
    /// enable or disable a friction anchor: friction drift correction
    /// (disabled by default, unless set in the URDF contact section)
    pub friction_anchor: Option<bool>,
    /// diagonal elements of the inertia tensor. Note that the base and links are centered around
    /// the center of mass and aligned with the principal axes of inertia
    /// so there are no off-diagonal elements in the inertia tensor.
    pub local_inertia_diagonal: Option<Vector3<f64>>,
    /// radius of the sphere to perform continuous collision detection.
    pub ccd_swept_sphere_radius: Option<f64>,
    /// contacts with a distance below this threshold will be processed by the constraint solver.
    /// For example, if 0, then contacts with distance 0.01 will not be processed as a constraint
    pub contact_processing_threshold: Option<f64>,
    /// When sleeping is enabled, objects that don't move (below a threshold) will be disabled
    /// as sleeping, if all other objects that influence it are also ready to sleep.
    pub activation_state: Option<ActivationState>,
    /// Joint damping coefficient applied at each joint. This coefficient is read from URDF joint damping field.
    /// Keep the value close to 0.
    /// Joint damping force = -damping_coefficient * joint_velocity
    pub joint_damping: Option<f64>,
    /// coefficient to allow scaling of friction in different directions.
    pub anisotropic_friction: Option<f64>,
    /// maximum joint velocity for the whole robot, if it is exceeded during constraint solving,
    /// it is clamped. Default maximum joint velocity is 100 units.
    pub max_joint_velocity: Option<f64>,
    /// change the collision margin. dependent on the shape type, it may or may not add some padding to the collision shape.
    pub collision_margin: Option<f64>,
    /// changes the lower and upper limits of a joint. (lower_limit, upper_limit)
    ///
    /// NOTE that at the moment, the joint limits are not updated in [`get_joint_info`](`crate::PhysicsClient::get_joint_info`)!
    pub joint_limits: Option<(f64, f64)>,
    /// change the maximum force applied to satisfy a joint limit.
    pub joint_limit_force: Option<f64>,
}

/// Contains information about the mass, center of mass, friction and other properties of the base and links.
/// Is returned by [`get_dynamics_info`](`crate::PhysicsClient::get_dynamics_info`).
#[derive(Debug)]
pub struct DynamicsInfo {
    /// mass in kg
    pub mass: f64,
    /// lateral (linear) contact friction
    pub lateral_friction: f64,
    /// spinning friction coefficient around contact normal
    pub spinning_friction: f64,
    /// rolling friction coefficient orthogonal to contact normal
    pub rolling_friction: f64,
    /// coefficient of restitution (bouncyness of contact).
    pub restitution: f64,

    /// The contact stiffness and contact damping of the link encoded as tuple (contact_stiffness, contact_damping).
    /// Is `None` if not available
    pub contact_stiffness_and_damping: Option<(f64, f64)>,

    /// diagonal elements of the inertia tensor. Note that the base and links are centered around
    /// the center of mass and aligned with the principal axes of inertia
    /// so there are no off-diagonal elements in the inertia tensor.
    pub local_inertia_diagonal: Vector3<f64>,
    ///  of inertial frame in local coordinates of the joint frame
    pub local_inertial_pose: Isometry3<f64>,
    /// body type of the object
    pub body_type: BodyType,
    ///  collision margin of the collision shape. collision margins depend on the shape type, it is not consistent.
    pub collision_margin: f64,
}
#[derive(Debug, PartialOrd, PartialEq)]
pub enum BodyType {
    RigidBody = 1,
    MultiBody = 2,
    SoftBody = 3,
}

impl From<b3DynamicsInfo> for DynamicsInfo {
    fn from(b3: b3DynamicsInfo) -> Self {
        #[allow(unused, non_snake_case)]
        let b3DynamicsInfo {
            m_mass,
            m_localInertialDiagonal,
            m_localInertialFrame,
            m_lateralFrictionCoeff,
            m_rollingFrictionCoeff,
            m_spinningFrictionCoeff,
            m_restitution,
            m_contactStiffness,
            m_contactDamping,
            m_activationState,
            m_bodyType,
            m_angularDamping,
            m_linearDamping,
            m_ccdSweptSphereRadius,
            m_contactProcessingThreshold,
            m_frictionAnchor,
            m_collisionMargin,
            m_dynamicType,
        } = b3;
        let contact_stiffness_and_damping = {
            if m_contactStiffness <= 0. || m_contactDamping <= 0. {
                None
            } else {
                Some((m_contactStiffness, m_contactDamping))
            }
        };
        DynamicsInfo {
            mass: m_mass,
            lateral_friction: m_lateralFrictionCoeff,
            spinning_friction: m_spinningFrictionCoeff,
            rolling_friction: m_rollingFrictionCoeff,
            restitution: m_restitution,
            contact_stiffness_and_damping,
            local_inertia_diagonal: m_localInertialDiagonal.into(),
            local_inertial_pose: combined_position_orientation_array_to_isometry(
                m_localInertialFrame,
            ),
            body_type: match m_bodyType {
                1 => BodyType::RigidBody,
                2 => BodyType::MultiBody,
                3 => BodyType::SoftBody,
                _ => panic!("internal error: Unknown BodyType ({})", m_bodyType),
            },
            collision_margin: m_collisionMargin,
        }
    }
}
/// axis-aligned minimum bounding box
#[derive(Debug)]
pub struct Aabb {
    /// minimum coordinates of the aabb
    pub min: Vector3<f64>,
    /// maximum coordinates of the aabb
    pub max: Vector3<f64>,
}

/// Is the result of [`get_overlapping_objects`](`crate::PhysicsClient::get_overlapping_objects`).
/// Each object specifies a link of a body.
#[derive(Debug, Copy, Clone)]
pub struct OverlappingObject {
    /// BodyID of the overlapping object
    pub body: BodyId,
    /// the index of the link which is overlapping. Is `None` for the base.
    pub link_index: Option<usize>,
}

/// Is the result of the get_closest_points and get_contact_points methods.
#[derive(Debug, Copy, Clone)]
pub struct ContactPoint {
    /// reserved
    #[doc(hidden)]
    pub contact_flag: i32,
    /// body unique id of body A. Is `None` When a collision shape was used instead
    pub body_a: Option<BodyId>,
    /// body unique id of body B. Is `None` When a collision shape was used instead
    pub body_b: Option<BodyId>,
    /// link index of body A, `None` for base
    pub link_index_a: Option<usize>,
    /// link index of body A, `None` for base
    pub link_index_b: Option<usize>,
    /// contact position on A, in Cartesian world coordinates
    pub position_on_a: Vector3<f64>,
    /// contact position on B, in Cartesian world coordinates
    pub position_on_b: Vector3<f64>,
    /// contact normal on B, pointing towards A
    pub contact_normal_on_b: Vector3<f64>,
    /// contact distance, positive for separation, negative for penetration
    pub contact_distance: f64,
    /// normal force applied during the last 'stepSimulation'. Is `None` when used with one of the
    /// get_closes_points methods
    pub normal_force: Option<f64>,
    /// first lateral friction
    pub lateral_friction_1: Vector3<f64>,
    /// second lateral friction
    pub lateral_friction_2: Vector3<f64>,
}

impl From<b3ContactPointData> for ContactPoint {
    fn from(b3: b3ContactPointData) -> Self {
        #[allow(non_snake_case)]
        let b3ContactPointData {
            m_contactFlags,
            m_bodyUniqueIdA,
            m_bodyUniqueIdB,
            m_linkIndexA,
            m_linkIndexB,
            m_positionOnAInWS,
            m_positionOnBInWS,
            m_contactNormalOnBInWS,
            m_contactDistance,
            m_normalForce,
            m_linearFrictionForce1,
            m_linearFrictionForce2,
            m_linearFrictionDirection1,
            m_linearFrictionDirection2,
        } = b3;
        let mut lateral_friction_1: Vector3<f64> = m_linearFrictionDirection1.into();
        lateral_friction_1 *= m_linearFrictionForce1;
        let mut lateral_friction_2: Vector3<f64> = m_linearFrictionDirection2.into();
        lateral_friction_2 *= m_linearFrictionForce2;
        let link_index_a = {
            if m_linkIndexA.is_negative() {
                None
            } else {
                Some(m_linkIndexA as usize)
            }
        };
        let link_index_b = {
            if m_linkIndexB.is_negative() {
                None
            } else {
                Some(m_linkIndexB as usize)
            }
        };
        let body_a = {
            if m_bodyUniqueIdA < 0 {
                None
            } else {
                Some(BodyId(m_bodyUniqueIdA))
            }
        };
        let body_b = {
            if m_bodyUniqueIdB < 0 {
                None
            } else {
                Some(BodyId(m_bodyUniqueIdB))
            }
        };
        ContactPoint {
            contact_flag: m_contactFlags,
            body_a,
            body_b,
            link_index_a,
            link_index_b,
            position_on_a: m_positionOnAInWS.into(),
            position_on_b: m_positionOnBInWS.into(),
            contact_normal_on_b: m_contactNormalOnBInWS.into(),
            contact_distance: m_contactDistance,
            normal_force: Some(m_normalForce),
            lateral_friction_1,
            lateral_friction_2,
        }
    }
}
