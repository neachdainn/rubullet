//! Custom data types for RuBullet

use crate::Error;
use nalgebra::{DMatrix, Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use rubullet_ffi::{b3JointInfo, b3JointSensorState, b3LinkState};
use std::convert::TryFrom;
use std::ffi::CStr;
use std::os::raw::c_int;

/// The unique ID for a body within a physics server.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct BodyId(pub(crate) c_int);

/// The unique ID for a Visual Shape
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct VisualId(pub c_int);

/// The unique ID for a Collision Shape
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct CollisionId(pub c_int);

/// The unique ID for a Texture
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct TextureId(pub(crate) c_int);

#[derive(Debug, PartialEq, Copy, Clone)]
/// An enum to represent different types of joints
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
#[derive(Debug)]
/// Contains basic information about a joint like its type and name.
/// # Example
/// ```rust
/// use rubullet::{PhysicsClient, UrdfOptions};
/// use nalgebra::Isometry3;
/// use rubullet::mode::Mode::Direct;
/// use easy_error::Terminator;
/// fn main() -> Result<(),Terminator> {
///
///     let mut client = PhysicsClient::connect(Direct)?;
///     client.set_additional_search_path("../rubullet-ffi/bullet3/libbullet3/data")?;
///     client.set_additional_search_path(
///         "../rubullet-ffi/bullet3/libbullet3/examples/pybullet/gym/pybullet_data",
///         )?;
///     let panda_id = client.load_urdf("franka_panda/panda.urdf", UrdfOptions::default())?;
///     let joint_info = client.get_joint_info(panda_id,4);
///     assert_eq!("panda_joint5",joint_info.joint_name);
///     Ok(())
/// }
/// ```
/// # See also
/// * [`JointState`](`crate::types::JointState`) - For information about the current state of the joint.
pub struct JointInfo {
    /// the same joint index as the input parameter
    pub joint_index: i32,
    /// the name of the joint, as specified in the URDF (or SDF etc) file
    pub joint_name: String,
    /// type of the joint, this also implies the number of position and velocity variables.
    pub joint_type: JointType,
    /// the first position index in the positional state variables for this body
    pub q_index: i32,
    /// the first velocity index in the velocity state variables for this body
    pub u_index: i32,
    /// reserved
    pub flags: i32,
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
    /// parent link index, -1 for base
    pub parent_index: i32,
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
                joint_index: m_joint_index,
                flags: m_flags,
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
                parent_index: m_parent_index,
            }
        }
    }
}

pub struct InverseKinematicsNullSpaceParameters<'a> {
    pub lower_limits: &'a [f64],
    pub upper_limits: &'a [f64],
    pub joint_ranges: &'a [f64],
    pub rest_poses: &'a [f64],
}

pub struct InverseKinematicsParameters<'a> {
    pub body: BodyId,
    pub end_effector_link_index: i32,
    pub target_position: [f64; 3],
    pub target_orientation: Option<[f64; 4]>,
    pub limits: Option<InverseKinematicsNullSpaceParameters<'a>>,
    pub joint_damping: Option<&'a [f64]>,
    pub solver: IKSolver,
    pub current_position: Option<&'a [f64]>,
    pub max_num_iterations: i32,
    pub residual_threshold: Option<f64>,
}

pub enum IKSolver {
    DLS = 0,
    SDLS = 1,
}

impl From<IKSolver> for i32 {
    fn from(solver: IKSolver) -> Self {
        solver as i32
    }
}

impl<'a> Default for InverseKinematicsParameters<'a> {
    fn default() -> Self {
        InverseKinematicsParameters {
            body: BodyId(0),
            end_effector_link_index: 0,
            target_position: [0., 0., 0.],
            target_orientation: None,
            limits: None,
            joint_damping: None,
            solver: IKSolver::DLS,
            current_position: None,
            max_num_iterations: 20,
            residual_threshold: None,
        }
    }
}

pub struct InverseKinematicsParametersBuilder<'a> {
    params: InverseKinematicsParameters<'a>,
}

impl<'a> InverseKinematicsParametersBuilder<'a> {
    pub fn new<Index>(
        body: BodyId,
        end_effector_link_index: Index,
        target_pose: &'a Isometry3<f64>,
    ) -> Self
    where
        Index: Into<i32>,
    {
        let target_position: [f64; 3] = target_pose.translation.vector.into();
        let quat = &target_pose.rotation.coords;
        let target_orientation = [quat.x, quat.y, quat.z, quat.w];
        let params = InverseKinematicsParameters {
            body,
            end_effector_link_index: end_effector_link_index.into(),
            target_position,
            target_orientation: Some(target_orientation),
            ..Default::default()
        };
        InverseKinematicsParametersBuilder { params }
    }
    pub fn ignore_orientation(mut self) -> Self {
        self.params.target_orientation = None;
        self
    }
    pub fn use_null_space(mut self, limits: InverseKinematicsNullSpaceParameters<'a>) -> Self {
        self.params.limits = Some(limits);
        self
    }
    pub fn set_joint_damping(mut self, joint_damping: &'a [f64]) -> Self {
        self.params.joint_damping = Some(joint_damping);
        self
    }
    pub fn set_ik_solver(mut self, solver: IKSolver) -> Self {
        self.params.solver = solver;
        self
    }
    pub fn set_current_position(mut self, current_position: &'a [f64]) -> Self {
        self.params.current_position = Some(current_position);
        self
    }
    pub fn set_max_num_iterations(mut self, iterations: u32) -> Self {
        self.params.max_num_iterations = iterations as i32;
        self
    }
    pub fn set_residual_threshold(mut self, residual_threshold: f64) -> Self {
        self.params.residual_threshold = Some(residual_threshold);
        self
    }
    pub fn build(self) -> InverseKinematicsParameters<'a> {
        self.params
    }
}

pub struct AddDebugTextOptions<'a> {
    pub text_color_rgb: &'a [f64],
    pub text_size: f64,
    pub life_time: f64,
    pub text_orientation: Option<&'a [f64]>,
    pub parent_object_id: Option<BodyId>,
    pub parent_link_index: Option<i32>,
    pub replace_item_id: Option<BodyId>,
}

impl<'a> Default for AddDebugTextOptions<'a> {
    fn default() -> Self {
        AddDebugTextOptions {
            text_color_rgb: &[1.; 3],
            text_size: 1.,
            life_time: 0.,
            text_orientation: None,
            parent_object_id: None,
            parent_link_index: None,
            replace_item_id: None,
        }
    }
}

pub struct AddDebugLineOptions<'a> {
    pub line_color_rgb: &'a [f64],
    pub line_width: f64,
    pub life_time: f64,
    pub parent_object_id: Option<BodyId>,
    pub parent_link_index: Option<i32>,
    pub replace_item_id: Option<BodyId>,
}

impl<'a> Default for AddDebugLineOptions<'a> {
    fn default() -> Self {
        AddDebugLineOptions {
            line_color_rgb: &[1.; 3],
            line_width: 1.,
            life_time: 0.,
            parent_object_id: None,
            parent_link_index: None,
            replace_item_id: None,
        }
    }
}

#[derive(Debug)]
pub struct Jacobian {
    pub linear_jacobian: DMatrix<f64>,
    pub angular_jacobian: DMatrix<f64>,
}

pub enum ExternalForceFrame {
    LinkFrame = 1,
    WorldFrame = 2,
}

#[derive(Debug, Copy, Clone, Default)]
pub struct KeyboardEvent {
    pub key: char,
    pub(crate) key_state: i32,
}

impl KeyboardEvent {
    pub fn was_triggered(&self) -> bool {
        self.key_state & 2 == 2
    }
    pub fn is_down(&self) -> bool {
        self.key_state & 1 == 1
    }
    pub fn is_released(&self) -> bool {
        self.key_state & 4 == 4
    }
}

#[derive(Debug, Copy, Clone)]
pub enum MouseEvent {
    Move {
        mouse_pos_x: f32,
        mouse_pos_y: f32,
    },
    Button {
        mouse_pos_x: f32,
        mouse_pos_y: f32,
        button_index: i32,
        button_state_flag: i32,
    },
}

impl MouseEvent {
    pub fn was_triggered(flag: i32) -> bool {
        flag & 2 == 2
    }
    pub fn is_down(flag: i32) -> bool {
        flag & 1 == 1
    }
    pub fn is_released(flag: i32) -> bool {
        flag & 4 == 4
    }
}

#[derive(Debug, Default, Copy, Clone)]
pub struct JointState {
    pub joint_position: f64,
    pub joint_velocity: f64,
    /// note to roboticists: this is NOT the motor torque/force, but the spatial reaction force vector at joint.
    pub joint_force_torque: [f64; 6],
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

    /// Use the inertia tensor provided in the URDF.
    ///
    /// By default, Bullet will recompute the inertial tensor based on the mass and volume of the
    /// collision shape. Use this is you can provide a more accurate inertia tensor.
    pub use_inertia_from_file: bool,

    /// Enables or disables self-collision.
    pub use_self_collision: bool,

    /// Allow the disabling of simulation after a body hasn't moved for a while.
    ///
    /// Interaction with active bodies will re-enable simulation.
    pub enable_sleeping: bool,

    /// Try to maintain the link order from the URDF file.
    pub maintain_link_order: bool,

    /// Caches as reuses graphics shapes. This will decrease loading times for similar objects
    pub enable_cached_graphics_shapes: bool,

    /// Applies a scale factor to the model.
    pub global_scaling: f64,

    // Future proofs the struct. Unfortunately, `#[non_exhaustive]` doesn't apply to structs.
    #[doc(hidden)]
    pub _unused: (),
}

impl Default for UrdfOptions {
    fn default() -> UrdfOptions {
        UrdfOptions {
            base_transform: Isometry3::identity(),
            use_fixed_base: false,
            use_inertia_from_file: false,
            use_self_collision: false,
            enable_sleeping: false,
            maintain_link_order: false,
            global_scaling: -1.0,
            enable_cached_graphics_shapes: false,
            _unused: (),
        }
    }
}

pub enum ControlMode {
    Position(f64),
    PositionWithPD {
        target_position: f64,
        target_velocity: f64,
        position_gain: f64,
        velocity_gain: f64,
        maximum_velocity: Option<f64>,
    },
    Velocity(f64),
    Torque(f64),
    PD {
        target_position: f64,
        target_velocity: f64,
        position_gain: f64,
        velocity_gain: f64,
        maximum_velocity: Option<f64>,
    },
}

impl ControlMode {
    pub(crate) fn get_int(&self) -> i32 {
        match self {
            ControlMode::Position(_) => 2,
            ControlMode::Velocity(_) => 0,
            ControlMode::Torque(_) => 1,
            ControlMode::PD { .. } => 3,
            ControlMode::PositionWithPD { .. } => 2,
        }
    }
}

pub enum ControlModeArray<'a> {
    Positions(&'a [f64]),
    PositionsWithPD {
        target_positions: &'a [f64],
        target_velocities: &'a [f64],
        position_gains: &'a [f64],
        velocity_gains: &'a [f64],
    },
    Velocities(&'a [f64]),
    Torques(&'a [f64]),
    PD {
        target_positions: &'a [f64],
        target_velocities: &'a [f64],
        position_gains: &'a [f64],
        velocity_gains: &'a [f64],
    },
}

impl ControlModeArray<'_> {
    pub(crate) fn get_int(&self) -> i32 {
        match self {
            ControlModeArray::Positions(_) => 2,
            ControlModeArray::Velocities(_) => 0,
            ControlModeArray::Torques(_) => 1,
            ControlModeArray::PD { .. } => 3,
            ControlModeArray::PositionsWithPD { .. } => 2,
        }
    }
}

pub struct JointMotorControlOptions {
    pub body_unique_id: BodyId,
    pub joint_index: i32,
    pub control_mode: ControlMode,
    pub target_position: Option<f64>,
    pub target_velocity: Option<f64>,
    pub force: Option<f64>,
    pub position_gain: Option<f64>,
    pub velocity_gain: Option<f64>,
    pub max_velocity: Option<f64>,
}

impl JointMotorControlOptions {
    fn new(body_unique_id: BodyId, joint_index: i32, control_mode: ControlMode) -> Self {
        JointMotorControlOptions {
            body_unique_id,
            joint_index,
            control_mode,
            target_position: None,
            target_velocity: None,
            force: None,
            position_gain: None,
            velocity_gain: None,
            max_velocity: None,
        }
    }
}

#[allow(non_camel_case_types)]
pub enum DebugVisualizerFlag {
    COV_ENABLE_GUI = 1,
    COV_ENABLE_SHADOWS,
    COV_ENABLE_WIREFRAME,
    COV_ENABLE_VR_TELEPORTING,
    COV_ENABLE_VR_PICKING,
    COV_ENABLE_VR_RENDER_CONTROLLERS,
    COV_ENABLE_RENDERING,
    COV_ENABLE_SYNC_RENDERING_INTERNAL,
    COV_ENABLE_KEYBOARD_SHORTCUTS,
    COV_ENABLE_MOUSE_PICKING,
    COV_ENABLE_Y_AXIS_UP,
    COV_ENABLE_TINY_RENDERER,
    COV_ENABLE_RGB_BUFFER_PREVIEW,
    COV_ENABLE_DEPTH_BUFFER_PREVIEW,
    COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
    COV_ENABLE_PLANAR_REFLECTION,
    COV_ENABLE_SINGLE_STEP_RENDERING,
}

#[derive(Debug)]
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
/// use rubullet::mode::Mode::Direct;
/// use easy_error::Terminator;
/// fn main() -> Result<(),Terminator> {
///     let mut client = PhysicsClient::connect(Direct)?;
///     client.set_additional_search_path("../rubullet-ffi/bullet3/libbullet3/data")?;
///     client.set_additional_search_path(
///         "../rubullet-ffi/bullet3/libbullet3/examples/pybullet/gym/pybullet_data",
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
pub struct LinkState {
    /// Cartesian pose of the center of mass
    pub world_pose: Isometry3<f64>,
    /// local offset of the intertial frame (center of mass) express in the URDF link frame
    pub local_inertial_pose: Isometry3<f64>,
    /// world pose of the URDF link frame
    pub world_link_frame_pose: Isometry3<f64>,
    ///Cartesian world linear  velocity. Only valid when ACTUAL_STATE_COMPUTE_LINKVELOCITY is set (b3RequestActualStateCommandComputeLinkVelocity)
    pub world_linear_velocity: [f64; 3],
    ///Cartesian world angular velocity. Only valid when ACTUAL_STATE_COMPUTE_LINKVELOCITY is set (b3RequestActualStateCommandComputeLinkVelocity)
    pub world_angular_velocity: [f64; 3],
}
impl From<b3LinkState> for LinkState {
    fn from(b3: b3LinkState) -> Self {
        let b3LinkState {
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
        } = b3;
        LinkState {
            world_pose: position_orientation_to_isometry(m_world_position, m_world_orientation),
            local_inertial_pose: position_orientation_to_isometry(
                m_local_inertial_position,
                m_local_inertial_orientation,
            ),
            world_link_frame_pose: position_orientation_to_isometry(
                m_world_link_frame_position,
                m_world_link_frame_orientation,
            ),
            world_linear_velocity: m_world_linear_velocity,
            world_angular_velocity: m_world_angular_velocity,
        }
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

/// VisualShape options are for the [create_visual_shape](`crate::PhysicsClient::create_visual_shape`)
/// function to specify additional options like the color.
pub struct VisualShapeOptions {
    /// offset of the shape with respect to the link frame
    pub frame_offset: Isometry3<f64>,
    /// color components for red, green, blue and alpha, each in range [0,1]
    pub rgba_colors: [f64; 4],
    /// specular reflection color, red, green, blue components in range [0,1]
    pub specular_colors: [f64; 3],
    /// Additional flags. Currently not used
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

pub enum GeometricCollisionShape {
    /// A Sphere determined by the radius in meter
    Sphere {
        /// radius in meter
        radius: f64,
    },
    /// A Cuboid
    Box {
        /// [x,y,z] lengths starting from the middle of the box.
        /// For example [0.5,0.5,0.5] is a unit cube.
        half_extents: [f64; 3],
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
        plane_normal: [f64; 3],
    },
    /// Load a .obj (Wavefront) file. Will create convex hulls for each object.
    MeshFile {
        /// Path to the .obj file as String.
        filename: String,
        /// Scaling of the Mesh. Use [1.;3] for original scaling.
        mesh_scale: [f64; 3],
        /// Set to 1 if you want to activate have the GEOM_FORCE_CONCAVE_TRIMESH Flag.
        /// this will create a concave static triangle mesh. This should not be used with
        /// dynamic / moving objects, only for static (mass = 0) terrain.
        flags: Option<i32>,
    },
    /// Create your own mesh.
    Mesh {
        /// list of [x,y,z] coordinates.
        vertices: Vec<[f64; 3]>,
        /// triangle indices, should be a multiple of 3
        indices: Option<Vec<i32>>,
        /// Scaling of the Mesh. Use [1.;3] for normal scaling.
        mesh_scale: [f64; 3],
    },
    /// Loads a Heightfield from a file
    HeigthfieldFile {
        /// Path to the .obj file as String.
        filename: String,
        /// Scaling of the Mesh. Use [1.;3] for original scaling.
        mesh_scale: [f64; 3],
        /// Texture scaling. Use 1. for original scaling.
        texture_scaling: f64,
    },
    /// Create your own Heightfiled. See heightfield.rs for an example.
    Heigthfield {
        /// Scaling of the Mesh. Use [1.;3] for normal scaling.
        mesh_scale: [f64; 3],
        /// Texture scaling. Use 1. for normal scaling.
        texture_scaling: f64,
        /// Heigthfield data. Should be of size num_rows * num_columns
        data: Vec<f32>,
        /// number of rows in data
        num_rows: i32,
        /// number of columns in data
        num_columns: i32,
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
        /// [x,y,z] lengths starting from the middle of the box.
        /// For example [0.5,0.5,0.5] is a unit cube.
        half_extents: [f64; 3],
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
        plane_normal: [f64; 3],
    },
    /// Loads a .obj (Wavefront) file. Will create convex hulls for each object.
    MeshFile {
        /// Path to the .obj file as String
        filename: String,
        /// Scaling of the Mesh. Use [1.;3] for original scaling.
        mesh_scale: [f64; 3],
    },
    /// Create your own mesh.
    Mesh {
        /// Scaling of the Mesh. Use [1.;3] for normal scaling.
        mesh_scale: [f64; 3],
        /// list of [x,y,z] coordinates.
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
    pub link_collision_shapes: Vec<CollisionId>,
    /// List of the visual shape unique id, one for each link.
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
    pub link_joint_axis: Vec<[f64; 3]>,
    /// experimental, best to leave it false.
    pub use_maximal_coordinates: bool,
    /// similar to the flags passed in load_urdf, for example URDF_USE_SELF_COLLISION.
    /// See [load_urdf](`crate::PhysicsClient::load_urdf()`) for flags explanation.
    pub flags: Option<i32>,
    /// array of base positions, for fast batch creation of many multibodies.
    /// See create_multi_body_batch.rs example.
    pub batch_positions: Option<Vec<[f64; 3]>>,
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
