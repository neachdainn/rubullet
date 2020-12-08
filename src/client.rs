//! The main physics client.
//!
//! This is largely modeled after the PyBullet API but utilizes Rust's more expressive type system
//! where available.
use std::convert::TryFrom;
use std::{ffi::CString, os::raw::c_int, path::Path, ptr};
// I currently do not know the best way to represent the file operations for Windows. PyBullet uses
// raw C-strings but that doesn't seem appropriate here. I don't really have a Windows machine, so
// until then...
use std::os::unix::ffi::OsStrExt;

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};

use crate::ffi::b3JointInfo;
use crate::{ffi, Error, Mode};

use self::gui_marker::GuiMarker;
use std::time::Duration;

/// The "handle" to the physics client.
///
/// For whatever reason, the Bullet C API obfuscates the handle type by defining the handle type as
/// a pointer to an (essentially) anonymous struct. That's ugly to use here, and we know it isn't
/// going to be null, so we'll just do this alias.
type Handle = std::ptr::NonNull<ffi::b3PhysicsClientHandle__>;

/// Connection to a physics server.
///
/// This serves as an abstraction over the possible physics servers, providing a unified interface.
pub struct PhysicsClient {
    /// The underlying `b3PhysicsClientHandle` that is guaranteed to not be null.
    handle: Handle,

    /// A marker indicating whether or not a GUI is in use by this client.
    gui_marker: Option<GuiMarker>,
}

impl PhysicsClient {
    pub fn connect(mode: Mode) -> Result<PhysicsClient, Error> {
        let (raw_handle, gui_marker) = match mode {
            Mode::Direct => unsafe { (ffi::b3ConnectPhysicsDirect(), None) },
            Mode::Gui => {
                // Only one GUI is allowed per process. Try to get the marker and fail if there is
                // another.
                let gui_marker = GuiMarker::acquire()?;

                // Looking at the source code for both of these functions, they do not assume
                // anything about the size of `argv` beyond what is supplied by `argc`. So, and
                // `argc` of zero should keep this safe.
                let raw_handle = if cfg!(target_os = "macos") {
                    unsafe {
                        ffi::b3CreateInProcessPhysicsServerAndConnectMainThread(0, ptr::null_mut())
                    }
                } else {
                    unsafe { ffi::b3CreateInProcessPhysicsServerAndConnect(0, ptr::null_mut()) }
                };

                (raw_handle, Some(gui_marker))
            }
        };

        // Make sure the returned pointer is valid.
        let handle =
            Handle::new(raw_handle).ok_or_else(|| Error::new("Bullet returned a null handle"))?;

        // At this point, we need to disconnect the physics client at any error. So we create the
        // Rust struct and allow the `Drop` implementation to take care of that.
        let mut client = PhysicsClient { handle, gui_marker };

        // Make sure it is up and running.
        if !client.can_submit_command() {
            return Err(Error::new("Physics server is not running"));
        }

        // Now perform a series of commands to finish starting up the server. I don't know what they
        // do but it's what PyBullet does. Note that PyBullet does not check these for `null` so I
        // am assuming that they either can't be null or the consumer does the check.
        unsafe {
            let command = ffi::b3InitSyncBodyInfoCommand(client.handle.as_ptr());
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(client.handle.as_ptr(), command);
            let status_type = ffi::b3GetStatusType(status_handle);

            if status_type != ffi::EnumSharedMemoryServerStatus::CMD_SYNC_BODY_INFO_COMPLETED as _ {
                return Err(Error::new("Connection terminated, couldn't get body info"));
            }

            let command = ffi::b3InitSyncUserDataCommand(client.handle.as_ptr());
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(client.handle.as_ptr(), command);
            let status_type = ffi::b3GetStatusType(status_handle);

            if status_type != ffi::EnumSharedMemoryServerStatus::CMD_SYNC_USER_DATA_COMPLETED as _ {
                return Err(Error::new("Connection terminated, couldn't get user data"));
            }
        }

        // The client is up and running
        Ok(client)
    }
    pub fn set_time_step(&mut self, time_step: &Duration) {
        unsafe {
            let command = ffi::b3InitPhysicsParamCommand(self.handle.as_ptr());
            let _ret = ffi::b3PhysicsParamSetTimeStep(command, time_step.as_secs_f64());
            let _status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
        }
    }
    pub fn set_real_time_simulation(&mut self, enable_real_time_simulation: bool) {
        unsafe {
            let command = ffi::b3InitPhysicsParamCommand(self.handle.as_ptr());
            let _ret = ffi::b3PhysicsParamSetRealTimeSimulation(
                command,
                enable_real_time_simulation as i32,
            );
            let _status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
        }
    }
    /// Sets an additional search path for loading assests.
    pub fn set_additional_search_path<P: AsRef<Path>>(&mut self, path: P) -> Result<(), Error> {
        if !self.can_submit_command() {
            return Err(Error::new("Not connected to physics server"));
        }

        let path = CString::new(path.as_ref().as_os_str().as_bytes())
            .map_err(|_| Error::new("Invalid path"))?;

        unsafe {
            // Based on PyBullet, it appears that this path is copied and it does not need to live
            // after calling the function.
            let command_handle =
                ffi::b3SetAdditionalSearchPath(self.handle.as_ptr(), path.as_ptr());
            let _status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
        }

        Ok(())
    }

    /// Sets the default gravity force for all objects.
    ///
    /// By default, there is no gravitational force enabled.
    pub fn set_gravity(&mut self, gravity: Vector3<f64>) -> Result<(), Error> {
        if !self.can_submit_command() {
            return Err(Error::new("Not connected to physics server"));
        }

        unsafe {
            // PyBullet error checks none of these. Looking through the code, it looks like there is
            // no possible way to return an error on them.
            let command = ffi::b3InitPhysicsParamCommand(self.handle.as_ptr());
            let _ret = ffi::b3PhysicsParamSetGravity(command, gravity.x, gravity.y, gravity.z);
            let _status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
        }

        Ok(())
    }

    /// Sends a command to the physics server to load a physics model from a Universal Robot
    /// Description File (URDF).
    pub fn load_urdf<P: AsRef<Path>>(
        &mut self,
        file: P,
        options: UrdfOptions,
    ) -> Result<BodyId, Error> {
        if !self.can_submit_command() {
            return Err(Error::new("Not connected to physics server"));
        }

        let file = CString::new(file.as_ref().as_os_str().as_bytes())
            .map_err(|_| Error::new("Invalid path"))?;

        // There's probably a cleaner way to do it. The one-liner I came up with was neat but bad
        // code.
        let mut flags = 0;
        if options.use_inertia_from_file {
            flags |= ffi::eURDF_Flags::URDF_USE_INERTIA_FROM_FILE as c_int;
        }
        if options.use_self_collision {
            flags |= ffi::eURDF_Flags::URDF_USE_SELF_COLLISION as c_int;
        }
        if options.enable_sleeping {
            flags |= ffi::eURDF_Flags::URDF_ENABLE_SLEEPING as c_int;
        }
        if options.maintain_link_order {
            flags |= ffi::eURDF_Flags::URDF_MAINTAIN_LINK_ORDER as c_int;
        }
        if options.enable_cached_graphics_shapes {
            flags |= ffi::eURDF_Flags::URDF_ENABLE_CACHED_GRAPHICS_SHAPES as c_int;
        }
        unsafe {
            // As always, PyBullet does not document and does not check return codes.
            let command = ffi::b3LoadUrdfCommandInit(self.handle.as_ptr(), file.as_ptr());
            let _ret = ffi::b3LoadUrdfCommandSetFlags(command, flags);
            let _ret = ffi::b3LoadUrdfCommandSetStartPosition(
                command,
                options.base_transform.translation.x,
                options.base_transform.translation.y,
                options.base_transform.translation.z,
            );
            let _ret = ffi::b3LoadUrdfCommandSetStartOrientation(
                command,
                options.base_transform.rotation.coords.x,
                options.base_transform.rotation.coords.y,
                options.base_transform.rotation.coords.z,
                options.base_transform.rotation.coords.w,
            );

            if options.use_fixed_base {
                let _ret = ffi::b3LoadUrdfCommandSetUseFixedBase(command, 1);
            }

            if options.global_scaling > 0.0 {
                let _ret =
                    ffi::b3LoadUrdfCommandSetGlobalScaling(command, options.global_scaling as f64);
            }

            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
            let status_type = ffi::b3GetStatusType(status_handle);
            if status_type != ffi::EnumSharedMemoryServerStatus::CMD_URDF_LOADING_COMPLETED as c_int
            {
                return Err(Error::new("Cannot load URDF file"));
            }

            Ok(BodyId(ffi::b3GetStatusBodyIndex(status_handle)))
        }
    }

    /// Performs all the actions in a single forward dynamics simulation step such as collision
    /// detection, constraint solving, and integration.
    // TODO: Mention changing step size and automatic steps.
    // TODO: Return analytics data?
    pub fn step_simulation(&mut self) -> Result<(), Error> {
        if !self.can_submit_command() {
            return Err(Error::new("Not connected to physics server"));
        }

        unsafe {
            let command = ffi::b3InitStepSimulationCommand(self.handle.as_ptr());
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
            let status_type = ffi::b3GetStatusType(status_handle);
            if status_type
                != ffi::EnumSharedMemoryServerStatus::CMD_STEP_FORWARD_SIMULATION_COMPLETED as i32
            {
                return Err(Error::new("Failed to perform forward step"));
            }
        }

        Ok(())
    }

    /// Reports the current transform of the base.
    pub fn get_base_transform(&mut self, body: BodyId) -> Result<Isometry3<f64>, Error> {
        if !self.can_submit_command() {
            return Err(Error::new("Not connected to physics server"));
        }

        unsafe {
            let cmd_handle = ffi::b3RequestActualStateCommandInit(self.handle.as_ptr(), body.0);
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), cmd_handle);
            let status_type = ffi::b3GetStatusType(status_handle);

            if status_type
                != ffi::EnumSharedMemoryServerStatus::CMD_ACTUAL_STATE_UPDATE_COMPLETED as c_int
            {
                return Err(Error::new("Failed to get base transform"));
            }

            // To be totally honest, I'm not sure this part is correct.
            let actual_state_q: *mut f64 = ptr::null_mut();
            ffi::b3GetStatusActualState(
                status_handle,
                ptr::null_mut(),
                ptr::null_mut(),
                ptr::null_mut(),
                ptr::null_mut(),
                &actual_state_q as _,
                ptr::null_mut(),
                ptr::null_mut(),
            );

            assert!(!actual_state_q.is_null());

            let tx = *actual_state_q;
            let ty = *(actual_state_q.offset(1));
            let tz = *(actual_state_q.offset(2));

            let rx = *(actual_state_q.offset(3));
            let ry = *(actual_state_q.offset(4));
            let rz = *(actual_state_q.offset(5));
            let rw = *(actual_state_q.offset(6));

            let tra = Translation3::new(tx, ty, tz);
            let rot = Quaternion::new(rw, rx, ry, rz);

            Ok(Isometry3::from_parts(
                tra,
                UnitQuaternion::from_quaternion(rot),
            ))
        }
    }

    /// Returns whether or not this client can submit a command.
    fn can_submit_command(&mut self) -> bool {
        unsafe { ffi::b3CanSubmitCommand(self.handle.as_ptr()) != 0 }
    }

    pub fn change_dynamics_linear_damping(&mut self, body: BodyId, linear_damping: f64) {
        unsafe {
            let command = ffi::b3InitChangeDynamicsInfo(self.handle.as_ptr());
            if linear_damping >= 0. {
                ffi::b3ChangeDynamicsInfoSetLinearDamping(command, body.0, linear_damping);
            }
            ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
        }
    }

    pub fn change_dynamics_angular_damping(&mut self, body: BodyId, angular_damping: f64) {
        unsafe {
            let command = ffi::b3InitChangeDynamicsInfo(self.handle.as_ptr());
            if angular_damping >= 0. {
                ffi::b3ChangeDynamicsInfoSetAngularDamping(command, body.0, angular_damping);
            }
            ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
        }
    }

    pub fn get_num_joints(&mut self, body: BodyId) -> i32 {
        unsafe { ffi::b3GetNumJoints(self.handle.as_ptr(), body.0) }
    }
    pub fn get_joint_info(&mut self, body: BodyId, joint_index: i32) -> b3JointInfo {
        unsafe {
            let mut joint_info = b3JointInfo {
                m_link_name: [2; 1024],
                m_joint_name: [2; 1024],
                m_joint_type: 0,
                m_q_index: 0,
                m_u_index: 0,
                m_joint_index: 0,
                m_flags: 0,
                m_joint_damping: 0.0,
                m_joint_friction: 0.0,
                m_joint_upper_limit: 0.0,
                m_joint_lower_limit: 0.0,
                m_joint_max_force: 0.0,
                m_joint_max_velocity: 0.0,
                m_parent_frame: [0.; 7],
                m_child_frame: [0.; 7],
                m_joint_axis: [0.; 3],
                m_parent_index: 0,
                m_q_size: 0,
                m_u_size: 0,
            };
            // let mut joint_info: b3JointInfo = b3JointInfo::default();
            ffi::b3GetJointInfo(self.handle.as_ptr(), body.0, joint_index, &mut joint_info);
            joint_info
        }
    }
    pub fn reset_joint_state(
        &mut self,
        body: BodyId,
        joint_index: i32,
        value: Option<f64>,
    ) -> Result<(), Error> {
        unsafe {
            let num_joints = ffi::b3GetNumJoints(self.handle.as_ptr(), body.0);
            if joint_index > num_joints {
                return Err(Error::new("Joint index out-of-range."));
            }
            let command_handle = ffi::b3CreatePoseCommandInit(self.handle.as_ptr(), body.0);

            ffi::b3CreatePoseCommandSetJointPosition(
                self.handle.as_ptr(),
                command_handle,
                joint_index,
                value.unwrap_or(0.),
            );
            ffi::b3CreatePoseCommandSetJointVelocity(
                self.handle.as_ptr(),
                command_handle,
                joint_index,
                0.,
            );
            let _handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
            Ok(())
        }
    }
    // TODO make some of the parameters optional
    #[allow(clippy::too_many_arguments)]
    pub fn calculate_inverse_kinematics(
        &mut self,
        body: BodyId,
        end_effector_link_index: i32,
        target_pos_obj: &[f64; 3],
        target_orn_obj: &[f64; 4],
        lower_limits: &[f64],
        upper_limits: &[f64],
        joint_ranges: &[f64],
        rest_poses: &[f64],
    ) -> Result<Vec<f64>, Error> {
        let solver = 0;
        let max_num_iterations = 5;
        let residual_threshold = -1.;

        let has_orn = true; // TODO make orientation optional

        let pos = target_pos_obj;
        let ori = target_orn_obj;
        let sz_lower_limits = lower_limits.len();
        let sz_upper_limits = upper_limits.len();
        let sz_joint_ranges = joint_ranges.len();
        let sz_rest_poses = rest_poses.len();
        let sz_joint_damping = 0;
        let sz_current_positions = 0;
        let dof_count = unsafe { ffi::b3ComputeDofCount(self.handle.as_ptr(), body.0) } as usize;

        let mut has_null_space = false;
        let mut has_joint_damping = false;
        let mut has_current_positions = false;

        let current_positions: Option<Vec<f64>> = None;
        let joint_damping: Option<Vec<f64>> = None;

        if dof_count != 0
            && (sz_lower_limits == dof_count)
            && (sz_upper_limits == dof_count)
            && (sz_joint_ranges == dof_count)
            && (sz_rest_poses == dof_count)
        {
            // let sz_in_bytes = std::mem::size_of::<f64>() * dof_count;
            has_null_space = true;
        }
        if sz_current_positions > 0 {
            if sz_current_positions != dof_count {
                return Err(Error::new(
                    "number of current_positions is not equal to the number of DoF's",
                ));
            } else {
                has_current_positions = true;
            }
        }
        if sz_joint_damping > 0 {
            if sz_current_positions < dof_count {
                return Err(Error::new("calculateInverseKinematics: the size of input joint damping values should be equal to the number of degrees of freedom, not using joint damping."));
            } else {
                has_joint_damping = true;
            }
        }
        let mut num_pos = 0;
        unsafe {
            let command =
                ffi::b3CalculateInverseKinematicsCommandInit(self.handle.as_ptr(), body.0);
            ffi::b3CalculateInverseKinematicsSelectSolver(command, solver);
            if has_current_positions {
                ffi::b3CalculateInverseKinematicsSetCurrentPositions(
                    command,
                    dof_count as i32,
                    current_positions.unwrap().as_ptr(),
                )
            }
            if max_num_iterations > 0 {
                ffi::b3CalculateInverseKinematicsSetMaxNumIterations(command, max_num_iterations);
            }
            if residual_threshold >= 0. {
                ffi::b3CalculateInverseKinematicsSetResidualThreshold(command, residual_threshold);
            }
            if has_null_space {
                if has_orn {
                    ffi::b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(
                        command,
                        dof_count as i32,
                        end_effector_link_index,
                        pos.as_ptr(),
                        ori.as_ptr(),
                        lower_limits.as_ptr(),
                        upper_limits.as_ptr(),
                        joint_ranges.as_ptr(),
                        rest_poses.as_ptr(),
                    );
                } else {
                    ffi::b3CalculateInverseKinematicsPosWithNullSpaceVel(
                        command,
                        dof_count as i32,
                        end_effector_link_index,
                        pos.as_ptr(),
                        lower_limits.as_ptr(),
                        upper_limits.as_ptr(),
                        joint_ranges.as_ptr(),
                        rest_poses.as_ptr(),
                    );
                }
            } else if has_orn {
                ffi::b3CalculateInverseKinematicsAddTargetPositionWithOrientation(
                    command,
                    end_effector_link_index,
                    pos.as_ptr(),
                    ori.as_ptr(),
                );
            } else {
                ffi::b3CalculateInverseKinematicsAddTargetPurePosition(
                    command,
                    end_effector_link_index,
                    pos.as_ptr(),
                );
            }

            if has_joint_damping {
                ffi::b3CalculateInverseKinematicsSetJointDamping(
                    command,
                    dof_count as i32,
                    joint_damping.unwrap().as_ptr(),
                )
            }
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
            let mut result_body_index: c_int = 0;
            let result = ffi::b3GetStatusInverseKinematicsJointPositions(
                status_handle,
                &mut result_body_index,
                &mut num_pos,
                &mut 0.,
            );
            if result != 0 && num_pos != 0 {
                let mut ik_output_joint_pos = vec![0.; num_pos as usize];
                ffi::b3GetStatusInverseKinematicsJointPositions(
                    status_handle,
                    &mut result_body_index,
                    &mut num_pos,
                    ik_output_joint_pos.as_mut_ptr(),
                );
                return Ok(ik_output_joint_pos);
            }
        }
        // let command =
        Err(Error::new("Error in calculateInverseKinematics"))
    }

    /// sets joint motor commands. This function differs a bit from the corresponding pybullet function.
    /// Instead of providing optional arguments that depend on the Control Mode. The necessary Parameters
    /// are directly encoded in the ControlMode Enum
    /// # Example
    /// ```rust
    /// use rubullet::{ControlMode, PhysicsClient, Mode};
    /// let mut client = PhysicsClient::connect(Mode::Direct)?;
    /// client.set_additional_search_path("bullet3/libbullet3/examples/pybullet/gym/pybullet_data")?;
    /// let panda_id = client.load_urdf("franka_panda/panda.urdf", Default::default())?;
    /// let joint_index = 1;
    /// client.set_joint_motor_control_2(panda_id, joint_index, ControlMode::Torque(100.), None);
    /// client.set_joint_motor_control_2(panda_id, joint_index, ControlMode::Position(0.4), Some(1000.));
    /// ```
    pub fn set_joint_motor_control_2(
        &mut self,
        body: BodyId,
        joint_index: i32,
        control_mode: ControlMode,
        maximum_force: Option<f64>,
    ) {
        let force = maximum_force.unwrap_or(100000.);
        let kp = 0.1;
        let kd = 1.0;
        let target_velocity = 0.;
        unsafe {
            let command_handle = ffi::b3JointControlCommandInit2(
                self.handle.as_ptr(),
                body.0,
                control_mode.get_int(),
            );
            let info = self.get_joint_info(body, joint_index);

            match control_mode {
                ControlMode::Position(target_position) => {
                    ffi::b3JointControlSetDesiredPosition(
                        command_handle,
                        info.m_q_index,
                        target_position,
                    );

                    ffi::b3JointControlSetKp(command_handle, info.m_u_index, kp);
                    ffi::b3JointControlSetDesiredVelocity(
                        command_handle,
                        info.m_u_index,
                        target_velocity,
                    );

                    ffi::b3JointControlSetKd(command_handle, info.m_u_index, kd);
                    ffi::b3JointControlSetMaximumForce(command_handle, info.m_u_index, force);
                }
                ControlMode::PD {
                    target_position: pos,
                    target_velocity: vel,
                    position_gain: pg,
                    velocity_gain: vg,
                    maximum_velocity: max_vel,
                } => {
                    if let Some(max_vel) = max_vel {
                        ffi::b3JointControlSetMaximumVelocity(
                            command_handle,
                            info.m_u_index,
                            max_vel,
                        );
                    }
                    ffi::b3JointControlSetDesiredPosition(command_handle, info.m_q_index, pos);

                    ffi::b3JointControlSetKp(command_handle, info.m_u_index, pg);
                    ffi::b3JointControlSetDesiredVelocity(command_handle, info.m_u_index, vel);

                    ffi::b3JointControlSetKd(command_handle, info.m_u_index, vg);
                    ffi::b3JointControlSetMaximumForce(command_handle, info.m_u_index, force);
                }
                ControlMode::Velocity(vel) => {
                    ffi::b3JointControlSetDesiredVelocity(command_handle, info.m_u_index, vel);
                    ffi::b3JointControlSetKd(command_handle, info.m_u_index, kd);
                    ffi::b3JointControlSetMaximumForce(command_handle, info.m_u_index, force);
                }
                ControlMode::Torque(f) => {
                    ffi::b3JointControlSetDesiredForceTorque(command_handle, info.m_u_index, f);
                }
            }
            let _status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
        }
    }

    // pub fn configure_debug_visualizer(&mut self,flag:DebugVisualizerFlag, enable:bool,light_position:Option<[f64;3]>, shadow_map_resolution:Option<i32>, shadow_map_world_size:Option<i32>) {
    pub fn configure_debug_visualizer(&mut self, flag: DebugVisualizerFlag, enable: bool) {
        unsafe {
            let command_handle = ffi::b3InitConfigureOpenGLVisualizer(self.handle.as_ptr());
            ffi::b3ConfigureOpenGLVisualizerSetVisualizationFlags(
                command_handle,
                flag as i32,
                enable as i32,
            );
            ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
        }
    }
}

impl Drop for PhysicsClient {
    fn drop(&mut self) {
        unsafe { ffi::b3DisconnectSharedMemory(self.handle.as_ptr()) }
    }
}

/// The unique ID for a body within a physics server.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct BodyId(c_int);

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
    fn get_int(&self) -> i32 {
        match self {
            ControlMode::Position(_) => 2,
            ControlMode::Velocity(_) => 0,
            ControlMode::Torque(_) => 1,
            ControlMode::PD { .. } => 3,
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

/// Module used to enforce the existence of only a single GUI
mod gui_marker {
    use std::sync::atomic::{AtomicBool, Ordering};

    /// A marker for whether or not a GUI has been started.
    ///
    /// PyBullet only allows a single GUI per process. I assume that this is a limitation of the
    /// underlying C API, so it will also be enforced here.
    static GUI_EXISTS: AtomicBool = AtomicBool::new(false);

    /// A marker type for keeping track of the existence of a GUI.
    pub struct GuiMarker {
        _unused: (),
    }

    impl GuiMarker {
        /// Attempts to acquire the GUI marker.
        pub fn acquire() -> Result<GuiMarker, crate::Error> {
            // We can probably use a weaker ordering but this will be called so little that we
            // may as well be sure about it.
            if GUI_EXISTS.compare_and_swap(false, true, Ordering::SeqCst) {
                Err(crate::Error::new(
                    "Only one in-process GUI connection allowed",
                ))
            } else {
                Ok(GuiMarker { _unused: () })
            }
        }
    }

    impl Drop for GuiMarker {
        fn drop(&mut self) {
            // We are the only marker so no need to CAS
            GUI_EXISTS.store(false, Ordering::SeqCst)
        }
    }
}

#[derive(Debug, PartialEq)]
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

impl b3JointInfo {
    pub fn get_joint_type(&self) -> JointType {
        JointType::try_from(self.m_joint_type).unwrap()
    }
}
