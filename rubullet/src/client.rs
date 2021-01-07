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

use nalgebra::{DMatrix, Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};

use self::gui_marker::GuiMarker;
use crate::{ControlMode, DebugVisualizerFlag, Error, Mode, UrdfOptions};
use image::{ImageBuffer, RgbaImage};
use rubullet_ffi as ffi;
use rubullet_ffi::EnumSharedMemoryServerStatus::{
    CMD_ACTUAL_STATE_UPDATE_COMPLETED, CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED,
    CMD_CALCULATED_JACOBIAN_COMPLETED, CMD_CALCULATED_MASS_MATRIX_COMPLETED,
    CMD_CAMERA_IMAGE_COMPLETED, CMD_CLIENT_COMMAND_COMPLETED, CMD_USER_DEBUG_DRAW_COMPLETED,
    CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED,
};
use rubullet_ffi::{
    b3CameraImageData, b3JointInfo, b3JointSensorState, b3KeyboardEventsData, b3LinkState,
    b3MouseEventsData,
};

use crate::types::{
    AddDebugLineOptions, AddDebugTextOptions, BodyId, ControlModeArray, ExternalForceFrame,
    InverseKinematicsParameters, Jacobian, JointInfo, JointState, JointType, KeyboardEvent,
    LinkState, MouseEvent,
};
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
    /// reset_simulation will remove all objects from the world and reset the world to initial conditions.
    pub fn reset_simulation(&mut self) {
        unsafe {
            let command_handle = ffi::b3InitResetSimulationCommand(self.handle.as_ptr());
            ffi::b3InitResetSimulationSetFlags(command_handle, 0);
            let _status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
        }
    }
    /// Warning: in many cases it is best to leave the timeStep to default, which is 240Hz.
    /// Several parameters are tuned with this value in mind. For example the number of solver
    /// iterations and the error reduction parameters (erp) for contact, friction and non-contact
    /// joints are related to the time step. If you change the time step, you may need to re-tune
    /// those values accordingly, especially the erp values.
    ///
    /// You can set the physics engine timestep that is used when calling
    /// [`step_simulation`](`Self::step_simulation()`).
    /// It is best to only call this method at the start of a simulation.
    /// Don't change this time step regularly.
    pub fn set_time_step(&mut self, time_step: &Duration) {
        unsafe {
            let command = ffi::b3InitPhysicsParamCommand(self.handle.as_ptr());
            let _ret = ffi::b3PhysicsParamSetTimeStep(command, time_step.as_secs_f64());
            let _status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
        }
    }
    /// By default, the physics server will not step the simulation, unless you explicitly send a
    /// [`step_simulation`](`Self::step_simulation()`) command.
    /// This way you can maintain control determinism of the simulation
    /// It is possible to run the simulation in real-time by letting the physics server
    /// automatically step the simulation according to its real-time-clock (RTC) using the
    /// set_real_time_simulation command. If you enable the real-time simulation,
    /// you don't need to call [`step_simulation`](`Self::step_simulation()`).
    ///
    /// Note that set_real_time_simulation has no effect in
    /// [`Direct mode`](`crate::mode::Mode::Direct`) :
    /// in [`Direct mode`](`crate::mode::Mode::Direct`) mode the physics
    /// server and client happen in the same thread and you trigger every command.
    /// In [`Gui mode`](`crate::mode::Mode::Gui`) and in Virtual Reality mode, and TCP/UDP mode,
    /// the physics server runs in a separate thread from the client (RuBullet),
    /// and set_real_time_simulation allows the physicsserver thread
    /// to add additional calls to  [`step_simulation`](`Self::step_simulation()`).
    ///
    /// # Arguments
    /// * `enable_real_time_simulation` - activates or deactivates real-time simulation
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
    /// You can reset the position and orientation of the base (root) of each object.
    /// It is best only to do this at the start, and not during a running simulation,
    /// since the command will override the effect of all physics simulation.
    /// The linear and angular velocity is set to zero.
    /// You can use [reset_base_velocity](`Self::reset_base_velocity()`)
    /// to reset to a non-zero linear and/or angular velocity.
    /// # Arguments
    /// * `body` - the [`BodyId`](`crate::types::BodyId`), as returned by [`load_urdf`](`Self::load_urdf()`) etc.
    /// * `pose` - reset the base of the object at the specified position in world space coordinates
    pub fn reset_base_transform(&mut self, body: BodyId, pose: &Isometry3<f64>) {
        unsafe {
            let command_handle = ffi::b3CreatePoseCommandInit(self.handle.as_ptr(), body.0);
            ffi::b3CreatePoseCommandSetBasePosition(
                command_handle,
                pose.translation.x,
                pose.translation.y,
                pose.translation.z,
            );
            ffi::b3CreatePoseCommandSetBaseOrientation(
                command_handle,
                pose.rotation.i,
                pose.rotation.j,
                pose.rotation.k,
                pose.rotation.w,
            );
            let _status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
        }
    }
    /// You get access to the linear and angular velocity of the base of a body.
    /// # Arguments
    /// * `body` - the [`BodyId`](`crate::types::BodyId`), as returned by [`load_urdf`](`Self::load_urdf()`) etc.
    /// # Return
    ///  Returns a result which is either a velocity array of size 6 (linear velocity, angular velocity)
    ///  or an Error
    /// # See also
    /// [reset_base_velocity](`Self::reset_base_velocity()`) to reset a base velocity and for examples
    pub fn get_base_velocity(&mut self, body: BodyId) -> Result<[f64; 6], Error> {
        let mut base_velocity = [0.; 6];
        unsafe {
            let cmd_handle = ffi::b3RequestActualStateCommandInit(self.handle.as_ptr(), body.0);
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), cmd_handle);
            let status_type = ffi::b3GetStatusType(status_handle);
            let actual_state_qdot: *mut f64 = ptr::null_mut();
            if status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED as i32 {
                return Err(Error::new("get_base_velocity_failed."));
            }
            ffi::b3GetStatusActualState(
                status_handle,
                ptr::null_mut(),
                ptr::null_mut(),
                ptr::null_mut(),
                ptr::null_mut(),
                ptr::null_mut(),
                &actual_state_qdot,
                ptr::null_mut(),
            );
            let base_velocity_slice = std::slice::from_raw_parts(actual_state_qdot, 6);
            base_velocity[..6].clone_from_slice(&base_velocity_slice[..6]);
        }
        Ok(base_velocity)
    }
    /// Reset the linear and/or angular velocity of the base of a body
    /// # Arguments
    /// * `body` - the [`BodyId`](`crate::types::BodyId`), as returned by [`load_urdf`](`Self::load_urdf()`) etc.
    /// * `linear_velocity` - either a &[f64;3] which contains the desired linear velocity (x,y,z)
    ///  in Cartesian world coordinates or `None` to not change the linear velocity
    /// * `angular_velocity` - either a &[f64;3] which contains the desired angular velocity
    /// (wx,wy,wz) in Cartesian world coordinates or `None` to not change the linear velocity
    ///
    /// # Errors
    /// * When you set both linear_velocity and angular_velocity to `None`
    ///
    /// # Example
    /// ```rust
    ///use easy_error::Terminator;
    ///use nalgebra::{Isometry3, Vector3};
    ///use rubullet::*;
    ///
    ///fn main() -> Result<(), Terminator> {
    ///    let mut physics_client = PhysicsClient::connect(Mode::Direct)?;
    ///    physics_client.set_additional_search_path("../rubullet-ffi/bullet3/libbullet3/data")?;
    ///    let _plane_id = physics_client.load_urdf("plane.urdf", Default::default())?;
    ///    let cube_start_position = Isometry3::translation(0.0, 0.0, 1.0);
    ///    let box_id = physics_client.load_urdf(
    ///        "r2d2.urdf",
    ///        UrdfOptions {
    ///            base_transform: cube_start_position,
    ///            ..Default::default()
    ///        },
    ///    )?;
    ///
    ///    physics_client
    ///        .reset_base_velocity(box_id, &[1., 2., 3.], &[4., 5., 6.])?;
    ///    let velocity = physics_client.get_base_velocity(box_id)?;
    ///    assert_eq!(velocity, [1., 2., 3., 4., 5., 6.]);
    ///
    ///    physics_client
    ///        .reset_base_velocity(box_id, &[0., 0., 0.], None)?;
    ///    let velocity = physics_client.get_base_velocity(box_id)?;
    ///    assert_eq!(velocity, [0., 0., 0., 4., 5., 6.]);
    ///
    ///    physics_client
    ///        .reset_base_velocity(box_id, None, &[0., 0., 0.])?;
    ///    let velocity = physics_client.get_base_velocity(box_id)?;
    ///    assert_eq!(velocity, [0.; 6]);
    ///
    ///    assert!(physics_client
    ///        .reset_base_velocity(box_id, None, None)
    ///        .is_err());
    ///    Ok(())
    ///}
    /// ```
    pub fn reset_base_velocity<
        'a,
        Linear: Into<Option<&'a [f64; 3]>>,
        Angular: Into<Option<&'a [f64; 3]>>,
    >(
        &mut self,
        body: BodyId,
        linear_velocity: Linear,
        angular_velocity: Angular,
    ) -> Result<(), Error> {
        let maybe_lin = linear_velocity.into();
        let maybe_angular = angular_velocity.into();
        unsafe {
            let command_handle = ffi::b3CreatePoseCommandInit(self.handle.as_ptr(), body.0);
            match (maybe_lin, maybe_angular) {
                (None, None) => {
                    return Err(Error::new(
                        "expected at least linearVelocity and/or angularVelocity.",
                    ));
                }
                (Some(linear), Some(angular)) => {
                    ffi::b3CreatePoseCommandSetBaseLinearVelocity(command_handle, linear.as_ptr());
                    ffi::b3CreatePoseCommandSetBaseAngularVelocity(
                        command_handle,
                        angular.as_ptr(),
                    );
                }
                (Some(linear), None) => {
                    ffi::b3CreatePoseCommandSetBaseLinearVelocity(command_handle, linear.as_ptr());
                }
                (None, Some(angular)) => {
                    ffi::b3CreatePoseCommandSetBaseAngularVelocity(
                        command_handle,
                        angular.as_ptr(),
                    );
                }
            }
            let _status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
        }
        Ok(())
    }
    /// Queries the Cartesian world pose for the center of mass for a link.
    /// It will also report the local inertial frame of the center of mass to the URDF link frame,
    /// to make it easier to compute the graphics/visualization frame.
    ///
    /// # Warning
    /// * the returned link velocity will only be valid if `compute_link_velocity` is set to true
    ///
    /// # Arguments
    /// * `body` - the [`BodyId`](`crate::types::BodyId`), as returned by [`load_urdf`](`Self::load_urdf()`) etc.
    /// * `link_index` - link index
    /// * `compute_link_velocity` - will compute the link velocity and put it into the [`LinkState`](`crate::types::LinkState`) if set to true. Otherwise the velocity within the LinkState will be invalid.
    /// * `compute_forward_kinematics` - if true  the Cartesian world pose will be recomputed using forward kinematics.
    ///
    /// # See also
    /// * [`LinkState`](`crate::types::LinkState`) for more information about different types of link frames
    /// * [`get_link_states()`](`Self::get_link_states()`) to get multiple link states
    pub fn get_link_state(
        &mut self,
        body: BodyId,
        link_index: i32,
        compute_link_velocity: bool,
        compute_forward_kinematics: bool,
    ) -> Result<LinkState, Error> {
        unsafe {
            if body.0 < 0 {
                return Err(Error::new("getLinkState failed; invalid bodyUniqueId"));
            }
            if link_index < 0 {
                return Err(Error::new("getLinkState failed; invalid linkIndex"));
            }
            let cmd_handle = ffi::b3RequestActualStateCommandInit(self.handle.as_ptr(), body.0);
            if compute_link_velocity {
                ffi::b3RequestActualStateCommandComputeLinkVelocity(cmd_handle, 1);
            }
            if compute_forward_kinematics {
                ffi::b3RequestActualStateCommandComputeForwardKinematics(cmd_handle, 1);
            }
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), cmd_handle);
            let status_type = ffi::b3GetStatusType(status_handle);
            if status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED as i32 {
                return Err(Error::new("getLinkState failed."));
            }
            let mut link_state = b3LinkState::default();
            if ffi::b3GetLinkState(
                self.handle.as_ptr(),
                status_handle,
                link_index,
                &mut link_state,
            ) != 0
            {
                return Ok(link_state.into());
            }
        }
        Err(Error::new("getLinkState failed."))
    }
    /// getLinkStates will return the information for multiple links.
    /// Instead of link_index it will accept link_indices as an array of i32.
    /// This can improve performance by reducing calling overhead of multiple calls to
    /// [`get_link_state()`](`Self::get_link_states()`) .
    ///
    /// # Warning
    /// * the returned link velocity will only be valid if `compute_link_velocity` is set to true
    ///
    /// # Arguments
    /// * `body` - the [`BodyId`](`crate::types::BodyId`), as returned by [`load_urdf`](`Self::load_urdf()`) etc.
    /// * `link_indices` - link indices
    /// * `compute_link_velocity` - will compute the link velocity and put it into the [`LinkState`](`crate::types::LinkState`) if set to true. Otherwise the velocity within the LinkState will be invalid.
    /// * `compute_forward_kinematics` - if true  the Cartesian world pose will be recomputed using forward kinematics.
    ///
    /// # See also
    /// * [`LinkState`](`crate::types::LinkState`) for more information about different types of link frames
    /// * [`get_link_state()`](`Self::get_link_states()`) to get only a single link state
    pub fn get_link_states(
        &mut self,
        body: BodyId,
        link_indices: &[i32],
        compute_link_velocity: bool,
        compute_forward_kinematics: bool,
    ) -> Result<Vec<LinkState>, Error> {
        unsafe {
            if body.0 < 0 {
                return Err(Error::new("getLinkState failed; invalid bodyUniqueId"));
            }
            if link_indices.iter().any(|&x| x < 0) {
                return Err(Error::new("getLinkState failed; invalid linkIndex"));
            }
            let cmd_handle = ffi::b3RequestActualStateCommandInit(self.handle.as_ptr(), body.0);
            if compute_link_velocity {
                ffi::b3RequestActualStateCommandComputeLinkVelocity(cmd_handle, 1);
            }
            if compute_forward_kinematics {
                ffi::b3RequestActualStateCommandComputeForwardKinematics(cmd_handle, 1);
            }
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), cmd_handle);
            let status_type = ffi::b3GetStatusType(status_handle);
            if status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED as i32 {
                return Err(Error::new("getLinkState failed."));
            }
            let mut link_states = Vec::<LinkState>::with_capacity(link_indices.len());
            for &link_index in link_indices.iter() {
                let mut link_state = b3LinkState::default();
                if ffi::b3GetLinkState(
                    self.handle.as_ptr(),
                    status_handle,
                    link_index,
                    &mut link_state,
                ) != 0
                {
                    link_states.push(link_state.into());
                } else {
                    return Err(Error::new("getLinkStates failed."));
                }
            }
            Ok(link_states)
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
    /// returns the number of joints of a body
    /// # Arguments
    /// * `body` - the [`BodyId`](`crate::types::BodyId`), as returned by [`load_urdf`](`Self::load_urdf()`) etc.
    pub fn get_num_joints(&mut self, body: BodyId) -> i32 {
        unsafe { ffi::b3GetNumJoints(self.handle.as_ptr(), body.0) }
    }
    /// Query info about a joint like its name and type
    /// # Arguments
    /// * `body` - the [`BodyId`](`crate::types::BodyId`), as returned by [`load_urdf`](`Self::load_urdf()`) etc.
    /// * `joint_index` - an index in the range \[0..[`get_num_joints(body)`](`Self::get_num_joints()`)\]
    ///
    /// See [JointInfo](`crate::types::JointInfo`) for an example use
    pub fn get_joint_info(&mut self, body: BodyId, joint_index: i32) -> JointInfo {
        self.get_joint_info_intern(body, joint_index).into()
    }

    fn get_joint_info_intern(&mut self, body: BodyId, joint_index: i32) -> b3JointInfo {
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
                m_joint_lower_limit: 0.0,
                m_joint_upper_limit: 0.0,
                m_joint_max_force: 0.0,
                m_joint_max_velocity: 0.0,
                m_parent_frame: [0.; 7],
                m_child_frame: [0.; 7],
                m_joint_axis: [0.; 3],
                m_parent_index: 0,
                m_q_size: 0,
                m_u_size: 0,
            };
            ffi::b3GetJointInfo(self.handle.as_ptr(), body.0, joint_index, &mut joint_info);
            joint_info
        }
    }
    /// You can reset the state of the joint. It is best only to do this at the start,
    /// while not running the simulation: resetJointState overrides all physics simulation.
    /// Note that we only support 1-DOF motorized joints at the moment,
    /// sliding joint or revolute joints.
    /// # Arguments
    /// * `body` - the [`BodyId`](`crate::types::BodyId`), as returned by [`load_urdf`](`Self::load_urdf()`) etc.
    /// * `joint_index` - a joint index in the range \[0..[`get_num_joints(body)`](`Self::get_num_joints()`)\]
    /// * `value` - the joint position (angle in radians or position)
    /// * `velocity`- optional joint velocity (angular or linear velocity)
    pub fn reset_joint_state<Velocity: Into<Option<f64>>>(
        &mut self,
        body: BodyId,
        joint_index: i32,
        value: f64,
        velocity: Velocity,
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
                value,
            );
            ffi::b3CreatePoseCommandSetJointVelocity(
                self.handle.as_ptr(),
                command_handle,
                joint_index,
                velocity.into().unwrap_or(0.),
            );
            let _handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
            Ok(())
        }
    }
    /// get information about the [joint state](`crate::types::JointState`)
    /// such as the joint position, velocity, joint reaction forces and joint motor torque.
    /// # Arguments
    /// * `body` - the [`BodyId`](`crate::types::BodyId`), as returned by [`load_urdf`](`Self::load_urdf()`) etc.
    /// * `joint_index` - a joint index in the range \[0..[`get_num_joints(body)`](`Self::get_num_joints()`)\]
    pub fn get_joint_state(&mut self, body: BodyId, joint_index: i32) -> Result<JointState, Error> {
        unsafe {
            if body.0 < 0 {
                return Err(Error::new("getJointState failed; invalid BodyId"));
            }
            if joint_index < 0 {
                return Err(Error::new("getJointState failed; invalid joint_index"));
            }
            let cmd_handle = ffi::b3RequestActualStateCommandInit(self.handle.as_ptr(), body.0);
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), cmd_handle);
            let status_type = ffi::b3GetStatusType(status_handle);
            if status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED as i32 {
                return Err(Error::new("getJointState failed."));
            }
            let mut sensor_state = b3JointSensorState::default();
            if 0 != ffi::b3GetJointState(
                self.handle.as_ptr(),
                status_handle,
                joint_index,
                &mut sensor_state,
            ) {
                return Ok(sensor_state.into());
            }
        }
        Err(Error::new("getJointState failed (2)."))
    }
    /// get_joint_states is the array version of [get_joint_state](`Self::get_joint_state()`).
    /// Instead of passing in a single joint_index, you pass in a list of joint_indices.
    /// # Arguments
    /// * `body` - the [`BodyId`](`crate::types::BodyId`), as returned by [`load_urdf`](`Self::load_urdf()`) etc.
    /// * `joint_indices` - a list of joint indices which each index  in the range \[0..[`get_num_joints(body)`](`Self::get_num_joints()`)\]
    pub fn get_joint_states(
        &mut self,
        body: BodyId,
        joint_indices: &[i32],
    ) -> Result<Vec<JointState>, Error> {
        unsafe {
            if body.0 < 0 {
                return Err(Error::new("getJointState failed; invalid BodyId"));
            }
            let num_joints = self.get_num_joints(body);
            if joint_indices.is_empty() {
                return Err(Error::new("expected a sequence of joint indices"));
            }
            let cmd_handle = ffi::b3RequestActualStateCommandInit(self.handle.as_ptr(), body.0);
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), cmd_handle);
            let status_type = ffi::b3GetStatusType(status_handle);
            if status_type != CMD_ACTUAL_STATE_UPDATE_COMPLETED as i32 {
                return Err(Error::new("getJointState failed."));
            }
            let mut result_list_joint_states =
                Vec::<JointState>::with_capacity(num_joints as usize);
            for &joint_index in joint_indices.iter() {
                if joint_index < 0 || joint_index >= num_joints {
                    return Err(Error::new("getJointStates failed; invalid joint_index"));
                }
                let mut sensor_state = b3JointSensorState::default();
                if 0 != ffi::b3GetJointState(
                    self.handle.as_ptr(),
                    status_handle,
                    joint_index,
                    &mut sensor_state,
                ) {
                    result_list_joint_states.push(sensor_state.into());
                } else {
                    return Err(Error::new("getJointState failed (2)."));
                }
            }
            Ok(result_list_joint_states)
        }
    }
    pub fn calculate_mass_matrix(
        &mut self,
        body: BodyId,
        object_positions: &[f64],
    ) -> Result<DMatrix<f64>, Error> {
        if !object_positions.is_empty() {
            let joint_positions = object_positions;
            let flags = 0; // TODO add flags
            unsafe {
                let command_handle = ffi::b3CalculateMassMatrixCommandInit(
                    self.handle.as_ptr(),
                    body.0,
                    joint_positions.as_ptr(),
                    joint_positions.len() as i32,
                );
                ffi::b3CalculateMassMatrixSetFlags(command_handle, flags);
                let status_handle =
                    ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
                let status_type = ffi::b3GetStatusType(status_handle);
                if status_type == CMD_CALCULATED_MASS_MATRIX_COMPLETED as i32 {
                    let mut dof_count = 0;
                    ffi::b3GetStatusMassMatrix(
                        self.handle.as_ptr(),
                        status_handle,
                        &mut dof_count,
                        std::ptr::null_mut(),
                    );
                    if dof_count != 0 {
                        let mut mass_vec = vec![0.; (dof_count * dof_count) as usize];
                        ffi::b3GetStatusMassMatrix(
                            self.handle.as_ptr(),
                            status_handle,
                            &mut dof_count,
                            mass_vec.as_mut_slice().as_mut_ptr(),
                        );
                        let mass_mat = DMatrix::<f64>::from_row_slice(
                            dof_count as usize,
                            dof_count as usize,
                            mass_vec.as_slice(),
                        );
                        return Ok(mass_mat);
                    }
                } else {
                    return Err(Error::new("Internal error in calculateMassMatrix"));
                }
            }
        }
        Err(Error::new("error in calculate_mass_matrix"))
    }

    pub fn calculate_inverse_kinematics(
        &mut self,
        params: InverseKinematicsParameters,
    ) -> Result<Vec<f64>, Error> {
        let solver = params.solver.into();
        let max_num_iterations = params.max_num_iterations;
        let residual_threshold = params.residual_threshold.unwrap_or(-1.);
        let body = params.body;
        let end_effector_link_index = params.end_effector_link_index;

        let pos = params.target_position;
        let ori = params.target_orientation;
        let mut sz_lower_limits = 0;
        let mut sz_upper_limits = 0;
        let mut sz_joint_ranges = 0;
        let mut sz_rest_poses = 0;

        if params.limits.is_some() {
            let limits = params.limits.as_ref().unwrap();
            sz_lower_limits = limits.lower_limits.len();
            sz_upper_limits = limits.upper_limits.len();
            sz_joint_ranges = limits.joint_ranges.len();
            sz_rest_poses = limits.rest_poses.len();
        }

        let dof_count = unsafe { ffi::b3ComputeDofCount(self.handle.as_ptr(), body.0) } as usize;

        let mut has_null_space = false;
        let mut has_joint_damping = false;
        let mut has_current_positions = false;

        let current_positions = params.current_position;
        let joint_damping = params.joint_damping;

        if dof_count != 0
            && (sz_lower_limits == dof_count)
            && (sz_upper_limits == dof_count)
            && (sz_joint_ranges == dof_count)
            && (sz_rest_poses == dof_count)
        {
            has_null_space = true;
        } else if params.limits.is_some() {
            eprintln!("Warning! Null space parameter lengths do not match the number DoF and will be ignored!");
        }
        if let Some(positions) = current_positions {
            if positions.len() != dof_count {
                return Err(Error::new(
                    "number of current_positions is not equal to the number of DoF's",
                ));
            } else {
                has_current_positions = true;
            }
        }
        if let Some(damping) = joint_damping {
            if damping.len() < dof_count {
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
                let limits = params.limits.unwrap();
                let lower_limits = limits.lower_limits;
                let upper_limits = limits.upper_limits;
                let joint_ranges = limits.joint_ranges;
                let rest_poses = limits.rest_poses;
                if let Some(orientation) = ori {
                    ffi::b3CalculateInverseKinematicsPosOrnWithNullSpaceVel(
                        command,
                        dof_count as i32,
                        end_effector_link_index,
                        pos.as_ptr(),
                        orientation.as_ptr(),
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
            } else if ori.is_some() {
                ffi::b3CalculateInverseKinematicsAddTargetPositionWithOrientation(
                    command,
                    end_effector_link_index,
                    pos.as_ptr(),
                    ori.unwrap().as_ptr(),
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
                    ik_output_joint_pos.as_mut_slice().as_mut_ptr(),
                );
                return Ok(ik_output_joint_pos);
            }
        }
        Err(Error::new("Error in calculateInverseKinematics"))
    }

    pub fn calculate_inverse_dynamics(
        &mut self,
        body_id: BodyId,
        object_positions: &[f64],
        object_velocities: &[f64],
        object_accelerations: &[f64],
    ) -> Result<Vec<f64>, Error> {
        let flags = 0; // TODO find out what those flags are and let the user set them
        if object_velocities.len() != object_accelerations.len() {
            return Err(Error::new(
                "number of object velocities should be equal to the number of object accelerations",
            ));
        }
        unsafe {
            let command_handle = ffi::b3CalculateInverseDynamicsCommandInit2(
                self.handle.as_ptr(),
                body_id.0,
                object_positions.as_ptr(),
                object_positions.len() as i32,
                object_velocities.as_ptr(),
                object_accelerations.as_ptr(),
                object_velocities.len() as i32,
            );
            ffi::b3CalculateInverseDynamicsSetFlags(command_handle, flags);
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
            let status_type = ffi::b3GetStatusType(status_handle);
            if status_type == CMD_CALCULATED_INVERSE_DYNAMICS_COMPLETED as i32 {
                let mut body_unique_id = 0;
                let mut dof_count = 0;
                ffi::b3GetStatusInverseDynamicsJointForces(
                    status_handle,
                    &mut body_unique_id,
                    &mut dof_count,
                    &mut 0.,
                );
                if dof_count != 0 {
                    let mut joint_forces_output = vec![0.; dof_count as usize];
                    ffi::b3GetStatusInverseDynamicsJointForces(
                        status_handle,
                        &mut 0,
                        &mut 0,
                        joint_forces_output.as_mut_slice().as_mut_ptr(),
                    );
                    return Ok(joint_forces_output);
                }
            }
        }
        Err(Error::new(
            "Error in calculateInverseDynamics, please check arguments.",
        ))
    }

    pub fn calculate_jacobian(
        &mut self,
        body_id: BodyId,
        link_index: i32,
        local_position: &Translation3<f64>,
        object_positions: &[f64],
        object_velocities: &[f64],
        object_accelerations: &[f64],
    ) -> Result<Jacobian, Error> {
        if object_positions.len() != object_velocities.len() {
            return Err(Error::new(
                "object_velocities  has not the same size as object_positions",
            ));
        }
        if object_positions.len() != object_accelerations.len() {
            return Err(Error::new(
                "object_accelerations  has not the same size as object_positions",
            ));
        }
        let num_joints = self.get_num_joints(body_id);
        let mut dof_count_org = 0;
        for j in 0..num_joints {
            let joint_type =
                JointType::try_from(self.get_joint_info_intern(body_id, j).m_joint_type).unwrap();
            match joint_type {
                JointType::Revolute | JointType::Prismatic => {
                    dof_count_org += 1;
                }
                JointType::Spherical => {
                    return Err(Error::new(
                        "Spherical joints are not supported in the rubullet binding",
                    ))
                }
                JointType::Planar => {
                    return Err(Error::new(
                        "Planar joints are not supported in the rubullet binding",
                    ))
                }
                _ => {}
            }
        }
        if dof_count_org != 0 && dof_count_org == object_positions.len() {
            let local_point = local_position.vector.as_slice();
            let joint_positions = object_positions;
            let joint_velocities = object_velocities;
            let joint_accelerations = object_accelerations;

            unsafe {
                let command_handle = ffi::b3CalculateJacobianCommandInit(
                    self.handle.as_ptr(),
                    body_id.0,
                    link_index,
                    local_point.as_ptr(),
                    joint_positions.as_ptr(),
                    joint_velocities.as_ptr(),
                    joint_accelerations.as_ptr(),
                );
                let status_handle =
                    ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
                let status_type = ffi::b3GetStatusType(status_handle);
                if status_type == CMD_CALCULATED_JACOBIAN_COMPLETED as i32 {
                    let mut dof_count = 0;
                    ffi::b3GetStatusJacobian(
                        status_handle,
                        &mut dof_count,
                        std::ptr::null_mut(),
                        std::ptr::null_mut(),
                    );
                    if dof_count != 0 {
                        let mut linear_jacobian = vec![0.; 3 * dof_count as usize];
                        let mut angular_jacobian = vec![0.; 3 * dof_count as usize];
                        ffi::b3GetStatusJacobian(
                            status_handle,
                            &mut dof_count,
                            linear_jacobian.as_mut_slice().as_mut_ptr(),
                            angular_jacobian.as_mut_slice().as_mut_ptr(),
                        );
                        let linear_jacobian_mat = DMatrix::from_row_slice(
                            3,
                            dof_count as usize,
                            linear_jacobian.as_mut_slice(),
                        );
                        let angular_jacobian_mat = DMatrix::from_row_slice(
                            3,
                            dof_count as usize,
                            angular_jacobian.as_mut_slice(),
                        );
                        return Ok(Jacobian {
                            linear_jacobian: linear_jacobian_mat,
                            angular_jacobian: angular_jacobian_mat,
                        });
                    }
                }
            }
        }
        Err(Error::new("Error in calculateJacobian"))
    }

    /// sets joint motor commands. This function differs a bit from the corresponding pybullet function.
    /// Instead of providing optional arguments that depend on the Control Mode. The necessary Parameters
    /// are directly encoded in the ControlMode Enum
    /// # Example
    /// ```rust
    /// use rubullet::{ControlMode, PhysicsClient, Mode};
    /// use easy_error::Terminator;
    /// pub fn main() -> Result<(),Terminator> {
    ///     let mut client = PhysicsClient::connect(Mode::Direct)?;
    ///     client.set_additional_search_path("../rubullet-ffi/bullet3/libbullet3/examples/pybullet/gym/pybullet_data")?;
    ///     let panda_id = client.load_urdf("franka_panda/panda.urdf", Default::default())?;
    ///     let joint_index = 1;
    ///     client.set_joint_motor_control_2(panda_id, joint_index, ControlMode::Torque(100.), None);
    ///     client.set_joint_motor_control_2(panda_id, joint_index, ControlMode::Position(0.4), Some(1000.));
    /// Ok(())
    /// }
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
            let info = self.get_joint_info_intern(body, joint_index);

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
                    position_gain: kp,
                    velocity_gain: kd,
                    maximum_velocity: max_vel,
                }
                | ControlMode::PositionWithPD {
                    target_position: pos,
                    target_velocity: vel,
                    position_gain: kp,
                    velocity_gain: kd,
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

                    ffi::b3JointControlSetKp(command_handle, info.m_u_index, kp);
                    ffi::b3JointControlSetDesiredVelocity(command_handle, info.m_u_index, vel);

                    ffi::b3JointControlSetKd(command_handle, info.m_u_index, kd);
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

    pub fn set_joint_motor_control_array(
        &mut self,
        body: BodyId,
        joint_indices: &[i32],
        control_mode: ControlModeArray,
        maximum_force: Option<Vec<f64>>,
    ) -> Result<(), Error> {
        let forces = maximum_force.unwrap_or_else(|| vec![100000.; joint_indices.len()]);
        if forces.len() != joint_indices.len() {
            return Err(Error::new(
                "number of maximum forces should match the number of joint indices",
            ));
        }
        let kp = 0.1;
        let kd = 1.0;
        let target_velocities = vec![0.; joint_indices.len()];
        let num_joints = self.get_num_joints(body);
        unsafe {
            let command_handle = ffi::b3JointControlCommandInit2(
                self.handle.as_ptr(),
                body.0,
                control_mode.get_int(),
            );

            for &joint_index in joint_indices.iter() {
                if joint_index < 0 || joint_index >= num_joints {
                    return Err(Error::new("Joint index out-of-range."));
                }
            }
            match control_mode {
                ControlModeArray::Positions(target_positions) => {
                    if target_positions.len() != joint_indices.len() {
                        return Err(Error::new(
                            "number of target positions should match the number of joint indices",
                        ));
                    }
                    for i in 0..target_positions.len() {
                        let info = self.get_joint_info_intern(body, joint_indices[i]);
                        ffi::b3JointControlSetDesiredPosition(
                            command_handle,
                            info.m_q_index,
                            target_positions[i],
                        );
                        ffi::b3JointControlSetKp(command_handle, info.m_u_index, kp);
                        ffi::b3JointControlSetDesiredVelocity(
                            command_handle,
                            info.m_u_index,
                            target_velocities[i],
                        );

                        ffi::b3JointControlSetKd(command_handle, info.m_u_index, kd);
                        ffi::b3JointControlSetMaximumForce(
                            command_handle,
                            info.m_u_index,
                            forces[i],
                        );
                    }
                }
                ControlModeArray::PD {
                    target_positions: pos,
                    target_velocities: vel,
                    position_gains: pg,
                    velocity_gains: vg,
                }
                | ControlModeArray::PositionsWithPD {
                    target_positions: pos,
                    target_velocities: vel,
                    position_gains: pg,
                    velocity_gains: vg,
                } => {
                    if pos.len() != joint_indices.len() {
                        return Err(Error::new(
                            "number of target positions should match the number of joint indices",
                        ));
                    }
                    if vel.len() != joint_indices.len() {
                        return Err(Error::new(
                            "number of target velocities should match the number of joint indices",
                        ));
                    }
                    if pg.len() != joint_indices.len() {
                        return Err(Error::new(
                            "number of position gains should match the number of joint indices",
                        ));
                    }
                    if vg.len() != joint_indices.len() {
                        return Err(Error::new(
                            "number of velocity gains should match the number of joint indices",
                        ));
                    }
                    for i in 0..pos.len() {
                        let info = self.get_joint_info_intern(body, joint_indices[i]);
                        ffi::b3JointControlSetDesiredPosition(
                            command_handle,
                            info.m_q_index,
                            pos[i],
                        );

                        ffi::b3JointControlSetKp(command_handle, info.m_u_index, pg[i]);
                        ffi::b3JointControlSetDesiredVelocity(
                            command_handle,
                            info.m_u_index,
                            vel[i],
                        );

                        ffi::b3JointControlSetKd(command_handle, info.m_u_index, vg[i]);
                        ffi::b3JointControlSetMaximumForce(
                            command_handle,
                            info.m_u_index,
                            forces[i],
                        );
                    }
                }
                ControlModeArray::Velocities(vel) => {
                    if vel.len() != joint_indices.len() {
                        return Err(Error::new(
                            "number of target velocities should match the number of joint indices",
                        ));
                    }
                    for i in 0..vel.len() {
                        let info = self.get_joint_info_intern(body, joint_indices[i]);
                        ffi::b3JointControlSetDesiredVelocity(
                            command_handle,
                            info.m_u_index,
                            vel[i],
                        );
                        ffi::b3JointControlSetKd(command_handle, info.m_u_index, kd);
                        ffi::b3JointControlSetMaximumForce(
                            command_handle,
                            info.m_u_index,
                            forces[i],
                        );
                    }
                }
                ControlModeArray::Torques(f) => {
                    if f.len() != joint_indices.len() {
                        return Err(Error::new(
                            "number of target torques should match the number of joint indices",
                        ));
                    }
                    for i in 0..f.len() {
                        let info = self.get_joint_info_intern(body, joint_indices[i]);
                        ffi::b3JointControlSetDesiredForceTorque(
                            command_handle,
                            info.m_u_index,
                            f[i],
                        );
                    }
                }
            }
            let _status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
        }
        Ok(())
    }

    pub fn compute_view_matrix(
        camera_eye_position: &[f32; 3],
        camera_target_position: &[f32; 3],
        camera_up_vector: &[f32; 3],
    ) -> [f32; 16] {
        let mut view_matrix = [0_f32; 16];
        unsafe {
            ffi::b3ComputeViewMatrixFromPositions(
                camera_eye_position.as_ptr(),
                camera_target_position.as_ptr(),
                camera_up_vector.as_ptr(),
                view_matrix.as_mut_ptr(),
            );
            view_matrix
        }
    }
    pub fn compute_view_matrix_from_yaw_pitch_roll(
        camera_target_position: &[f32; 3],
        distance: f32,
        yaw: f32,
        pitch: f32,
        roll: f32,
        up_axis_index: i32,
    ) -> [f32; 16] {
        let mut view_matrix = [0_f32; 16];
        unsafe {
            ffi::b3ComputeViewMatrixFromYawPitchRoll(
                camera_target_position.as_ptr(),
                distance,
                yaw,
                pitch,
                roll,
                up_axis_index,
                view_matrix.as_mut_ptr(),
            );
            view_matrix
        }
    }

    pub fn compute_projection_matrix(
        left: f32,
        right: f32,
        bottom: f32,
        top: f32,
        near_val: f32,
        far_val: f32,
    ) -> [f32; 16] {
        let mut projection_matrix = [0_f32; 16];
        unsafe {
            ffi::b3ComputeProjectionMatrix(
                left,
                right,
                bottom,
                top,
                near_val,
                far_val,
                projection_matrix.as_mut_ptr(),
            );
            projection_matrix
        }
    }
    pub fn compute_projection_matrix_fov(
        fov: f32,
        aspect: f32,
        near_val: f32,
        far_val: f32,
    ) -> [f32; 16] {
        let mut projection_matrix = [0_f32; 16];
        unsafe {
            ffi::b3ComputeProjectionMatrixFOV(
                fov,
                aspect,
                near_val,
                far_val,
                projection_matrix.as_mut_ptr(),
            );
            projection_matrix
        }
    }
    pub fn get_camera_image(
        &mut self,
        width: i32,
        height: i32,
        view_matrix: &[f32; 16],
        projection_matrix: &[f32; 16],
    ) -> Result<RgbaImage, Error> {
        unsafe {
            let command = ffi::b3InitRequestCameraImage(self.handle.as_ptr());
            ffi::b3RequestCameraImageSetPixelResolution(command, width, height);
            ffi::b3RequestCameraImageSetCameraMatrices(
                command,
                view_matrix.as_ptr(),
                projection_matrix.as_ptr(),
            );
            if self.can_submit_command() {
                let status_handle =
                    ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
                let status_type = ffi::b3GetStatusType(status_handle);
                if status_type == CMD_CAMERA_IMAGE_COMPLETED as i32 {
                    let mut image_data = b3CameraImageData::default();
                    ffi::b3GetCameraImageData(self.handle.as_ptr(), &mut image_data);
                    let buffer = std::slice::from_raw_parts(
                        image_data.m_rgb_color_data,
                        (width * height * 4) as usize,
                    );
                    let img: RgbaImage =
                        ImageBuffer::from_vec(width as u32, height as u32, buffer.into()).unwrap();
                    return Ok(img);
                }
            }
            Err(Error::new("getCameraImage failed"))
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

    pub fn add_user_debug_line<'a, Options: Into<Option<AddDebugLineOptions<'a>>>>(
        &mut self,
        line_from_xyz: &[f64],
        line_to_xyz: &[f64],
        options: Options,
    ) -> Result<BodyId, Error> {
        unsafe {
            let options = options.into().unwrap_or_default();
            let command_handle = ffi::b3InitUserDebugDrawAddLine3D(
                self.handle.as_ptr(),
                line_from_xyz.as_ptr(),
                line_to_xyz.as_ptr(),
                options.line_color_rgb.as_ptr(),
                options.line_width,
                options.life_time,
            );
            if let Some(parent) = options.parent_object_id {
                ffi::b3UserDebugItemSetParentObject(
                    command_handle,
                    parent.0,
                    options.parent_link_index.unwrap_or(-1),
                );
            }
            if let Some(replacment) = options.replace_item_id {
                ffi::b3UserDebugItemSetReplaceItemUniqueId(command_handle, replacment.0);
            }
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
            let status_type = ffi::b3GetStatusType(status_handle);
            if status_type == CMD_USER_DEBUG_DRAW_COMPLETED as i32 {
                let debug_item = BodyId(ffi::b3GetDebugItemUniqueId(status_handle));
                return Ok(debug_item);
            }
            Err(Error::new("Error in addUserDebugLine."))
        }
    }
    ///
    /// Buttons are currently not working
    /// # Example
    /// ```no_run
    /// use rubullet::PhysicsClient;
    /// use rubullet::mode::Mode::Gui;
    ///   use easy_error::Terminator;
    /// pub fn main() -> Result<(),Terminator> {
    ///     let mut client = PhysicsClient::connect(Gui)?;
    ///     let slider = client.add_user_debug_parameter("my_slider",0.,1.,0.5)?;
    ///     let button = client.add_user_debug_parameter("my_button",1.,0.,1.)?;
    ///     let value_slider = client.read_user_debug_parameter(slider)?;
    ///     let value_button = client.read_user_debug_parameter(button)?; // value increases by one for every press
    ///     Ok(())
    /// }
    /// ```
    // TODO Figure out why button are not working
    pub fn add_user_debug_parameter<Text: Into<String>>(
        &mut self,
        param_name: Text,
        range_min: f64,
        range_max: f64,
        start_value: f64,
    ) -> Result<BodyId, Error> {
        unsafe {
            let command_handle = ffi::b3InitUserDebugAddParameter(
                self.handle.as_ptr(),
                param_name.into().as_str().as_ptr(),
                range_min,
                range_max,
                start_value,
            );
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
            let status_type = ffi::b3GetStatusType(status_handle);
            if status_type == CMD_USER_DEBUG_DRAW_COMPLETED as i32 {
                let debug_item_unique_id = ffi::b3GetDebugItemUniqueId(status_handle);
                return Ok(BodyId(debug_item_unique_id));
            }
            Err(Error::new("Error in addUserDebugParameter."))
        }
    }

    pub fn read_user_debug_parameter(&mut self, item_unique_id: BodyId) -> Result<f64, Error> {
        unsafe {
            let command_handle =
                ffi::b3InitUserDebugReadParameter(self.handle.as_ptr(), item_unique_id.0);
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
            let status_type = ffi::b3GetStatusType(status_handle);
            if status_type == CMD_USER_DEBUG_DRAW_PARAMETER_COMPLETED as i32 {
                let mut param_value = 0.;
                let ok = ffi::b3GetStatusDebugParameterValue(status_handle, &mut param_value);
                if ok != 0 {
                    return Ok(param_value);
                }
            }
            Err(Error::new("Failed to read parameter."))
        }
    }
    pub fn add_user_debug_text<
        'a,
        Text: Into<String>,
        Options: Into<Option<AddDebugTextOptions<'a>>>,
    >(
        &mut self,
        text: Text,
        text_position: &[f64],
        options: Options,
    ) -> Result<BodyId, Error> {
        unsafe {
            let options = options.into().unwrap_or_default();
            let command_handle = ffi::b3InitUserDebugDrawAddText3D(
                self.handle.as_ptr(),
                text.into().as_str().as_ptr(),
                text_position.as_ptr(),
                options.text_color_rgb.as_ptr(),
                options.text_size,
                options.life_time,
            );
            if let Some(parent_object) = options.parent_object_id {
                ffi::b3UserDebugItemSetParentObject(
                    command_handle,
                    parent_object.0,
                    options.parent_link_index.unwrap_or(-1),
                );
            }
            if let Some(text_orientation) = options.text_orientation {
                ffi::b3UserDebugTextSetOrientation(command_handle, text_orientation.as_ptr());
            }
            if let Some(replacement_id) = options.replace_item_id {
                ffi::b3UserDebugItemSetReplaceItemUniqueId(command_handle, replacement_id.0);
            }
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
            let status_type = ffi::b3GetStatusType(status_handle);
            if status_type == CMD_USER_DEBUG_DRAW_COMPLETED as i32 {
                let debug_item_id = BodyId(ffi::b3GetDebugItemUniqueId(status_handle));
                return Ok(debug_item_id);
            }
        }
        Err(Error::new("Error in add_user_debug_text"))
    }
    pub fn remove_user_debug_item(&mut self, id: BodyId) {
        unsafe {
            let command_handle = ffi::b3InitUserDebugDrawRemove(self.handle.as_ptr(), id.0);
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
            let _status_type = ffi::b3GetStatusType(status_handle);
        }
    }
    pub fn remove_all_user_debug_items(&mut self) {
        unsafe {
            let command_handle = ffi::b3InitUserDebugDrawRemoveAll(self.handle.as_ptr());
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
            let _status_type = ffi::b3GetStatusType(status_handle);
        }
    }

    pub fn set_debug_object_color(
        &mut self,
        body: BodyId,
        link_index: i32,
        object_debug_color: Option<&[f64]>,
    ) {
        unsafe {
            let command_handle = ffi::b3InitDebugDrawingCommand(self.handle.as_ptr());

            if let Some(color) = object_debug_color {
                ffi::b3SetDebugObjectColor(command_handle, body.0, link_index, color.as_ptr());
            } else {
                ffi::b3RemoveDebugObjectColor(command_handle, body.0, link_index);
            }
            ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
        }
    }

    pub fn get_keyboard_events(&mut self) -> Vec<KeyboardEvent> {
        unsafe {
            let mut keyboard_events = b3KeyboardEventsData::default();
            let command_handle = ffi::b3RequestKeyboardEventsCommandInit(self.handle.as_ptr());
            ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
            ffi::b3GetKeyboardEventsData(self.handle.as_ptr(), &mut keyboard_events);
            let mut events =
                Vec::<KeyboardEvent>::with_capacity(keyboard_events.m_numKeyboardEvents as usize);
            let data = std::slice::from_raw_parts_mut(
                keyboard_events.m_keyboardEvents,
                keyboard_events.m_numKeyboardEvents as usize,
            );
            for &event in data.iter() {
                events.push(KeyboardEvent {
                    key: std::char::from_u32_unchecked(event.m_keyCode as u32),
                    key_state: event.m_keyState,
                });
            }
            events
        }
    }
    pub fn get_mouse_events(&mut self) -> Vec<MouseEvent> {
        unsafe {
            let mut mouse_events = b3MouseEventsData::default();
            let command_handle = ffi::b3RequestMouseEventsCommandInit(self.handle.as_ptr());
            ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
            ffi::b3GetMouseEventsData(self.handle.as_ptr(), &mut mouse_events);
            let mut events =
                Vec::<MouseEvent>::with_capacity(mouse_events.m_numMouseEvents as usize);
            let data = std::slice::from_raw_parts_mut(
                mouse_events.m_mouseEvents,
                mouse_events.m_numMouseEvents as usize,
            );
            for &event in data.iter() {
                if event.m_eventType == 1 {
                    events.push(MouseEvent::Move {
                        mouse_pos_x: event.m_mousePosX,
                        mouse_pos_y: event.m_mousePosY,
                    });
                } else if event.m_eventType == 2 {
                    events.push(MouseEvent::Button {
                        mouse_pos_x: event.m_mousePosX,
                        mouse_pos_y: event.m_mousePosY,
                        button_index: event.m_buttonIndex,
                        button_state_flag: event.m_buttonState,
                    });
                }
            }
            events
        }
    }

    pub fn apply_external_force(
        &mut self,
        body: BodyId,
        link_index: i32,
        force_object: &[f64],
        position_object: &[f64],
        flags: ExternalForceFrame,
    ) -> Result<(), Error> {
        if force_object.len() != 3 {
            return Err(Error::new("force object has wrong length"));
        }
        if position_object.len() != 3 {
            return Err(Error::new("position object has wrong length"));
        }
        unsafe {
            let command = ffi::b3ApplyExternalForceCommandInit(self.handle.as_ptr());
            ffi::b3ApplyExternalForce(
                command,
                body.0,
                link_index,
                force_object.as_ptr(),
                position_object.as_ptr(),
                flags as i32,
            );
            let _status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
        }
        Ok(())
    }
    pub fn apply_external_torque(
        &mut self,
        body: BodyId,
        link_index: i32,
        torque_object: &[f64],
        flags: ExternalForceFrame,
    ) -> Result<(), Error> {
        if torque_object.len() != 3 {
            return Err(Error::new("torque object has wrong length"));
        }
        unsafe {
            let command = ffi::b3ApplyExternalForceCommandInit(self.handle.as_ptr());
            ffi::b3ApplyExternalTorque(
                command,
                body.0,
                link_index,
                torque_object.as_ptr(),
                flags as i32,
            );
            let _status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
        }
        Ok(())
    }
    pub fn enable_joint_torque_sensor(
        &mut self,
        body: BodyId,
        joint_index: i32,
        enable_sensor: bool,
    ) -> Result<(), Error> {
        unsafe {
            let command_handle = ffi::b3CreateSensorCommandInit(self.handle.as_ptr(), body.0);
            ffi::b3CreateSensorEnable6DofJointForceTorqueSensor(
                command_handle,
                joint_index,
                enable_sensor as i32,
            );
            let status_handle =
                ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
            let status_type = ffi::b3GetStatusType(status_handle);
            if status_type == CMD_CLIENT_COMMAND_COMPLETED as i32 {
                return Ok(());
            }
            Err(Error::new("Error creating sensor."))
        }
    }
}

impl Drop for PhysicsClient {
    fn drop(&mut self) {
        unsafe { ffi::b3DisconnectSharedMemory(self.handle.as_ptr()) }
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
            match GUI_EXISTS.compare_exchange(false, true, Ordering::SeqCst, Ordering::SeqCst) {
                Ok(false) => Ok(GuiMarker { _unused: () }),
                _ => Err(crate::Error::new(
                    "Only one in-process GUI connection allowed",
                )),
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
