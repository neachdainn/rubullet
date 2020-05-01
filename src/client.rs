//! The main physics client.
//!
//! This is largely modeled after the PyBullet API but utilizes Rust's more expressive type system
//! where available.
use std::{ffi::CString, path::Path, os::raw::c_int, ptr};

// I currently do not know the best way to represent the file operations for Windows. PyBullet uses
// raw C-strings but that doesn't seem appropriate here. I don't really have a Windows machine, so
// until then...
use std::os::unix::ffi::OsStrExt;

use crate::{Error, Mode, ffi};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};

/// The "handle" to the physics client.
///
/// For whatever reason, the Bullet C API obfuscates the handle type by defining the handle type as
/// a pointer to an (essentially) anonymous struct. That's ugly to use here, and we know it isn't
/// going to be null, so we'll just do this alias.
type Handle = std::ptr::NonNull<ffi::b3PhysicsClientHandle__>;

/// Connection to a physics server.
///
/// This serves as an abstraction over the possible physics servers, providing a unified interface.
pub struct PhysicsClient
{
	/// The underlying `b3PhysicsClientHandle` that is guaranteed to not be null.
	handle: Handle,

	/// A marker indicating whether or not a GUI is in use by this client.
	gui_marker: Option<GuiMarker>,
}

impl PhysicsClient
{
	pub fn connect(mode: Mode) -> Result<PhysicsClient, Error>
	{
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
					unsafe { ffi::b3CreateInProcessPhysicsServerAndConnectMainThread(0, ptr::null_mut()) }
				} else {
					unsafe { ffi::b3CreateInProcessPhysicsServerAndConnect(0, ptr::null_mut()) }
				};

				(raw_handle, Some(gui_marker))
			}
		};

		// Make sure the returned pointer is valid.
		let handle = Handle::new(raw_handle).ok_or(Error::new("Bullet returned a null handle"))?;

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
			let status_handle = ffi::b3SubmitClientCommandAndWaitStatus(client.handle.as_ptr(), command);
			let status_type = ffi::b3GetStatusType(status_handle);

			if status_type != ffi::EnumSharedMemoryServerStatus::CMD_SYNC_BODY_INFO_COMPLETED as _ {
				return Err(Error::new("Connection terminated, couldn't get body info"));
			}

			let command = ffi::b3InitSyncUserDataCommand(client.handle.as_ptr());
			let status_handle = ffi::b3SubmitClientCommandAndWaitStatus(client.handle.as_ptr(), command);
			let status_type = ffi::b3GetStatusType(status_handle);

			if status_type != ffi::EnumSharedMemoryServerStatus::CMD_SYNC_USER_DATA_COMPLETED as _ {
				return Err(Error::new("Connection terminated, couldn't get user data"));
			}
		}

		// The client is up and running
		Ok(client)
	}

	/// Sets an additional search path for loading assests.
	pub fn set_additional_search_path<P: AsRef<Path>>(&mut self, path: P) -> Result<(), Error>
	{
		if !self.can_submit_command() {
			return Err(Error::new("Not connected to physics server"));
		}

		let path = CString::new(path.as_ref().as_os_str().as_bytes())
			.map_err(|_| Error::new("Invalid path"))?;

		unsafe {
			// Based on PyBullet, it appears that this path is copied and it does not need to live
			// after calling the function.
			let command_handle = ffi::b3SetAdditionalSearchPath(self.handle.as_ptr(), path.as_ptr());
			let _status_handle = ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command_handle);
		}

		Ok(())
	}

	/// Sets the default gravity force for all objects.
	///
	/// By default, there is no gravitational force enabled.
	pub fn set_gravity(&mut self, gravity: Vector3<f64>) -> Result<(), Error>
	{
		if !self.can_submit_command() {
			return Err(Error::new("Not connected to physics server"));
		}

		unsafe {
			// PyBullet error checks none of these. Looking through the code, it looks like there is
			// no possible way to return an error on them.
			let command = ffi::b3InitPhysicsParamCommand(self.handle.as_ptr());
			let _ret = ffi::b3PhysicsParamSetGravity(command, gravity.x, gravity.y, gravity.z);
			let _status_handle = ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
		}

		Ok(())
	}

	/// Sends a command to the physics server to load a physics model from a Universal Robot
	/// Description File (URDF).
	pub fn load_urdf<P: AsRef<Path>>(&mut self, file: P, options: UrdfOptions) -> Result<BodyId, Error>
	{
		if !self.can_submit_command() {
			return Err(Error::new("Not connected to physics server"));
		}

		let file = CString::new(file.as_ref().as_os_str().as_bytes())
			.map_err(|_| Error::new("Invalid path"))?;

		// There's probably a cleaner way to do it. The one-liner I came up with was neat but bad
		// code.
		let mut flags = 0;
		if options.use_inertia_from_file { flags |= ffi::eURDF_Flags::URDF_USE_INERTIA_FROM_FILE as c_int; }
		if options.use_self_collision { flags |= ffi::eURDF_Flags::URDF_USE_SELF_COLLISION as c_int; }
		if options.enable_sleeping { flags |= ffi::eURDF_Flags::URDF_ENABLE_SLEEPING as c_int; }
		if options.maintain_link_order { flags |= ffi::eURDF_Flags::URDF_MAINTAIN_LINK_ORDER as c_int; }

		unsafe {
			// As always, PyBullet does not document and does not check return codes.
			let command = ffi::b3LoadUrdfCommandInit(self.handle.as_ptr(), file.as_ptr());
			let _ret = ffi::b3LoadUrdfCommandSetFlags(command, flags);
			let _ret = ffi::b3LoadUrdfCommandSetStartPosition(command, options.base_transform.translation.x, options.base_transform.translation.y, options.base_transform.translation.z);
			let _ret = ffi::b3LoadUrdfCommandSetStartOrientation(command, options.base_transform.rotation.coords.x, options.base_transform.rotation.coords.y, options.base_transform.rotation.coords.z, options.base_transform.rotation.coords.w);

			if options.use_fixed_base {
				let _ret = ffi::b3LoadUrdfCommandSetUseFixedBase(command, 1);
			}

			if options.global_scaling != 1.0 && options.global_scaling > 0.0 {
				let _ret = ffi::b3LoadUrdfCommandSetGlobalScaling(command, options.global_scaling as f64);
			}

			let status_handle = ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
			let status_type = ffi::b3GetStatusType(status_handle);
			if status_type != ffi::EnumSharedMemoryServerStatus::CMD_URDF_LOADING_COMPLETED as c_int {
				return Err(Error::new("Cannot load URDF file"));
			}

			Ok(BodyId(ffi::b3GetStatusBodyIndex(status_handle)))
		}
	}

	/// Performs all the actions in a single forward dynamics simulation step such as collision
	/// detection, constraint solving, and integration.
	// TODO: Mention changing step size and automatic steps.
	// TODO: Return analytics data?
	pub fn step_simulation(&mut self) -> Result<(), Error>
	{
		if !self.can_submit_command() {
			return Err(Error::new("Not connected to physics server"));
		}

		unsafe {
			let command = ffi::b3InitStepSimulationCommand(self.handle.as_ptr());
			let status_handle = ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), command);
			let status_type = ffi::b3GetStatusType(status_handle);

			if status_type != ffi::EnumSharedMemoryServerStatus::CMD_STEP_FORWARD_SIMULATION_COMPLETED as i32 {
				return Err(Error::new("Failed to perform forward step"));
			}
		}

		Ok(())
	}

	/// Reports the current transform of the base.
	pub fn get_base_transform(&mut self, body: BodyId) -> Result<Isometry3<f64>, Error>
	{
		if !self.can_submit_command() {
			return Err(Error::new("Not connected to physics server"));
		}

		unsafe {
			let cmd_handle = ffi::b3RequestActualStateCommandInit(self.handle.as_ptr(), body.0);
			let status_handle = ffi::b3SubmitClientCommandAndWaitStatus(self.handle.as_ptr(), cmd_handle);
			let status_type = ffi::b3GetStatusType(status_handle);

			if status_type != ffi::EnumSharedMemoryServerStatus::CMD_ACTUAL_STATE_UPDATE_COMPLETED as c_int {
				return Err(Error::new("Failed to get base transform"));
			}

			// To be totally honest, I'm not sure this part is correct.
			let actual_state_q: *mut f64 = ptr::null_mut();
			ffi::b3GetStatusActualState(status_handle,
				ptr::null_mut(), ptr::null_mut(), ptr::null_mut(),
				ptr::null_mut(), &actual_state_q as _, ptr::null_mut(), ptr::null_mut()
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

			Ok(Isometry3::from_parts(tra, UnitQuaternion::from_quaternion(rot)))
		}
	}

	/// Returns whether or not this client can submit a command.
	fn can_submit_command(&mut self) -> bool
	{
		unsafe { ffi::b3CanSubmitCommand(self.handle.as_ptr()) != 0 }
	}
}

impl Drop for PhysicsClient
{
	fn drop(&mut self)
	{
		unsafe { ffi::b3DisconnectSharedMemory(self.handle.as_ptr()) }
	}
}

/// The unique ID for a body within a physics server.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Hash)]
pub struct BodyId(c_int);

/// Options for loading a URDF into the physics server.
pub struct UrdfOptions
{
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

	/// Applies a scale factor to the model.
	pub global_scaling: f64,

	// Future proofs the struct. Unfortunately, `#[non_exhaustive]` doesn't apply to structs.
	#[doc(hidden)]
	pub _unused: (),
}

impl Default for UrdfOptions
{
	fn default() -> UrdfOptions
	{
		UrdfOptions {
			base_transform: Isometry3::identity(),
			use_fixed_base: false,
			use_inertia_from_file: false,
			use_self_collision: false,
			enable_sleeping: false,
			maintain_link_order: false,
			global_scaling: 1.0,

			_unused: (),
		}
	}
}

/// Module used to enforce the existence of only a single GUI
mod gui_marker
{
	use std::sync::atomic::{AtomicBool, Ordering};

	/// A marker for whether or not a GUI has been started.
	///
	/// PyBullet only allows a single GUI per process. I assume that this is a limitation of the
	/// underlying C API, so it will also be enforced here.
	static GUI_EXISTS: AtomicBool = AtomicBool::new(false);

	/// A marker type for keeping track of the existence of a GUI.
	pub struct GuiMarker { _unused: () }

	impl GuiMarker
	{
		/// Attempts to acquire the GUI marker.
		pub fn acquire() -> Result<GuiMarker, crate::Error>
		{
			// We can probably use a weaker ordering but this will be called so little that we
			// may as well be sure about it.
			if GUI_EXISTS.compare_and_swap(false, true, Ordering::SeqCst) {
				Err(crate::Error::new("Only one in-process GUI connection allowed"))
			} else {
				Ok(GuiMarker { _unused: () })
			}
		}
	}

	impl Drop for GuiMarker
	{
		fn drop(&mut self)
		{
			// We are the only marker so no need to CAS
			GUI_EXISTS.store(false, Ordering::SeqCst)
		}
	}
}
use self::gui_marker::GuiMarker;
