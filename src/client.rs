//! The main physics client.
//!
//! This is largely modeled after the PyBullet API but utilizes Rust's more expressive type system
//! where available.
use std::ptr;

use crate::{Error, Mode, ffi};

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
		let client = PhysicsClient { handle, gui_marker };

		// Make sure it is up and running.
		if unsafe { ffi::b3CanSubmitCommand(handle.as_ptr()) } == 0 {
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
}

impl Drop for PhysicsClient
{
	fn drop(&mut self)
	{
		unsafe { ffi::b3DisconnectSharedMemory(self.handle.as_ptr()) }
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
