use crate::client::marker::GuiMarker;
use crate::{Error, PhysicsClient};
use rubullet_sys as ffi;

/// Choose which type of server you want.
pub enum ServerMode {
    GraphicsMainThread {
        /// port on which the server listens. `None` is for the default port "6667"
        tcp_port: Option<u16>,
    },
    Graphics {
        /// port on which the server listens. `None` is for the default port "6667"
        tcp_port: Option<u16>,
    },
}
/// A PhysicsServer is actually a PhysicsClient which is run as a TCP Server and with
/// all methods apart from [`is_connected`](`Self::is_connected`) disabled,
/// as they would fail otherwise. This is for advanced users and you are usually you should be
/// better of with a normal [`PhysicsClient`](`crate::PhysicsClient`).
pub struct PhysicsServer(pub(crate) PhysicsClient);
impl PhysicsServer {
    /// creates a new TCP Server. The GUI is rendered locally but everything else happens on the client.
    /// # Example
    /// ```no_run
    /// use anyhow::Result;
    /// use rubullet::*;
    /// use std::time::Duration;
    ///
    /// fn main() -> Result<()> {
    ///     let mut physics_server = PhysicsServer::new(ServerMode::Graphics { tcp_port: None })?;
    ///     while physics_server.is_connected() {
    ///         std::thread::sleep(Duration::from_secs_f64(1. / 240.));
    ///     }
    ///     Ok(())
    /// }
    /// ```
    pub fn new(mode: ServerMode) -> Result<PhysicsServer, Error> {
        let (raw_handle, _gui_marker) = match mode {
            ServerMode::GraphicsMainThread { tcp_port } => {
                let tcp_port = tcp_port.unwrap_or(6667);
                // Only one GUI is allowed per process. Try to get the marker and fail if there is
                // another.
                let gui_marker = GuiMarker::acquire()?;

                unsafe {
                    let raw_handle =
                        ffi::b3CreateInProcessGraphicsServerAndConnectMainThreadSharedMemory(
                            tcp_port as i32,
                        );
                    (raw_handle, Some(gui_marker))
                }
            }
            ServerMode::Graphics { tcp_port } => {
                let tcp_port = tcp_port.unwrap_or(6667);
                // Only one GUI is allowed per process. Try to get the marker and fail if there is
                // another.
                let gui_marker = GuiMarker::acquire()?;

                let raw_handle = if cfg!(target_os = "macos") {
                    unsafe {
                        ffi::b3CreateInProcessGraphicsServerAndConnectMainThreadSharedMemory(
                            tcp_port as i32,
                        )
                    }
                } else {
                    unsafe {
                        ffi::b3CreateInProcessGraphicsServerAndConnectSharedMemory(tcp_port as i32)
                    }
                };

                (raw_handle, Some(gui_marker))
            }
        };
        let handle = raw_handle.expect("Bullet returned a null pointer");

        // At this point, we need to disconnect the physics client at any error. So we create the
        // Rust struct and allow the `Drop` implementation to take care of that.
        let mut client = PhysicsClient {
            handle,
            _gui_marker,
            _shared_memory_marker: None,
        };

        //Make sure it is up and running.
        if !client.can_submit_command() {
            return Err(Error::new("Physics server is not running"));
        }

        // The client is up and running
        Ok(PhysicsServer(client))
    }
    pub fn is_connected(&mut self) -> bool {
        self.0.is_connected()
    }
}
