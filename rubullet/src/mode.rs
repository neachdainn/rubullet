//! Various connection modes used by Bullet.

/// Ways to connect to physics clients.
#[non_exhaustive]
pub enum Mode<'a> {
    /// Starts a physics server within the process and connects without a transport layer.
    Direct,

    /// Creates a physics server with a graphical frontend and communicates with it.
    Gui,
    GuiMainThread,
    /// can be used together with sharedMemory
    GuiServer,
    /// can be used together with sharedMemory
    SharedMemoryServer,
    SharedMemoryGui,
    /// connects to a remote [`PhysicsServer`](`crate::PhysicsServer`) via TCP.
    /// See `graphics_server.rs` and `graphics_client.rs` demo
    GraphicsServerTcp {
        /// hostname of the server.
        hostname: &'a str,
        /// port on which the server listens. `None` is for the default port `6667`
        port: Option<u16>,
    },
    /// get access to an already running PhysicsClient which was created with `GuiServer` or `SharedMemoryServer`
    SharedMemory,

    Udp {
        /// hostname of the server.
        hostname: &'a str,
        /// port on which the server listens. `None` is for the default port `1234`
        port: Option<u16>,
    },
    Tcp {
        /// hostname of the server.
        hostname: &'a str,
        /// port on which the server listens. `None` is for the default port `6667`
        port: Option<u16>,
    },
}
