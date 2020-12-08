//! Various connection modes used by Bullet.
//!
//! A better API than this exists for dealing with the variations on clients, servers, and GUIs.
//! However, I really only plan on using the `Direct` mode with the occasional `Gui` for
//! verification purposes. If other people start using this, I'll probably revisit the way modes are
//! exposed and will do something better.

/// Ways to connect to physics servers.
#[non_exhaustive]
pub enum Mode {
    /// Starts a physics server within the process and connects without a transport layer.
    Direct,

    /// Creates a physics server with a graphical frontend and communicates with it.
    Gui,
}
