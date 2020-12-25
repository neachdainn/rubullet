//! A Rust interface for Bullet physics inspired by PyBullet.
#![allow(dead_code)]

pub use crate::{
    client::{BodyId, ControlMode, DebugVisualizerFlag, JointType, PhysicsClient, UrdfOptions},
    error::Error,
    mode::Mode,
};
/// A utility for creating C-string literals.
/*
macro_rules! cstr
{
    ($lit:expr) => {
        concat!($lit, "\0") as *const str
            as *const [::std::os::raw::c_char]
            as *const ::std::os::raw::c_char
    }
}
*/
pub mod client;
pub mod error;
pub mod mode;
