//! A Rust interface for Bullet physics inspired by PyBullet.
#![allow(dead_code)]

pub use crate::{
    client::PhysicsClient,
    error::Error,
    mode::Mode,
    types::{
        BodyId, ControlMode, ControlModeArray, DebugVisualizerFlag,
        InverseKinematicsNullSpaceParameters, InverseKinematicsParametersBuilder, JointInfo,
        JointState, JointType, UrdfOptions,
    },
};
// A utility for creating C-string literals.
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
pub mod types;
