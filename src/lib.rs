//! A Rust interface for Bullet physics inspired by PyBullet.
#![allow(dead_code)]

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

mod client;
mod error;
mod mode;
mod ffi;

pub use crate::{client::{BodyId, PhysicsClient, UrdfOptions}, error::Error, mode::Mode};
