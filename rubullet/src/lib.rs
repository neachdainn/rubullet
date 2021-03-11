//! A Rust interface for Bullet physics inspired by PyBullet.
//!
//! # Example
//! ```no_run
//! use std::{thread, time::Duration};
//!
//! use anyhow::Result;
//! use nalgebra::{Isometry3, Vector3};
//! use rubullet::*;
//!
//! fn main() -> Result<()> {
//!     let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
//!
//!     physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
//!     physics_client.set_gravity(Vector3::new(0.0, 0.0, -10.0));
//!
//!     let _plane_id = physics_client.load_urdf("plane.urdf", None)?;
//!
//!     let cube_start_position = Isometry3::translation(0.0, 0.0, 1.0);
//!     let box_id = physics_client.load_urdf(
//!         "r2d2.urdf",
//!         UrdfOptions {
//!             base_transform: cube_start_position,
//!             ..Default::default()
//!         },
//!     )?;
//!
//!     for _ in 0..10000 {
//!         physics_client.step_simulation()?;
//!         thread::sleep(Duration::from_micros(4167));
//!     }
//!
//!     let cube_transform = physics_client.get_base_transform(box_id)?;
//!     println!("{}", cube_transform);
//!
//!     Ok(())
//! }
//! ```
pub use crate::{
    client::PhysicsClient,
    error::Error,
    mode::Mode,
    types::{
        Aabb, ActivationState, AddDebugLineOptions, AddDebugTextOptions, BodyId, BodyInfo,
        BodyType, ChangeConstraintOptions, ChangeDynamicsOptions, ChangeVisualShapeOptions,
        CollisionId, ConstraintId, ConstraintInfo, ConstraintSolverType, ContactPoint, ControlMode,
        ControlModeArray, DebugVisualizerCameraInfo, DebugVisualizerFlag, DynamicsInfo,
        ExternalForceFrame, GeometricCollisionShape, GeometricVisualShape, IkSolver, Images,
        InverseKinematicsNullSpaceParameters, InverseKinematicsParameters,
        InverseKinematicsParametersBuilder, ItemId, Jacobian, JointFeedbackMode, JointInfo,
        JointInfoFlags, JointState, JointType, KeyboardEvent, LinkState, LoadModelFlags, LogFlags,
        LogId, LoggingType, MouseButtonState, MouseEvent, MultiBodyOptions, OverlappingObject,
        PhysicsEngineParameters, RayHitInfo, RayTestBatchOptions, RayTestOptions, SdfOptions,
        SetPhysicsEngineParameterOptions, StateId, StateLoggingOptions, TextureId, UrdfOptions,
        Velocity, VisualId, VisualShapeData, VisualShapeOptions,
    },
};
pub use image;
pub use nalgebra;
mod client;
mod error;
pub mod logging_utils;
mod mode;
mod types;
