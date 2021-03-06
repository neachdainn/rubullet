![GitHub Workflow Status](https://img.shields.io/github/workflow/status/neachdainn/rubullet/Rust)
[![crates.io](https://img.shields.io/crates/v/rubullet.svg)](https://crates.io/crates/rubullet)
[![crates.io](https://img.shields.io/crates/l/rubullet.svg)](https://crates.io/crates/rubullet)
[![crates.io](https://img.shields.io/crates/d/rubullet.svg)](https://crates.io/crates/rubullet)
[![docs.rs](https://docs.rs/rubullet/badge.svg)](https://docs.rs/rubullet)
# RuBullet

RuBullet is a Rust implementation of [PyBullet](https://pybullet.org/).
In other words, it uses the [Bullet3](https://github.com/bulletphysics/bullet3) 
C API in order to expose a functionality that is similar to PyBullet.
Development is ongoing and functionality is currently limited.
![](https://i.imgur.com/UonCX5F.png)
## Status
Right now RuBullet should cover most of the basic use cases. It can:
* Create a PhysicsClient in Direct, Gui or other modes
* Load models from URDF, SDF, MuJoCo or Bullet files
* Create own models within the simulation
* Control robots in position, velocity or torque mode
* Calculate inverse dynamics, inverse kinematics, jacobians and mass matrices
* Render camera images
* Read information about links and joints
* Change dynamics of a body
* Create GUI sliders, buttons or put debugging text or lines in the simulation
* Get keyboard and mouse events 
* Create and manage constraints
* Logging
* Saving states and loading states
* Set physics engine parameters
* Collision Detection Queries
* Deformables and Cloth

Things which are not implemented yet:
* Everything MultiDOF related
* Virtual Reality
* Plugins

The API is unstable and subject to change.

# Example
```rust
use std::{thread, time::Duration};

use anyhow::Result;
use nalgebra::{Isometry3, Vector3};
use rubullet::*;

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;

    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    physics_client.set_gravity(Vector3::new(0.0, 0.0, -10.0));

    let _plane_id = physics_client.load_urdf("plane.urdf", Default::default())?;

    let cube_start_position = Isometry3::translation(0.0, 0.0, 1.0);
    let box_id = physics_client.load_urdf(
        "r2d2.urdf",
        UrdfOptions {
            base_transform: cube_start_position,
            ..Default::default()
        },
    )?;

    for _ in 0..10000 {
        physics_client.step_simulation()?;
        thread::sleep(Duration::from_micros(4167));
    }

    let cube_transform = physics_client.get_base_transform(box_id)?;
    println!("{}", cube_transform);

    Ok(())
}
```

## Bug reports and Merge Requests
The current development happens as a part of marcbone's master thesis. Therefore, merge requests can not be accepted until
July 5, 2021. We are disabling merge requests until then which sadly also disables issues. If you find any bugs or have suggestions please write an email to
one of the maintainers.

## License
RuBullet is licensed under MIT

