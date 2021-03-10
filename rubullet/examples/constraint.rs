use anyhow::Result;
use nalgebra::{Isometry3, UnitQuaternion, Vector3};
use rubullet::*;
use std::f64::consts::PI;
use std::time::Duration;

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    physics_client.load_urdf("plane.urdf", None)?;
    let cube_id = physics_client.load_urdf(
        "cube_small.urdf",
        UrdfOptions {
            base_transform: Isometry3::translation(0., 0., 1.),
            ..Default::default()
        },
    )?;
    physics_client.set_gravity([0., 0., -10.]);
    physics_client.set_real_time_simulation(true);

    let cid = physics_client.create_constraint(
        cube_id,
        None,
        None,
        None,
        JointType::Fixed,
        [0.; 3],
        Isometry3::identity(),
        Isometry3::translation(0., 0., 1.),
    )?;
    println!("{:?}", cid);
    println!("{:?}", physics_client.get_constraint(0)?);
    let mut a = -PI;
    loop {
        a += 0.01;
        if a > PI {
            a -= PI;
        }
        std::thread::sleep(Duration::from_secs_f64(0.01));
        let change_constraint_options = ChangeConstraintOptions {
            joint_child_pivot: Some(Vector3::new(a, 0., 1.)),
            joint_child_frame_orientation: Some(UnitQuaternion::from_euler_angles(a, 0., 0.)),
            max_force: Some(50.),
            ..Default::default()
        };
        physics_client.change_constraint(cid, change_constraint_options);
    }
}
