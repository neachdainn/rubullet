use anyhow::Result;
use nalgebra::{Isometry3, Vector3};
use rubullet::*;
use std::time::Duration;

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    physics_client.reset_simulation_with_flags(ResetFlags::DEFORMABLE_WORLD);

    physics_client.set_gravity([0., 0., -10.]);

    let cube_id = physics_client.load_urdf(
        "cube.urdf",
        UrdfOptions {
            base_transform: Isometry3::translation(0., 1., 2.),
            use_maximal_coordinates: Some(true),
            ..Default::default()
        },
    )?;
    let cloth_id = physics_client.load_soft_body(
        "cloth_z_up.obj",
        SoftBodyOptions {
            base_pose: Isometry3::translation(0., 0., 2.),
            scale: Some(0.5),
            mass: Some(1.),
            use_neo_hookean: false,
            use_bending_springs: true,
            use_mass_spring: true,
            spring_elastic_stiffness: 40.,
            spring_damping_stiffness: 0.1,
            spring_damping_all_directions: true,
            use_self_collision: false,
            friction_coeff: 0.5,
            use_face_contact: true,
            ..Default::default()
        },
    )?;

    physics_client.change_visual_shape(
        cloth_id,
        None,
        ChangeVisualShapeOptions {
            flags: Some(VisualShapeFlags::DOUBLE_SIDED),
            ..Default::default()
        },
    )?;

    physics_client.create_soft_body_anchor(cloth_id, 24, None, None, None)?;
    physics_client.create_soft_body_anchor(cloth_id, 20, None, None, None)?;
    physics_client.create_soft_body_anchor(
        cloth_id,
        15,
        cube_id,
        None,
        Vector3::new(0.5, -0.5, 0.),
    )?;
    physics_client.create_soft_body_anchor(
        cloth_id,
        19,
        cube_id,
        None,
        Vector3::new(-0.5, -0.5, 0.),
    )?;
    physics_client.set_physics_engine_parameter(SetPhysicsEngineParameterOptions {
        sparse_sdf_voxel_size: Some(0.25),
        ..Default::default()
    });

    physics_client.set_gravity([0., 0., -10.]);
    loop {
        std::thread::sleep(Duration::from_secs_f64(1. / 240.));
        physics_client.step_simulation()?;
        // physics_client.set_gravity([0., 0., -10.]);
    }
}
