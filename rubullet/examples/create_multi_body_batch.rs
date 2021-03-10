//! An introduction to the usage of RuBullet.
use std::time::Duration;

use anyhow::Result;
use nalgebra::{Isometry3, Point3, Vector3};
use rubullet::DebugVisualizerFlag::{CovEnableGui, CovEnableRendering, CovEnableTinyRenderer};
use rubullet::*;

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;

    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    physics_client.set_time_step(Duration::from_secs_f64(1. / 120.));
    let _plane_id = physics_client.load_urdf("plane100.urdf", None)?;

    physics_client.configure_debug_visualizer(CovEnableRendering, false);
    physics_client.configure_debug_visualizer(CovEnableGui, false);
    physics_client.configure_debug_visualizer(CovEnableTinyRenderer, false);

    let shift = Isometry3::translation(0.0, -0.02, 0.0);
    let mesh_scaling = Vector3::from_element(0.1);

    let vertices = vec![
        [-1., -1., 1.],
        [1., -1., 1.],
        [1., 1., 1.],
        [-1., 1., 1.],
        [-1., -1., -1.],
        [1., -1., -1.],
        [1., 1., -1.],
        [-1., 1., -1.],
        [-1., -1., -1.],
        [-1., 1., -1.],
        [-1., 1., 1.],
        [-1., -1., 1.],
        [1., -1., -1.],
        [1., 1., -1.],
        [1., 1., 1.],
        [1., -1., 1.],
        [-1., -1., -1.],
        [-1., -1., 1.],
        [1., -1., 1.],
        [1., -1., -1.],
        [-1., 1., -1.],
        [-1., 1., 1.],
        [1., 1., 1.],
        [1., 1., -1.],
    ];
    let normals = vec![
        [0., 0., 1.],
        [0., 0., 1.],
        [0., 0., 1.],
        [0., 0., 1.],
        [0., 0., -1.],
        [0., 0., -1.],
        [0., 0., -1.],
        [0., 0., -1.],
        [-1., 0., 0.],
        [-1., 0., 0.],
        [-1., 0., 0.],
        [-1., 0., 0.],
        [1., 0., 0.],
        [1., 0., 0.],
        [1., 0., 0.],
        [1., 0., 0.],
        [0., -1., 0.],
        [0., -1., 0.],
        [0., -1., 0.],
        [0., -1., 0.],
        [0., 1., 0.],
        [0., 1., 0.],
        [0., 1., 0.],
        [0., 1., 0.],
    ];

    let uvs = vec![
        [0.75, 0.25],
        [1., 0.25],
        [1., 0.],
        [0.75, 0.],
        [0.5, 0.25],
        [0.25, 0.25],
        [0.25, 0.],
        [0.5, 0.],
        [0.5, 0.],
        [0.75, 0.],
        [0.75, 0.25],
        [0.5, 0.25],
        [0.25, 0.5],
        [0.25, 0.25],
        [0., 0.25],
        [0., 0.5],
        [0.25, 0.5],
        [0.25, 0.25],
        [0.5, 0.25],
        [0.5, 0.5],
        [0., 0.],
        [0., 0.25],
        [0.25, 0.25],
        [0.25, 0.],
    ];
    let indices = vec![
        0, 1, 2, 0, 2, 3, //ground face
        6, 5, 4, 7, 6, 4, //top face
        10, 9, 8, 11, 10, 8, 12, 13, 14, 12, 14, 15, 18, 17, 16, 19, 18, 16, 20, 21, 22, 20, 22,
        23,
    ];

    let visual_shape_id = physics_client.create_visual_shape(
        GeometricVisualShape::Mesh {
            mesh_scaling: Some(mesh_scaling),
            vertices,
            indices,
            uvs: Some(uvs),
            normals: Some(normals),
        },
        VisualShapeOptions {
            specular_colors: [0.4, 0.4, 0.],
            frame_offset: shift,
            ..Default::default()
        },
    )?;
    let collision_shape_id = physics_client.create_collision_shape(
        GeometricCollisionShape::Box {
            half_extents: mesh_scaling,
        },
        Isometry3::identity(),
    )?;

    let tex_uid = physics_client.load_texture("tex256.png")?;
    let mut batch_positions = Vec::with_capacity(32 * 32 * 10 * 3);
    for x in 0..32 {
        for y in 0..32 {
            for z in 0..10 {
                batch_positions.push(Point3::new(
                    x as f64 * mesh_scaling[0] * 5.5,
                    y as f64 * mesh_scaling[1] * 5.5,
                    (0.5 + z as f64) * mesh_scaling[2] * 2.5,
                ));
            }
        }
    }

    let body_id = physics_client.create_multi_body(
        collision_shape_id,
        visual_shape_id,
        MultiBodyOptions {
            base_pose: Isometry3::translation(0., 0., 2.),
            use_maximal_coordinates: true,

            batch_positions: Some(batch_positions),
            ..Default::default()
        },
    )?;

    physics_client.change_visual_shape(
        body_id,
        None,
        ChangeVisualShapeOptions {
            texture_id: Some(tex_uid),
            ..Default::default()
        },
    )?;
    physics_client.configure_debug_visualizer(CovEnableRendering, true);
    physics_client.set_gravity(Vector3::new(0.0, 0.0, -10.0));

    loop {
        physics_client.step_simulation()?;
    }
}
