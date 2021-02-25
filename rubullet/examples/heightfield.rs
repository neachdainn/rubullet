//! An introduction to the usage of RuBullet.
use std::time::Duration;

use anyhow::Result;
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use rand::prelude::*;
use rubullet::types::DebugVisualizerFlag::CovEnableRendering;
use rubullet::*;

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;

    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    physics_client.configure_debug_visualizer(CovEnableRendering, false);
    let height_pertubation_range = 0.05;
    let mut rng = thread_rng();
    let num_heightfield_rows = 256;
    let num_heightfield_columns = 256;
    let mut heightfield_data = vec![0_f32; num_heightfield_rows * num_heightfield_columns];
    for j in 0..num_heightfield_columns / 2 {
        for i in 0..num_heightfield_rows / 2 {
            let height: f32 = rng.gen_range((0.)..height_pertubation_range);
            heightfield_data[2 * i + 2 * j * num_heightfield_rows] = height;
            heightfield_data[2 * i + 1 + 2 * j * num_heightfield_rows] = height;
            heightfield_data[2 * i + (2 * j + 1) * num_heightfield_rows] = height;
            heightfield_data[2 * i + 1 + (2 * j + 1) * num_heightfield_rows] = height;
        }
    }
    let terrain_shape = physics_client.create_collision_shape(
        GeometricCollisionShape::Heightfield {
            mesh_scale: [0.05, 0.05, 1.],
            texture_scaling: (num_heightfield_rows - 1) as f64 / 2.,
            data: heightfield_data,
            num_rows: num_heightfield_rows as i32,
            num_columns: num_heightfield_columns as i32,
            replace_heightfield: None,
        },
        Isometry3::identity(),
    )?;

    let terrain = physics_client.create_multi_body(
        terrain_shape,
        VisualId(0),
        MultiBodyOptions::default(),
    )?;

    physics_client.change_visual_shape(
        terrain,
        None,
        ChangeVisualShapeOptions {
            rgba_color: Some([1.; 4]),
            ..Default::default()
        },
    )?;

    let sphere_radius = 0.05;

    let col_sphere_id = physics_client.create_collision_shape(
        GeometricCollisionShape::Sphere {
            radius: sphere_radius,
        },
        Isometry3::identity(),
    )?;
    let col_box_id = physics_client.create_collision_shape(
        GeometricCollisionShape::Box {
            half_extents: Vector3::from_element(sphere_radius),
        },
        Isometry3::identity(),
    )?;
    let mass = 1.;
    let visual_shape_id = VisualId(-1);

    for i in 0..3 {
        for j in 0..3 {
            for k in 0..3 {
                let base_pose = Isometry3::from_parts(
                    Translation3::new(
                        i as f64 * 5. * sphere_radius,
                        j as f64 * 5. * sphere_radius,
                        1. + k as f64 * 5. * sphere_radius + 1.,
                    ),
                    UnitQuaternion::identity(),
                );
                let sphere_uid = {
                    if k & 2 != 0 {
                        physics_client.create_multi_body(
                            col_sphere_id,
                            visual_shape_id,
                            MultiBodyOptions {
                                base_mass: mass,
                                base_pose,
                                ..Default::default()
                            },
                        )?
                    } else {
                        let link_masses = vec![1.];
                        let link_collision_shapes = vec![col_box_id];
                        let link_visual_shapes = vec![VisualId(-1)];
                        let link_poses = vec![Isometry3::translation(0., 0., 0.11)];
                        let link_inertial_frame_poses = vec![Isometry3::translation(0., 0., 0.)];
                        let indices = vec![0];
                        let joint_types = vec![JointType::Revolute];
                        let axis = vec![Vector3::new(0., 0., 1.)];

                        physics_client.create_multi_body(
                            col_box_id,
                            visual_shape_id,
                            MultiBodyOptions {
                                base_mass: mass,
                                base_pose,
                                link_masses,
                                link_collision_shapes,
                                link_visual_shapes,
                                link_poses,
                                link_inertial_frame_poses,
                                link_parent_indices: indices,
                                link_joint_types: joint_types,
                                link_joint_axis: axis,
                                ..Default::default()
                            },
                        )?
                    }
                };

                for joint in 0..physics_client.get_num_joints(sphere_uid) {
                    physics_client.set_joint_motor_control(
                        sphere_uid,
                        joint,
                        ControlMode::Velocity(1.),
                        Some(10.),
                    );
                }
            }
        }
    }
    physics_client.configure_debug_visualizer(CovEnableRendering, true);
    physics_client.set_gravity(Vector3::new(0.0, 0.0, -10.0))?;
    physics_client.set_real_time_simulation(true);

    loop {
        std::thread::sleep(Duration::from_secs_f64(0.01));
    }
}
