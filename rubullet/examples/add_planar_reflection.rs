use anyhow::Result;
use nalgebra::{Isometry3, Vector3};
use rubullet::*;
use std::time::Duration;

fn get_ray_from_to(
    camera_info: DebugVisualizerCameraInfo,
    mouse_x: f32,
    mouse_y: f32,
) -> (Vector3<f64>, Vector3<f64>) {
    println!("{:?}", camera_info);
    let cam_pos: Vector3<f32> = camera_info.target - camera_info.dist * camera_info.camera_forward;
    let far_plane = 10000.;

    let ray_forward: Vector3<f32> = camera_info.target - cam_pos;
    let inv_len = far_plane / ray_forward.norm();
    let ray_forward = ray_forward * inv_len;

    let ray_from = cam_pos;

    let one_over_width = 1. / camera_info.width as f32;
    let one_over_height = 1. / camera_info.height as f32;

    let d_hor = camera_info.horizontal * one_over_width;
    let d_ver = camera_info.vertical * one_over_height;

    let ray_to_center = ray_from + ray_forward;
    let ray_to: Vector3<f32> = ray_to_center - 0.5 * camera_info.horizontal
        + 0.5 * camera_info.vertical
        + Vector3::new(mouse_x, mouse_x, mouse_x).component_mul(&d_hor)
        - Vector3::new(mouse_y, mouse_y, mouse_y).component_mul(&d_ver);

    (
        Vector3::new(ray_from.x as f64, ray_from.y as f64, ray_from.z as f64),
        Vector3::new(ray_to.x as f64, ray_to.y as f64, ray_to.z as f64),
    )
}

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    physics_client.configure_debug_visualizer(DebugVisualizerFlag::CovEnableGui, false);
    physics_client.set_physics_engine_parameter(SetPhysicsEngineParameterOptions {
        num_solver_iterations: Some(10),
        ..Default::default()
    });
    physics_client.set_time_step(Duration::from_secs_f64(1. / 120.));
    let log_id = physics_client.start_state_logging(
        LoggingType::ProfileTimings,
        "visualShapeBench.json",
        None,
    )?;

    let plane = physics_client.load_urdf(
        "plane_transparent.urdf",
        UrdfOptions {
            use_maximal_coordinates: Some(true),
            ..Default::default()
        },
    )?;
    physics_client.configure_debug_visualizer(DebugVisualizerFlag::CovEnableRendering, false);
    physics_client.configure_debug_visualizer(DebugVisualizerFlag::CovEnablePlanarReflection, true);
    physics_client.configure_debug_visualizer(DebugVisualizerFlag::CovEnableGui, false);
    physics_client.configure_debug_visualizer(DebugVisualizerFlag::CovEnableTinyRenderer, false);

    let shift = Isometry3::translation(0., -0.02, 0.);
    let mesh_scale = Vector3::new(0.1, 0.1, 0.1);

    let visual_shape_id = physics_client.create_visual_shape(
        GeometricVisualShape::MeshFile {
            filename: "duck.obj".into(),
            mesh_scaling: Some(mesh_scale),
        },
        VisualShapeOptions {
            frame_offset: shift,
            rgba_colors: [1.; 4],
            specular_colors: [0.4, 0.4, 0.],
            flags: None,
        },
    )?;

    let collision_shape_id = physics_client.create_collision_shape(
        GeometricCollisionShape::MeshFile {
            filename: "duck_vhacd.obj".into(),
            mesh_scaling: Some(mesh_scale),
            flags: None,
        },
        shift,
    )?;

    let range_x = 3;
    let range_y = 3;

    for i in 0..range_x {
        for j in 0..range_y {
            physics_client.create_multi_body(
                collision_shape_id,
                visual_shape_id,
                MultiBodyOptions {
                    base_mass: 1.,
                    base_pose: Isometry3::translation(
                        ((-range_x as f64 / 2.) + i as f64) * mesh_scale.x * 2.,
                        ((-range_y as f64 / 2.) + j as f64) * mesh_scale.y * 2.,
                        1.,
                    ),
                    use_maximal_coordinates: true,
                    ..Default::default()
                },
            )?;
        }
    }

    physics_client.configure_debug_visualizer(DebugVisualizerFlag::CovEnableRendering, true);
    physics_client.stop_state_logging(log_id);
    physics_client.set_gravity([0., 0., -10.]);
    physics_client.set_real_time_simulation(true);

    let colors = [
        [1., 0., 0., 1.],
        [0., 1., 0., 1.],
        [0., 0., 1., 1.],
        [1., 1., 1., 1.],
    ];
    let mut current_color = 0;

    loop {
        let mouse_events = physics_client.get_mouse_events();
        for event in mouse_events {
            match event {
                MouseEvent::Move { .. } => {}
                MouseEvent::Button {
                    mouse_pos_x,
                    mouse_pos_y,
                    button_index,
                    button_state,
                } => {
                    if button_state.was_triggered() && button_index == 0 {
                        let (ray_from, ray_to) = get_ray_from_to(
                            physics_client.get_debug_visualizer_camera(),
                            mouse_pos_x,
                            mouse_pos_y,
                        );
                        let ray_info = physics_client.ray_test(ray_from, ray_to, None)?;
                        if let Some(ray) = ray_info {
                            if ray.body_id != plane {
                                physics_client.change_visual_shape(
                                    ray.body_id,
                                    ray.link_index,
                                    ChangeVisualShapeOptions {
                                        rgba_color: Some(colors[current_color]),
                                        ..Default::default()
                                    },
                                )?;
                                current_color = (current_color + 1) % colors.len()
                            }
                        }
                    }
                }
            }
        }
    }
}
