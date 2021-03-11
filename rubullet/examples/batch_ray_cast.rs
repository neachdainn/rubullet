use anyhow::Result;
use nalgebra::{Isometry3, Vector3};
use rubullet::*;
use std::f64::consts::PI;

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    physics_client.configure_debug_visualizer(DebugVisualizerFlag::CovEnableGui, false);

    // physics_client.load_urdf("r2d2.urdf", None)?;
    let _r2d2 = physics_client.load_urdf(
        "r2d2.urdf",
        UrdfOptions {
            base_transform: Isometry3::translation(3., 3., 1.),
            ..Default::default()
        },
    )?;

    let num_rays = 104;
    let ray_len = 13.;

    let mut ray_from: Vec<Vector3<f64>> = Vec::with_capacity(num_rays);
    let mut ray_to: Vec<Vector3<f64>> = Vec::with_capacity(num_rays);
    let mut ray_ids: Vec<ItemId> = Vec::with_capacity(num_rays);

    let ray_hit_color = [1., 0., 0.];
    let ray_miss_color = [0., 1., 0.];
    let ray_default_color = [0., 0., 1.];

    let replace_lines = true;

    for i in 0..num_rays {
        ray_from.push(Vector3::new(0., 0., 1.));
        let ray = [
            ray_len * f64::sin(2. * PI * i as f64 / num_rays as f64),
            ray_len * f64::cos(2. * PI * i as f64 / num_rays as f64),
            1.,
        ];

        ray_to.push(ray.into());
        if replace_lines {
            ray_ids.push(
                physics_client
                    .add_user_debug_line(
                        ray_from[i],
                        ray_to[i],
                        AddDebugLineOptions {
                            line_color_rgb: ray_default_color,
                            ..Default::default()
                        },
                    )
                    .unwrap(),
            );
        }
    }
    let num_steps = 327680;
    for _ in 0..num_steps {
        physics_client.step_simulation()?;
        let results = physics_client.ray_test_batch(
            &ray_from,
            &ray_to,
            RayTestBatchOptions {
                num_threads: Some(4),
                ..Default::default()
            },
        )?;
        if !replace_lines {
            physics_client.remove_all_user_debug_items();
        }
        for i in 0..num_rays {
            if let Some(result) = results[i] {
                physics_client.add_user_debug_line(
                    ray_from[i],
                    result.hit_position,
                    AddDebugLineOptions {
                        line_color_rgb: ray_hit_color,
                        replace_item_id: Some(ray_ids[i]),
                        ..Default::default()
                    },
                )?;
            } else {
                physics_client.add_user_debug_line(
                    ray_from[i],
                    ray_to[i],
                    AddDebugLineOptions {
                        line_color_rgb: ray_miss_color,
                        replace_item_id: Some(ray_ids[i]),
                        ..Default::default()
                    },
                )?;
            }
        }
    }
    Ok(())
}
