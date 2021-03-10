use anyhow::Result;
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use rubullet::*;
use std::f64::consts::TAU;

pub enum ClosestPointsMethod {
    Body,
    BodyShape,
    Shape,
}
const METHOD: ClosestPointsMethod = ClosestPointsMethod::Shape;
fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    physics_client.configure_debug_visualizer(DebugVisualizerFlag::CovEnableGui, false);
    let geom = physics_client.create_collision_shape(
        GeometricCollisionShape::Sphere { radius: 0.1 },
        Isometry3::identity(),
    )?;
    let geom_box = physics_client.create_collision_shape(
        GeometricCollisionShape::Box {
            half_extents: [0.2; 3].into(),
        },
        Isometry3::identity(),
    )?;
    let mut base_pose_b = Isometry3::from_parts(
        Translation3::new(1.5, 0., 1.),
        UnitQuaternion::from_euler_angles(0., 0.3, 0.),
    );
    let ob_a = physics_client.create_multi_body(
        geom,
        VisualId::NONE,
        MultiBodyOptions {
            base_mass: 0.,
            base_pose: Isometry3::translation(0.5, 0., 1.),
            ..Default::default()
        },
    )?;
    let ob_b = physics_client.create_multi_body(
        geom_box,
        VisualId::NONE,
        MultiBodyOptions {
            base_mass: 0.,
            base_pose: base_pose_b,
            ..Default::default()
        },
    )?;

    let line_width = 3.;
    let color_rgb = [1., 0., 0.];

    let line_id = physics_client.add_user_debug_line(
        [0.; 3],
        [0.; 3],
        AddDebugLineOptions {
            line_color_rgb: color_rgb,
            line_width,
            life_time: 0.,
            ..Default::default()
        },
    )?;
    let mut pitch = 0.;
    let mut yaw = 0.;

    loop {
        pitch += 0.01;
        if pitch >= TAU {
            pitch = 0.;
        }
        yaw += 0.01;
        if yaw >= TAU {
            yaw = 0.;
        }
        base_pose_b.rotation = UnitQuaternion::from_euler_angles(0., pitch, yaw);
        physics_client.reset_base_transform(ob_b, base_pose_b);

        let pts = match METHOD {
            ClosestPointsMethod::Body => {
                physics_client.get_closest_points_body_body(ob_a, None, ob_b, None, 100.)?
            }
            ClosestPointsMethod::BodyShape => physics_client.get_closest_points_body_shape(
                ob_a,
                None,
                geom_box,
                base_pose_b,
                100.,
            )?,
            ClosestPointsMethod::Shape => physics_client.get_closest_points_shape_shape(
                geom,
                Isometry3::translation(0.5, 0., 1.),
                geom_box,
                base_pose_b,
                100.,
            )?,
        };

        if !pts.is_empty() {
            println!("{:?}", pts[0]);
            let pt_a = pts[0].position_on_a;
            let pt_b = pts[0].position_on_b;
            physics_client.add_user_debug_line(
                pt_a,
                pt_b,
                AddDebugLineOptions {
                    line_color_rgb: color_rgb,
                    line_width,
                    life_time: 0.,
                    replace_item_id: Some(line_id),
                    ..Default::default()
                },
            )?;
        }
    }
}
