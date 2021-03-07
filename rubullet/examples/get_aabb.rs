use anyhow::Result;
use rubullet::*;
fn draw_aabb(a: Aabb, client: &mut PhysicsClient) {
    let to = [a.max.x, a.min.y, a.min.z];
    client
        .add_user_debug_line(
            a.min,
            to,
            AddDebugLineOptions {
                line_color_rgb: [1., 0., 0.],
                ..Default::default()
            },
        )
        .unwrap();
    let to = [a.min.x, a.max.y, a.min.z];
    client
        .add_user_debug_line(
            a.min,
            to,
            AddDebugLineOptions {
                line_color_rgb: [0., 1., 0.],
                ..Default::default()
            },
        )
        .unwrap();
    let to = [a.min.x, a.min.y, a.max.z];
    client
        .add_user_debug_line(
            a.min,
            to,
            AddDebugLineOptions {
                line_color_rgb: [0., 0., 1.],
                ..Default::default()
            },
        )
        .unwrap();

    let from = [a.min.x, a.min.y, a.max.z];
    let to = [a.min.x, a.max.y, a.max.z];
    client
        .add_user_debug_line(
            from,
            to,
            AddDebugLineOptions {
                line_color_rgb: [1., 1., 1.],
                ..Default::default()
            },
        )
        .unwrap();
    let to = [a.max.x, a.min.y, a.max.z];
    client
        .add_user_debug_line(
            from,
            to,
            AddDebugLineOptions {
                line_color_rgb: [1., 1., 1.],
                ..Default::default()
            },
        )
        .unwrap();
    let from = [a.max.x, a.min.y, a.min.z];

    client
        .add_user_debug_line(
            from,
            to,
            AddDebugLineOptions {
                line_color_rgb: [1., 1., 1.],
                ..Default::default()
            },
        )
        .unwrap();

    let to = [a.max.x, a.max.y, a.min.z];
    client
        .add_user_debug_line(
            from,
            to,
            AddDebugLineOptions {
                line_color_rgb: [1., 1., 1.],
                ..Default::default()
            },
        )
        .unwrap();
    let from = [a.max.x, a.max.y, a.min.z];
    let to = [a.min.x, a.max.y, a.min.z];
    client
        .add_user_debug_line(
            from,
            to,
            AddDebugLineOptions {
                line_color_rgb: [1., 1., 1.],
                ..Default::default()
            },
        )
        .unwrap();

    let from = [a.min.x, a.max.y, a.min.z];
    let to = [a.min.x, a.max.y, a.max.z];
    client
        .add_user_debug_line(
            from,
            to,
            AddDebugLineOptions {
                line_color_rgb: [1., 1., 1.],
                ..Default::default()
            },
        )
        .unwrap();

    client
        .add_user_debug_line(
            a.max,
            to,
            AddDebugLineOptions {
                line_color_rgb: [1., 1., 1.],
                ..Default::default()
            },
        )
        .unwrap();

    let to = [a.max.x, a.min.y, a.max.z];
    client
        .add_user_debug_line(
            a.max,
            to,
            AddDebugLineOptions {
                line_color_rgb: [1., 1., 1.],
                ..Default::default()
            },
        )
        .unwrap();
    let to = [a.max.x, a.max.y, a.min.z];
    client
        .add_user_debug_line(
            a.max,
            to,
            AddDebugLineOptions {
                line_color_rgb: [1., 1., 1.],
                ..Default::default()
            },
        )
        .unwrap();
}
fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    let r2d2 = physics_client.load_urdf("r2d2.urdf", None)?;
    let aabb = physics_client.get_aabb(r2d2, None)?;
    println!("{:?}", aabb);
    draw_aabb(aabb, &mut physics_client);
    for i in 0..physics_client.get_num_joints(r2d2) {
        let aabb = physics_client.get_aabb(r2d2, i)?;
        println!("{:?}", aabb);
        draw_aabb(aabb, &mut physics_client);
    }
    loop {
        physics_client.step_simulation()?;
    }
}
