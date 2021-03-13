use anyhow::Result;
use rubullet::*;
use std::time::Duration;

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::GraphicsServerTcp {
        hostname: "localhost",
        port: None,
    })?;

    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    let _plane_id = physics_client.load_urdf(
        "plane.urdf",
        UrdfOptions {
            use_maximal_coordinates: Some(false),
            ..Default::default()
        },
    )?;

    while physics_client.is_connected() {
        physics_client.step_simulation()?;
        std::thread::sleep(Duration::from_secs_f64(1. / 240.));
    }
    Ok(())
}
