use anyhow::Result;
use rubullet::*;

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    physics_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    physics_client.load_urdf("plane.urdf", None)?;
    let quadruped = physics_client.load_urdf("quadruped/quadruped.urdf", None)?;

    let log_id = physics_client.start_state_logging(
        LoggingType::Minitaur,
        "LOG00048.TXT",
        StateLoggingOptions {
            object_ids: vec![quadruped],
            ..Default::default()
        },
    )?;
    physics_client.step_simulation()?;
    physics_client.step_simulation()?;
    physics_client.step_simulation()?;
    physics_client.step_simulation()?;
    physics_client.step_simulation()?;
    physics_client.stop_state_logging(log_id);
    Ok(())
}
