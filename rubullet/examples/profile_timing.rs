use anyhow::Result;
use rubullet::*;
use std::time::{Duration, Instant};

fn main() -> Result<()> {
    let mut physics_client = PhysicsClient::connect(Mode::Gui)?;
    let t = Instant::now() + Duration::from_secs_f64(3.1);
    let log_id = physics_client.start_state_logging(
        LoggingType::ProfileTimings,
        "chrome_about_tracing.json",
        None,
    )?;
    while Instant::now() < t {
        physics_client.step_simulation()?;

        physics_client.submit_profile_timing("rusttest");
        std::thread::sleep(Duration::from_secs_f64(1. / 240.));

        physics_client.submit_profile_timing("nested");
        for _ in 0..100 {
            physics_client.submit_profile_timing("deep_nested");
            physics_client.submit_profile_timing(None);
        }
        std::thread::sleep(Duration::from_millis(1));
        physics_client.submit_profile_timing(None);
        physics_client.submit_profile_timing(None);
    }
    physics_client.stop_state_logging(log_id);
    Ok(())
}
