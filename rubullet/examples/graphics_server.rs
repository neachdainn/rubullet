use anyhow::Result;
use rubullet::*;
use std::time::Duration;

fn main() -> Result<()> {
    let mut physics_server = PhysicsServer::new(ServerMode::Graphics { tcp_port: None })?;
    while physics_server.is_connected() {
        std::thread::sleep(Duration::from_secs_f64(1. / 240.));
    }
    Ok(())
}
