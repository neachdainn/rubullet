use anyhow::Result;
use nalgebra::Isometry3;
use rubullet::*;
use std::time::Duration;

fn main() -> Result<()> {
    let mut gui_client = PhysicsClient::connect(Mode::GuiServer)?;
    gui_client.set_additional_search_path("../rubullet-sys/bullet3/libbullet3/data")?;
    let cube = gui_client.load_urdf(
        "cube.urdf",
        UrdfOptions {
            use_maximal_coordinates: Some(false),
            ..Default::default()
        },
    )?;
    std::thread::spawn(move || {
        let mut shared_memory_client = PhysicsClient::connect(Mode::SharedMemory).unwrap();
        let mut i: usize = 0;
        loop {
            std::thread::sleep(Duration::from_secs_f64(1. / 240.));
            shared_memory_client.reset_base_transform(
                cube,
                Isometry3::translation(f64::sin(i as f64 * 0.02), 0., 0.),
            );
            i += 1;
        }
    });

    // this thread will fail as there can only be one client using the SharedMemory Mode
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_secs_f64(1.));
        let mut client_fail = PhysicsClient::connect(Mode::SharedMemory).unwrap();
        let mut i: usize = 0;
        loop {
            std::thread::sleep(Duration::from_secs_f64(1. / 240.));
            client_fail.reset_base_transform(
                cube,
                Isometry3::translation(f64::sin(i as f64 * 0.02), 0., 0.),
            );
            i += 1;
        }
    });

    while gui_client.is_connected() {
        gui_client.step_simulation()?;
        println!(
            "{}",
            gui_client.get_base_transform(cube).unwrap().translation.x
        );
        std::thread::sleep(Duration::from_secs_f64(1. / 2.));
    }
    Ok(())
}
