//! An introduction to the usage of RuBullet.
use rubullet::*;
use easy_error::Terminator;

fn main() -> Result<(), Terminator>
{
	let physics_client = PhysicsClient::connect(Mode::Gui)?;
	std::thread::sleep(std::time::Duration::from_secs(60 * 60));

	todo!();
}
