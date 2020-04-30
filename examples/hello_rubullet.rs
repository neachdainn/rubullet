//! An introduction to the usage of RuBullet.
use rubullet::*;
use easy_error::Terminator;

fn main() -> Result<(), Terminator>
{
	let physics_client = PhysicsClient::connect(Mode::Gui)?;

	todo!();
}
