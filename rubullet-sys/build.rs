fn main() {
    // PyBullet does not enable `BT_THREADSAFE`. I assume this is because of the GIL - PyBullet does
    // nothing without the GIL locked. We'll make the same guarantee by using Rust's ownership
    // model.
    let mut config = &mut cmake::Config::new("bullet3");
    if config.get_profile() != "Release" {
        config = config.profile("RelWithDebInfo");
    }
    let dst = config.build();

    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:rustc-link-lib=static=BulletExampleBrowserLib");
    println!("cargo:rustc-link-lib=static=BulletRoboticsGUI");
    println!("cargo:rustc-link-lib=static=BulletRobotics");
    println!("cargo:rustc-link-lib=static=BulletFileLoader");
    println!("cargo:rustc-link-lib=static=BulletWorldImporter");
    println!("cargo:rustc-link-lib=static=BulletSoftBody");
    println!("cargo:rustc-link-lib=static=BulletDynamics");
    println!("cargo:rustc-link-lib=static=BulletCollision");
    println!("cargo:rustc-link-lib=static=BulletInverseDynamicsUtils");
    println!("cargo:rustc-link-lib=static=BulletInverseDynamics");
    println!("cargo:rustc-link-lib=static=LinearMath");
    println!("cargo:rustc-link-lib=static=OpenGLWindow");
    println!("cargo:rustc-link-lib=static=gwen");
    println!("cargo:rustc-link-lib=static=BussIK");
    println!("cargo:rustc-link-lib=static=Bullet3Common");
    println!("cargo:rustc-link-lib=static=cbullet");

    if cfg!(target_os = "macos") {
        println!("cargo:rustc-link-lib=c++");
        println!("cargo:rustc-link-lib=framework=Cocoa");
        println!("cargo:rustc-link-lib=framework=OpenGL");
    } else {
        println!("cargo:rustc-link-lib=stdc++");
    }
}
