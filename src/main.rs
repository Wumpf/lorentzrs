extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::window::*;
use na::{Point3};

fn main() {
    let mut window = Window::new_with_setup(
        "lorenz-rs",
        1920,
        1080,
        CanvasSetup {
            vsync: true,
            samples: NumSamples::Sixteen,
        },
    );
    window.set_background_color(0.1, 0.1, 0.15);

    while window.render() {
        let a = Point3::new(-0.1, -0.1, 0.0);
        let b = Point3::new(0.0, 0.1, 0.0);
        let c = Point3::new(0.1, -0.1, 0.0);

        window.draw_line(&a, &b, &Point3::new(1.0, 0.0, 0.0));
        window.draw_line(&b, &c, &Point3::new(0.0, 1.0, 0.0));
        window.draw_line(&c, &a, &Point3::new(0.0, 0.0, 1.0));
    }
}
