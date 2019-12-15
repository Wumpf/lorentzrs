extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::window::*;
use kiss3d::camera::ArcBall;
use na::{Vector3, Point3};

fn lorenz_eq(p: Point3<f64>, sigma: f64, beta: f64, rho: f64) -> Vector3<f64> {
    Vector3::new(sigma * (p[0] - p[1]), p[0] * (rho - p[2]) - p[1], p[0] * p[1] - beta * p[2])
}

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

    let eye = Point3::new(-1000.0f32, 0.0, 0.0);
    let at = Point3::origin();
    let mut camera = ArcBall::new_with_frustrum(std::f32::consts::PI / 4.0, 0.1, 10000.0, eye, at);

    while window.render_with_camera(&mut camera) {
        let sigma = 10.0;
        let beta = 8.0/3.0;
        let rho = 28.0;

        let num_samples = 10000;
        let dt = 0.001;

        // Want to try out proper integration, but let's start simple with forward euler and watch it burn.
        let mut prev = Point3::new(1.0f64, 1.0, 1.0);
        for _ in 0..num_samples {
            let next = prev + lorenz_eq(prev, sigma, beta, rho) * dt;
            window.draw_line(&nalgebra::convert(prev), &nalgebra::convert(next), &Point3::new(1.0, 0.0, 0.0));
            prev = next;
        }
    }
}
