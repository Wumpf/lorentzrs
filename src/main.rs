extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::window::*;
use kiss3d::camera::ArcBall;
use na::{Vector3, Point3, clamp};

fn lorenz_eq(p: Point3<f64>, sigma: f64, beta: f64, rho: f64) -> Vector3<f64> {
    Vector3::new(
        sigma * (p[1] - p[0]),
        p[0] * (rho - p[2]) - p[1],
        p[0] * p[1] - beta * p[2]
    )
}

fn rk4step<F: Fn(Point3<f64>) -> Vector3<f64>>(p: Point3<f64>, h: f64, f: F) -> Vector3<f64> {
    // https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
    let k1 = h * f(p);
    let k2 = h * f(p + k1 * 0.5);
    let k3 = h * f(p + k2 * 0.5);
    let k4 = h * f(p + k3);

    (k1 + 2.0 * (k2 + k3) + k4) / 6.0
}

fn heatmap(t: f32) -> Point3<f32> {
    Point3::new(
        clamp(t * 3.0, 0.0, 1.0),
        clamp(t * 3.0 - 1.0, 0.0, 1.0),
        clamp(t * 3.0 - 2.0, 0.0, 1.0))
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

    let eye = Point3::new(50.0f32, 0.0, 0.0);
    let at = Point3::origin();
    let mut camera = ArcBall::new_with_frustrum(std::f32::consts::PI / 3.0, 0.1, 1000.0, eye, at);

    while window.render_with_camera(&mut camera) {
        let sigma = 10.0;
        let beta = 8.0/3.0;
        let rho = 28.0;

        let num_samples = 10000;
        let dt = 0.01;

        let mut prev = Point3::new(1.0f64, 1.0, 1.0);
        for s in 0..num_samples {
            // Want to try out proper integration, but let's start simple with forward euler and watch it burn.
            //let next = prev + lorenz_eq(prev, sigma, beta, rho) * dt;
            // Runge Kutta integration
            let next = prev + rk4step(prev, dt, |p| lorenz_eq(p, sigma, beta, rho));

            let progress = (s as f32) / (num_samples as f32 - 1.0);
            window.draw_line(&nalgebra::convert(prev), &nalgebra::convert(next), &heatmap(progress));
            prev = next;
        }
    }
}
