extern crate minifb;
extern crate nalgebra as na;

use minifb::{Key, Scale, ScaleMode, Window, WindowOptions};
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

const WIDTH: usize = 20;
const HEIGHT: usize = 20;

fn main() {
    let mut buffer: Vec<u32> = vec![0; WIDTH * HEIGHT * 2];

    let mut window = Window::new(
        "Lorenz thing - ESC to exit",
        WIDTH,
        HEIGHT,
        WindowOptions {
            resize: true,
            scale: Scale::X2,
            scale_mode: ScaleMode::AspectRatioStretch,
            ..WindowOptions::default()
        },
    )
    .expect("Unable to Open Window");

    // Limit to max ~60 fps update rate
    window.limit_update_rate(Some(std::time::Duration::from_micros(16600)));


    let mut angle: f64 = 0.0;

    window.set_background_color(0, 0, 0);

    let num_steps = 100;
    let step_size = 0.1;
    let sigma = 10.0;
    let beta = 8.0/3.0;
    let rho = 28.0;

    while window.is_open() && !window.is_key_down(Key::Escape) {
        for i in 0..buffer.len() {
            let mut p = Point3::new(
                ((i % WIDTH) as f64 / (WIDTH - 1) as f64) * 30.0 - 15.0,
                1.0,
                ((i / HEIGHT) as f64 / (HEIGHT - 1) as f64) * 30.0,
            );
            
            for _ in 0..num_steps {
                let p = p + rk4step(p, step_size, |p| lorenz_eq(p, sigma, beta, rho));
            }

            buffer[i] = (clamp(p.x / 15.0 + 1.0, 0.0, 1.0) * 255.0) as u32;
        }

        angle += 0.1;

        // We unwrap here as we want this code to exit if it fails
        window.update_with_buffer(&buffer, WIDTH, HEIGHT).unwrap();
    }
}
