extern crate minifb;
extern crate nalgebra as na;
extern crate num_traits;
extern crate rayon;

use minifb::{Key, KeyRepeat, Scale, ScaleMode, Window, WindowOptions};
use na::{clamp, Point3, Vector3};
use num_traits::bounds::Bounded;
use rayon::prelude::*;

fn lorenz_eq(p: Point3<f64>, sigma: f64, beta: f64, rho: f64) -> Vector3<f64> {
    Vector3::new(sigma * (p[1] - p[0]), p[0] * (rho - p[2]) - p[1], p[0] * p[1] - beta * p[2])
}

fn rk4step<F: Fn(Point3<f64>) -> Vector3<f64>>(p: Point3<f64>, h: f64, f: F) -> Vector3<f64> {
    // https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta_methods
    let k1 = h * f(p);
    let k2 = h * f(p + k1 * 0.5);
    let k3 = h * f(p + k2 * 0.5);
    let k4 = h * f(p + k3);

    (k1 + 2.0 * (k2 + k3) + k4) / 6.0
}

struct BoundingBox {
    min: Point3<f64>,
    max: Point3<f64>,
}

fn compute_boundingbox<'a, T: Iterator<Item = &'a Point3<f64>>>(points: T) -> BoundingBox {
    points.fold(
        BoundingBox {
            min: Point3::max_value(),
            max: Point3::min_value(),
        },
        |aabb, p| BoundingBox {
            min: Point3::new(aabb.min.x.min(p.x), aabb.min.y.min(p.y), aabb.min.z.min(p.z)),
            max: Point3::new(aabb.max.x.max(p.x), aabb.max.y.max(p.y), aabb.max.z.max(p.z)),
        },
    )
}

fn pack_rgb(r: f64, g: f64, b: f64) -> u32 {
    let ri = (clamp(r, 0.0, 1.0) * 255.0) as u32;
    let gi = (clamp(g, 0.0, 1.0) * 255.0) as u32;
    let bi = (clamp(b, 0.0, 1.0) * 255.0) as u32;
    bi + (gi << 8) + (ri << 16)
}

fn update_framebuffer(points: &Vec<Point3<f64>>, buffer: &mut Vec<u32>, bb: &BoundingBox) {
    buffer.par_iter_mut().zip(points.par_iter()).for_each(|(pix, p)| {
        let r = (p.x - bb.min.x) / (bb.max.x - bb.min.x);
        let g = (p.y - bb.min.y) / (bb.max.y - bb.min.y);
        let b = (p.z - bb.min.z) / (bb.max.z - bb.min.z);
        *pix = pack_rgb(r, g, b); 
    });
}

const WIDTH: usize = 600;
const HEIGHT: usize = 600;

fn main() {
    let mut buffer: Vec<u32> = vec![0; WIDTH * HEIGHT];
    let mut points: Vec<Point3<f64>> = vec![Point3::new(0.0, 0.0, 0.0); WIDTH * HEIGHT];

    let mut window = Window::new(
        "Lorenz thing - ESC to exit",
        WIDTH,
        HEIGHT,
        WindowOptions {
            resize: true,
            scale: Scale::X1,
            scale_mode: ScaleMode::AspectRatioStretch,
            ..WindowOptions::default()
        },
    )
    .expect("Unable to Open Window");

    // Limit to max ~60 fps update rate
    window.limit_update_rate(Some(std::time::Duration::from_micros(16600)));
    window.set_background_color(0, 0, 0);

    let step_size = 0.01;
    let sigma = 10.0;
    let beta = 8.0 / 3.0;
    let rho = 28.0;

    // Init points.
    for i in 0..points.len() {
        let x = (i % WIDTH) as f64 / (WIDTH - 1) as f64;
        let y = (i / HEIGHT) as f64 / (HEIGHT - 1) as f64;
        points[i] = Point3::new(x * 30.0 - 15.0, 0.0, y * 30.0);
    }

    println!("ready");

    while window.is_open() && !window.is_key_down(Key::Escape) {
        if window.is_key_down(Key::Space) {
            // compute n steps
            points.par_iter_mut().for_each(|p| {
                let step = rk4step(*p, step_size, |p| lorenz_eq(p, sigma, beta, rho));
                *p += step;
            });

            let bb = compute_boundingbox(points.iter());
            //println!("min: {:?} max: {:?}", bb.min, bb.max);
            update_framebuffer(&points, &mut buffer, &bb);
        }
        window.update_with_buffer(&buffer, WIDTH, HEIGHT).unwrap();
    }
}
