use std::f64::consts::PI;

#[derive(Debug, Clone, Copy)]
pub struct TelemetryPoint {
    pub time_offset_ms: u64,
    pub longitude: f64,
    pub latitude: f64,
    pub altitude: f64,
    pub sun_intensity: f32,
}

#[derive(Debug, Clone, Copy)]
struct Point2D {
    x: f64,
    y: f64,
}

trait Segment2D {
    fn length(&self) -> f64;
    fn get_point(&self, s: f64) -> Point2D;
}

struct LineSegment {
    p1: Point2D,
    p2: Point2D,
    length: f64,
}

impl LineSegment {
    fn new(p1: Point2D, p2: Point2D) -> Self {
        let length = (p2.x - p1.x).hypot(p2.y - p1.y);
        Self { p1, p2, length }
    }
}

impl Segment2D for LineSegment {
    fn length(&self) -> f64 { self.length }
    fn get_point(&self, s: f64) -> Point2D {
        let frac = if self.length > 0.0 { (s / self.length).clamp(0.0, 1.0) } else { 0.0 };
        Point2D {
            x: self.p1.x + (self.p2.x - self.p1.x) * frac,
            y: self.p1.y + (self.p2.y - self.p1.y) * frac,
        }
    }
}

struct ArcSegment {
    center: Point2D,
    radius: f64,
    start_angle: f64,
    sweep_angle: f64,
    length: f64,
}

impl ArcSegment {
    fn new(center: Point2D, radius: f64, start_angle: f64, sweep_angle: f64) -> Self {
        let length = radius * sweep_angle.abs();
        Self { center, radius, start_angle, sweep_angle, length }
    }
}

impl Segment2D for ArcSegment {
    fn length(&self) -> f64 { self.length }
    fn get_point(&self, s: f64) -> Point2D {
        let frac = if self.length > 0.0 { (s / self.length).clamp(0.0, 1.0) } else { 0.0 };
        let angle = self.start_angle + self.sweep_angle * frac;
        Point2D {
            x: self.center.x + self.radius * angle.cos(),
            y: self.center.y + self.radius * angle.sin(),
        }
    }
}

struct Path2D {
    segments: Vec<Box<dyn Segment2D>>,
}

impl Path2D {
    fn new() -> Self {
        Self { segments: Vec::new() }
    }

    fn total_length(&self) -> f64 {
        self.segments.iter().map(|s| s.length()).sum()
    }

    fn add_dubins_path(&mut self, p1: Point2D, h1: f64, p2: Point2D, h2: f64, radius: f64) {
        let path = dubins_solver::solve(p1, h1, p2, h2, radius);
        
        let mut sweep1 = (path.t1.y - path.c1.y).atan2(path.t1.x - path.c1.x) - (p1.y - path.c1.y).atan2(p1.x - path.c1.x);
        while sweep1 < -PI { sweep1 += 2.0 * PI; }
        while sweep1 > PI { sweep1 -= 2.0 * PI; }
        if path.dir1 == dubins_solver::TurnDir::L && sweep1 < 0.0 { sweep1 += 2.0 * PI; }
        if path.dir1 == dubins_solver::TurnDir::R && sweep1 > 0.0 { sweep1 -= 2.0 * PI; }
        
        if sweep1.abs() > 1e-6 {
            self.segments.push(Box::new(ArcSegment::new(path.c1, radius, (p1.y - path.c1.y).atan2(p1.x - path.c1.x), sweep1)));
        }
        
        self.segments.push(Box::new(LineSegment::new(path.t1, path.t2)));
        
        let mut sweep2 = (p2.y - path.c2.y).atan2(p2.x - path.c2.x) - (path.t2.y - path.c2.y).atan2(path.t2.x - path.c2.x);
        while sweep2 < -PI { sweep2 += 2.0 * PI; }
        while sweep2 > PI { sweep2 -= 2.0 * PI; }
        if path.dir2 == dubins_solver::TurnDir::L && sweep2 < 0.0 { sweep2 += 2.0 * PI; }
        if path.dir2 == dubins_solver::TurnDir::R && sweep2 > 0.0 { sweep2 -= 2.0 * PI; }
        
        if sweep2.abs() > 1e-6 {
            self.segments.push(Box::new(ArcSegment::new(path.c2, radius, (path.t2.y - path.c2.y).atan2(path.t2.x - path.c2.x), sweep2)));
        }
    }

    fn get_point(&self, s: f64) -> Point2D {
        let mut remaining = s.clamp(0.0, self.total_length());
        for seg in &self.segments {
            if remaining <= seg.length() + 1e-6 {
                return seg.get_point(remaining);
            }
            remaining -= seg.length();
        }
        if let Some(last) = self.segments.last() {
            last.get_point(last.length())
        } else {
            Point2D { x: 0.0, y: 0.0 }
        }
    }
}

mod dubins_solver {
    use super::Point2D;
    use std::f64::consts::PI;

    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum TurnDir { L, R }

    #[derive(Debug, Clone)]
    pub struct DubinsPath {
        pub path_type: &'static str,
        pub length: f64,
        pub c1: Point2D,
        pub dir1: TurnDir,
        pub t1: Point2D,
        pub t2: Point2D,
        pub c2: Point2D,
        pub dir2: TurnDir,
    }

    pub fn solve(p1: Point2D, h1: f64, p2: Point2D, h2: f64, r: f64) -> DubinsPath {
        let mut paths = Vec::new();

        let r1 = Point2D { x: h1.cos(), y: -h1.sin() };
        let l1 = Point2D { x: -h1.cos(), y: h1.sin() };
        let r2 = Point2D { x: h2.cos(), y: -h2.sin() };
        let l2 = Point2D { x: -h2.cos(), y: h2.sin() };

        let c_r1 = Point2D { x: p1.x + r * r1.x, y: p1.y + r * r1.y };
        let c_l1 = Point2D { x: p1.x + r * l1.x, y: p1.y + r * l1.y };
        let c_r2 = Point2D { x: p2.x + r * r2.x, y: p2.y + r * r2.y };
        let c_l2 = Point2D { x: p2.x + r * l2.x, y: p2.y + r * l2.y };

        let mut evaluate = |path_type: &'static str, c1: Point2D, dir1: TurnDir, c2: Point2D, dir2: TurnDir, t1: Point2D, t2: Point2D| {
            let mut sweep1 = (t1.y - c1.y).atan2(t1.x - c1.x) - (p1.y - c1.y).atan2(p1.x - c1.x);
            while sweep1 < -PI { sweep1 += 2.0 * PI; }
            while sweep1 > PI { sweep1 -= 2.0 * PI; }
            if dir1 == TurnDir::L && sweep1 < 0.0 { sweep1 += 2.0 * PI; }
            if dir1 == TurnDir::R && sweep1 > 0.0 { sweep1 -= 2.0 * PI; }

            let mut sweep2 = (p2.y - c2.y).atan2(p2.x - c2.x) - (t2.y - c2.y).atan2(t2.x - c2.x);
            while sweep2 < -PI { sweep2 += 2.0 * PI; }
            while sweep2 > PI { sweep2 -= 2.0 * PI; }
            if dir2 == TurnDir::L && sweep2 < 0.0 { sweep2 += 2.0 * PI; }
            if dir2 == TurnDir::R && sweep2 > 0.0 { sweep2 -= 2.0 * PI; }

            let len = r * sweep1.abs() + (t2.x - t1.x).hypot(t2.y - t1.y) + r * sweep2.abs();
            paths.push(DubinsPath { path_type, length: len, c1, dir1, t1, t2, c2, dir2 });
        };

        // LSL
        let mut v = Point2D { x: c_l2.x - c_l1.x, y: c_l2.y - c_l1.y };
        let mut d = v.x.hypot(v.y);
        if d > 1e-6 {
            let gamma = v.y.atan2(v.x);
            let nx = gamma.cos();
            let ny = -gamma.sin();
            let t1 = Point2D { x: c_l1.x + r * nx, y: c_l1.y + r * ny };
            let t2 = Point2D { x: c_l2.x + r * nx, y: c_l2.y + r * ny };
            evaluate("LSL", c_l1, TurnDir::L, c_l2, TurnDir::L, t1, t2);
        }

        // RSR
        v = Point2D { x: c_r2.x - c_r1.x, y: c_r2.y - c_r1.y };
        d = v.x.hypot(v.y);
        if d > 1e-6 {
            let gamma = v.y.atan2(v.x);
            let nx = -gamma.cos();
            let ny = gamma.sin();
            let t1 = Point2D { x: c_r1.x + r * nx, y: c_r1.y + r * ny };
            let t2 = Point2D { x: c_r2.x + r * nx, y: c_r2.y + r * ny };
            evaluate("RSR", c_r1, TurnDir::R, c_r2, TurnDir::R, t1, t2);
        }

        // RSL
        v = Point2D { x: c_l2.x - c_r1.x, y: c_l2.y - c_r1.y };
        d = v.x.hypot(v.y);
        if d >= 2.0 * r {
            let gamma = v.y.atan2(v.x);
            let beta = (2.0 * r / d).asin();
            let path_heading = gamma + beta;
            let t1 = Point2D { x: c_r1.x + r * (-path_heading.cos()), y: c_r1.y + r * path_heading.sin() };
            let t2 = Point2D { x: c_l2.x + r * path_heading.cos(), y: c_l2.y + r * (-path_heading.sin()) };
            evaluate("RSL", c_r1, TurnDir::R, c_l2, TurnDir::L, t1, t2);
        }

        // LSR
        v = Point2D { x: c_r2.x - c_l1.x, y: c_r2.y - c_l1.y };
        d = v.x.hypot(v.y);
        if d >= 2.0 * r {
            let gamma = v.y.atan2(v.x);
            let beta = (2.0 * r / d).asin();
            let path_heading = gamma - beta;
            let t1 = Point2D { x: c_l1.x + r * path_heading.cos(), y: c_l1.y + r * (-path_heading.sin()) };
            let t2 = Point2D { x: c_r2.x + r * (-path_heading.cos()), y: c_r2.y + r * path_heading.sin() };
            evaluate("LSR", c_l1, TurnDir::L, c_r2, TurnDir::R, t1, t2);
        }

        paths.into_iter()
            .min_by(|a, b| a.length.partial_cmp(&b.length).unwrap())
            .expect("No valid Dubins path found")
    }
}

pub fn generate(
    departure_lon: f64,
    departure_lat: f64,
    arrival_lon: f64,
    arrival_lat: f64,
    total_duration_ms: u64,
    dep_heading_deg: Option<f64>,
    arr_heading_deg: Option<f64>,
) -> Vec<TelemetryPoint> {
    let lat_mid = (departure_lat + arrival_lat).to_radians() / 2.0;
    let m_per_deg_lat = 111320.0;
    let m_per_deg_lon = 111320.0 * lat_mid.cos();

    let to_2d = |lon: f64, lat: f64| Point2D {
        x: (lon - departure_lon) * m_per_deg_lon,
        y: (lat - departure_lat) * m_per_deg_lat,
    };
    let to_geo = |p: Point2D| (
        departure_lon + p.x / m_per_deg_lon,
        departure_lat + p.y / m_per_deg_lat,
    );

    let p_dep = to_2d(departure_lon, departure_lat);
    let p_arr = to_2d(arrival_lon, arrival_lat);

    let direct_heading_rad = (p_arr.x - p_dep.x).atan2(p_arr.y - p_dep.y);
    let dep_h_rad = dep_heading_deg.map(|d| d.to_radians()).unwrap_or(direct_heading_rad);
    let arr_h_rad = arr_heading_deg.map(|d| d.to_radians()).unwrap_or(direct_heading_rad);

    let w0 = p_dep;
    let w1 = Point2D { x: w0.x + 10000.0 * dep_h_rad.sin(), y: w0.y + 10000.0 * dep_h_rad.cos() };
    let w2 = Point2D { x: p_arr.x - 15000.0 * arr_h_rad.sin(), y: p_arr.y - 15000.0 * arr_h_rad.cos() };
    let w3 = p_arr;

    let mut path = Path2D::new();
    let turn_radius = 4000.0;
    
    path.segments.push(Box::new(LineSegment::new(w0, w1)));
    path.add_dubins_path(w1, dep_h_rad, w2, arr_h_rad, turn_radius);
    path.segments.push(Box::new(LineSegment::new(w2, w3)));

    let s_total = path.total_length();

    let cruise_alt = 10000.0;
    
    // Distances for takeoff and landing
    let ideal_ground = 3000.0_f64;
    let ideal_landing = 3000.0_f64;
    let ideal_climb = 30000.0;
    let ideal_descent = 50000.0;
    
    // Safety check: if path is extremely short, shrink the ground distances
    let ground_dist = ideal_ground.min(s_total * 0.1);
    let landing_dist = ideal_landing.min(s_total * 0.1);
    let air_dist = s_total - ground_dist - landing_dist;

    let (climb_dist, desc_dist, cruise_dist) = if air_dist >= ideal_climb + ideal_descent {
        (ideal_climb, ideal_descent, air_dist - ideal_climb - ideal_descent)
    } else {
        (air_dist * (3.0 / 8.0), air_dist * (5.0 / 8.0), 0.0)
    };

    let s_ground_end = ground_dist;
    let s_climb_end = s_ground_end + climb_dist;
    let s_desc_start = s_climb_end + cruise_dist;
    let s_land_start = s_desc_start + desc_dist;

    let km = s_total / 1000.0;
    let cruise_alt_m = if km < 300.0 {
        lerp(6000.0, 7000.0, km / 300.0)
    } else if km < 800.0 {
        lerp(7000.0, 10000.0, (km - 300.0) / 500.0)
    } else if km < 3000.0 {
        lerp(10000.0, 11500.0, (km - 800.0) / 2200.0)
    } else {
        lerp(11500.0, 13000.0, ((km - 3000.0) / 10000.0).min(1.0))
    };

    let get_altitude = |s: f64| -> f64 {
        if s <= s_ground_end {
            0.0
        } else if s <= s_climb_end {
            let climb_dist_actual = s_climb_end - s_ground_end;
            let sigma = s - s_ground_end;
            let d_rot = climb_dist_actual * 0.1;
            let d_lvl = climb_dist_actual * 0.2;
            let d_lin = climb_dist_actual - d_rot - d_lvl;
            let max_slope = cruise_alt_m / (climb_dist_actual - 0.5 * (d_rot + d_lvl));
            
            if sigma <= d_rot {
                0.5 * max_slope * (sigma * sigma / d_rot)
            } else if sigma <= d_rot + d_lin {
                let z_rot_end = 0.5 * max_slope * d_rot;
                z_rot_end + max_slope * (sigma - d_rot)
            } else {
                let s_lvl = sigma - (d_rot + d_lin);
                let z_lin_end = 0.5 * max_slope * d_rot + max_slope * d_lin;
                z_lin_end + max_slope * s_lvl - 0.5 * max_slope * (s_lvl * s_lvl / d_lvl)
            }
        } else if s <= s_desc_start {
            cruise_alt_m
        } else if s <= s_land_start {
            let desc_dist_actual = s_land_start - s_desc_start;
            let sigma = s - s_desc_start;
            let d_tod = desc_dist_actual * 0.2;
            let d_flare = desc_dist_actual * 0.1;
            let d_lin = desc_dist_actual - d_tod - d_flare;
            let max_slope = cruise_alt_m / (desc_dist_actual - 0.5 * (d_tod + d_flare));
            
            if sigma <= d_tod {
                cruise_alt_m - 0.5 * max_slope * (sigma * sigma / d_tod)
            } else if sigma <= d_tod + d_lin {
                let z_tod_end = cruise_alt_m - 0.5 * max_slope * d_tod;
                z_tod_end - max_slope * (sigma - d_tod)
            } else {
                let s_flare = sigma - (d_tod + d_lin);
                let z_lin_end = cruise_alt_m - 0.5 * max_slope * d_tod - max_slope * d_lin;
                z_lin_end - (max_slope * s_flare - 0.5 * max_slope * (s_flare * s_flare / d_flare))
            }
        } else {
            0.0
        }
    };

    let get_speed_shape = |s: f64| -> f64 {
        if s < s_ground_end {
            lerp(0.05, 0.3, s / s_ground_end)
        } else if s < s_climb_end {
            lerp(0.3, 1.0, (s - s_ground_end) / (s_climb_end - s_ground_end))
        } else if s < s_desc_start {
            1.0
        } else if s < s_land_start {
            lerp(1.0, 0.2, (s - s_desc_start) / (s_land_start - s_desc_start))
        } else {
            lerp(0.2, 0.02, (s - s_land_start) / (s_total - s_land_start))
        }
    };

    let integration_steps = 1000;
    let ds = s_total / integration_steps as f64;
    let mut unscaled_time = 0.0;
    for i in 0..integration_steps {
        let s_mid = i as f64 * ds + ds / 2.0;
        unscaled_time += ds / get_speed_shape(s_mid);
    }

    let total_s = total_duration_ms as f64 / 1000.0;
    let mut points = Vec::new();
    let mut current_s = 0.0;
    let mut current_t = 0.0;
    let dt = 2.0;

    while current_t <= total_s {
        let p2d = path.get_point(current_s);
        let (lon, lat) = to_geo(p2d);
        let alt = get_altitude(current_s);
        
        points.push(TelemetryPoint {
            time_offset_ms: (current_t * 1000.0) as u64,
            longitude: lon,
            latitude: lat,
            altitude: alt,
            sun_intensity: 1.0, // Default sun intensity
        });

        let v_shape = get_speed_shape(current_s);
        let real_v = v_shape * (unscaled_time / total_s);
        current_s += real_v * dt;
        current_t += dt;
    }

    let (final_lon, final_lat) = to_geo(path.get_point(s_total));
    points.push(TelemetryPoint {
        time_offset_ms: total_duration_ms,
        longitude: final_lon,
        latitude: final_lat,
        altitude: 0.0,
        sun_intensity: 1.0,
    });

    points
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}
