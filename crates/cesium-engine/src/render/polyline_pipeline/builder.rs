use glam::DVec3;

/// A single raw control point uploaded to the GPU.
/// The vertex shader reads these from a storage buffer and expands them
/// into thick-ribbon quads — no CPU-side geometry expansion needed.
#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ControlPoint {
    /// World-space position relative to the flight's `reference_point` (f32 precision).
    pub position: [f32; 3],
    /// Normalised progress along the full flight path (0.0 – 1.0).
    pub progress: f32,
}

/// Builds an adaptive set of control points from a `SampledPositionProperty`.
/// Output is a flat `Vec<ControlPoint>` — no pre-expanded geometry.
pub struct AdaptiveSubdivisionBuilder {
    pub tolerance: f64,
    /// Minimum time step in seconds to avoid infinite recursion.
    pub min_step: f64,
    pub force_all_samples: bool,
}

impl AdaptiveSubdivisionBuilder {
    pub fn new(tolerance: f64) -> Self {
        Self {
            tolerance,
            min_step: 0.1, // 100 ms
            force_all_samples: false,
        }
    }

    /// Build a flat list of control points from the position property.
    /// `reference_point` is subtracted from each position so values fit in f32.
    pub fn build(
        &self,
        property: &crate::property::sampled::SampledPositionProperty,
        reference_point: DVec3,
    ) -> Vec<ControlPoint> {
        use crate::property::Property;
        use crate::time::SimulationTime;

        let start_time = match property.start_time() {
            Some(t) => t,
            None => return Vec::new(),
        };
        let stop_time = match property.stop_time() {
            Some(t) => t,
            None => return Vec::new(),
        };

        if start_time.seconds >= stop_time.seconds {
            return Vec::new();
        }

        let total_duration = stop_time.seconds - start_time.seconds;

        if self.force_all_samples {
            return property
                .samples()
                .iter()
                .map(|(t, p)| {
                    let progress =
                        ((t.seconds - start_time.seconds) / total_duration).clamp(0.0, 1.0) as f32;
                    let rel = *p - reference_point;
                    ControlPoint {
                        position: [rel.x as f32, rel.y as f32, rel.z as f32],
                        progress,
                    }
                })
                .collect();
        }

        let mut path_points: Vec<(DVec3, f64)> = Vec::new(); // (position, time)

        let mut current_time = start_time.seconds;
        let mut last_p = property
            .evaluate(SimulationTime::new(current_time))
            .unwrap();
        path_points.push((last_p, current_time));

        let max_step = 60.0 * 5.0; // 5-minute max step

        while current_time < stop_time.seconds {
            let next_time = (current_time + max_step).min(stop_time.seconds);
            let p_start = last_p;
            let p_end = property.evaluate(SimulationTime::new(next_time)).unwrap();

            self.subdivide(property, current_time, next_time, p_start, p_end, &mut path_points);

            path_points.push((p_end, next_time));
            current_time = next_time;
            last_p = p_end;
        }

        path_points
            .into_iter()
            .map(|(p, t)| {
                let progress = ((t - start_time.seconds) / total_duration).clamp(0.0, 1.0) as f32;
                let rel = p - reference_point;
                ControlPoint {
                    position: [rel.x as f32, rel.y as f32, rel.z as f32],
                    progress,
                }
            })
            .collect()
    }

    fn subdivide(
        &self,
        property: &crate::property::sampled::SampledPositionProperty,
        t_start: f64,
        t_end: f64,
        p_start: DVec3,
        p_end: DVec3,
        points: &mut Vec<(DVec3, f64)>,
    ) {
        use crate::property::Property;
        use crate::time::SimulationTime;

        if (t_end - t_start) <= self.min_step {
            return;
        }

        let t_mid = (t_start + t_end) * 0.5;
        let p_mid_true = property.evaluate(SimulationTime::new(t_mid)).unwrap();

        let line_vec = p_end - p_start;
        let length_sq = line_vec.length_squared();

        let dist = if length_sq < 1e-8 {
            (p_mid_true - p_start).length()
        } else {
            let t = ((p_mid_true - p_start).dot(line_vec) / length_sq).clamp(0.0, 1.0);
            let projection = p_start + line_vec * t;
            (p_mid_true - projection).length()
        };

        if dist > self.tolerance {
            self.subdivide(property, t_start, t_mid, p_start, p_mid_true, points);
            points.push((p_mid_true, t_mid));
            self.subdivide(property, t_mid, t_end, p_mid_true, p_end, points);
        }
    }
}
