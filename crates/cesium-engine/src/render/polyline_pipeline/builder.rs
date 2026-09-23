use glam::DVec3;

/// A single raw control point uploaded to the GPU.
/// The vertex shader reads these from a storage buffer and expands them
/// into thick-ribbon quads with double-single RTC high/low precision — eliminating jitter.
#[repr(C)]
#[derive(Copy, Clone, Debug, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ControlPoint {
    /// High part of world-space ECEF position (Megameters).
    pub pos_hi: [f32; 3],
    /// Distance travelled along the path to reach this point (Megameters).
    ///
    /// `progress` below is normalised *time*, and a flight spends its time very
    /// unevenly along its length — the climb and the descent are slow. Anything that
    /// needs to talk about the route in miles rather than in minutes needs this instead.
    pub distance: f32,
    /// Low part of world-space ECEF position (Megameters).
    pub pos_lo: [f32; 3],
    /// Normalised progress along the full flight path (0.0 – 1.0).
    pub progress: f32,
}

impl ControlPoint {
    pub fn from_dvec3(pos: DVec3, progress: f32) -> Self {
        let hi = [pos.x as f32, pos.y as f32, pos.z as f32];
        let lo = [
            (pos.x - hi[0] as f64) as f32,
            (pos.y - hi[1] as f64) as f32,
            (pos.z - hi[2] as f64) as f32,
        ];
        Self {
            pos_hi: hi,
            distance: 0.0,
            pos_lo: lo,
            progress,
        }
    }
}

/// Builds an adaptive set of control points from a `SampledPositionProperty`.
/// Output is a flat `Vec<ControlPoint>` — no pre-expanded geometry.
pub struct AdaptiveSubdivisionBuilder {
    pub tolerance: f64,
    /// Minimum time step in seconds to avoid infinite recursion.
    pub min_step: f64,
    pub force_all_samples: bool,
    /// Stretches of the path, as `(start, end, max_length)` — times in seconds, length in
    /// Megametres — inside which no segment may be longer than `max_length`, however
    /// straight the path is there.
    ///
    /// For geometry that is moved point by point after it is built, such as a route line
    /// laid onto terrain: the tolerance test alone leaves a straight stretch with nothing
    /// between its two ends to move.
    pub max_segment_lengths: Vec<(f64, f64, f64)>,
}

impl AdaptiveSubdivisionBuilder {
    pub fn new(tolerance: f64) -> Self {
        Self {
            tolerance,
            min_step: 0.1, // 100 ms
            force_all_samples: false,
            max_segment_lengths: Vec::new(),
        }
    }

    /// Whether the segment from `t_start` to `t_end`, `length` long, crosses a stretch
    /// whose [`Self::max_segment_lengths`] it exceeds.
    fn too_long(&self, t_start: f64, t_end: f64, length: f64) -> bool {
        self.max_segment_lengths
            .iter()
            .any(|&(a, b, max)| t_start < b && t_end > a && length > max)
    }

    /// Whether the segment from `t_start` to `t_end` crosses any of the stretches in
    /// [`Self::max_segment_lengths`].
    fn held_short(&self, t_start: f64, t_end: f64) -> bool {
        self.max_segment_lengths
            .iter()
            .any(|&(a, b, _)| t_start < b && t_end > a)
    }

    /// The first end of a stretch in [`Self::max_segment_lengths`] after `t`. Each gets a
    /// point of its own, so whatever changes there changes at a point of the line and not
    /// partway along a segment.
    fn next_stretch_end(&self, t: f64) -> f64 {
        self.max_segment_lengths
            .iter()
            .flat_map(|&(a, b, _)| [a, b])
            .filter(|&e| e > t + 1e-9)
            .fold(f64::INFINITY, f64::min)
    }

    /// Build a flat list of control points from the position property.
    /// Double precision is preserved via emulated high/low floating-point representation.
    pub fn build(
        &self,
        property: &crate::property::sampled::SampledPositionProperty,
        _reference_point: DVec3,
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
            let points: Vec<(DVec3, f32)> = property
                .samples()
                .iter()
                .map(|(t, p)| {
                    let progress =
                        ((t.seconds - start_time.seconds) / total_duration).clamp(0.0, 1.0) as f32;
                    (*p, progress)
                })
                .collect();
            return with_arc_length(&points);
        }

        let mut path_points: Vec<(DVec3, f64)> = Vec::new(); // (position, time)

        let mut current_time = start_time.seconds;
        let mut last_p = property
            .evaluate(SimulationTime::new(current_time))
            .unwrap();
        path_points.push((last_p, current_time));

        let max_step = 60.0 * 5.0; // 5-minute max step

        while current_time < stop_time.seconds {
            let next_time = (current_time + max_step)
                .min(self.next_stretch_end(current_time))
                .min(stop_time.seconds);
            let p_start = last_p;
            let p_end = property.evaluate(SimulationTime::new(next_time)).unwrap();

            self.subdivide(property, current_time, next_time, p_start, p_end, &mut path_points);

            path_points.push((p_end, next_time));
            current_time = next_time;
            last_p = p_end;
        }

        let points: Vec<(DVec3, f32)> = path_points
            .into_iter()
            .map(|(p, t)| {
                let progress = ((t - start_time.seconds) / total_duration).clamp(0.0, 1.0) as f32;
                (p, progress)
            })
            .collect();
        with_arc_length(&points)
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

        // A chord of no length is measured from its start. `1e-8` Mm² is any chord under
        // 100 m, so such chords are measured by their own half-length and halved down to
        // `min_step`. Inside a stretch held to a maximum length every chord is that short
        // on purpose, so there only a chord of truly no length (under 0.1 mm) counts.
        let degenerate_sq = if self.held_short(t_start, t_end) { 1e-20 } else { 1e-8 };
        let dist = if length_sq < degenerate_sq {
            (p_mid_true - p_start).length()
        } else {
            let t = ((p_mid_true - p_start).dot(line_vec) / length_sq).clamp(0.0, 1.0);
            let projection = p_start + line_vec * t;
            (p_mid_true - projection).length()
        };

        if dist > self.tolerance || self.too_long(t_start, t_end, line_vec.length()) {
            self.subdivide(property, t_start, t_mid, p_start, p_mid_true, points);
            points.push((p_mid_true, t_mid));
            self.subdivide(property, t_mid, t_end, p_mid_true, p_end, points);
        }
    }
}

/// Turns positions and progresses into control points, measuring how far along the path
/// each one lies.
///
/// The length is summed over the chords between consecutive points rather than along the
/// true curve. The points are placed by a chord-deviation test with a tolerance measured
/// in centimetres, so the two agree to far better than anything drawn from this could show.
fn with_arc_length(points: &[(DVec3, f32)]) -> Vec<ControlPoint> {
    let mut out = Vec::with_capacity(points.len());
    let mut travelled = 0.0_f64;
    for (i, (pos, progress)) in points.iter().enumerate() {
        if i > 0 {
            travelled += (*pos - points[i - 1].0).length();
        }
        let mut cp = ControlPoint::from_dvec3(*pos, *progress);
        cp.distance = travelled as f32;
        out.push(cp);
    }
    out
}
