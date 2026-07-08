use glam::DVec3;
use crate::time::SimulationTime;
use crate::math::interpolation;

/// A generic trait for properties that can change over time.
pub trait Property<T> {
    fn evaluate(&self, time: SimulationTime) -> Option<T>;
}

/// A property whose value never changes.
pub struct ConstantProperty<T> {
    value: T,
}

impl<T: Clone> ConstantProperty<T> {
    pub fn new(value: T) -> Self {
        Self { value }
    }
}

impl<T: Clone> Property<T> for ConstantProperty<T> {
    fn evaluate(&self, _time: SimulationTime) -> Option<T> {
        Some(self.value.clone())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterpolationAlgorithm {
    Linear,
    CatmullRom,
}

pub struct SampledPositionProperty {
    samples: Vec<(SimulationTime, DVec3)>,
    pub algorithm: InterpolationAlgorithm,
}

impl SampledPositionProperty {
    pub fn new() -> Self {
        Self {
            samples: Vec::new(),
            algorithm: InterpolationAlgorithm::Linear,
        }
    }

    pub fn with_algorithm(mut self, algorithm: InterpolationAlgorithm) -> Self {
        self.algorithm = algorithm;
        self
    }

    pub fn add_sample(&mut self, time: SimulationTime, position: DVec3) {
        // Find index to maintain sorted order
        let idx = self.samples.binary_search_by(|(t, _)| t.partial_cmp(&time).unwrap());
        match idx {
            Ok(i) => self.samples[i] = (time, position), // Replace existing sample
            Err(i) => self.samples.insert(i, (time, position)), // Insert at correct sorted position
        }
    }

    pub fn start_time(&self) -> Option<SimulationTime> {
        self.samples.first().map(|(t, _)| *t)
    }

    pub fn stop_time(&self) -> Option<SimulationTime> {
        self.samples.last().map(|(t, _)| *t)
    }

    pub fn samples(&self) -> &[(SimulationTime, DVec3)] {
        &self.samples
    }
}

impl Property<DVec3> for SampledPositionProperty {
    fn evaluate(&self, time: SimulationTime) -> Option<DVec3> {
        if self.samples.is_empty() {
            return None;
        }

        if self.samples.len() == 1 {
            return Some(self.samples[0].1);
        }

        let first = self.samples.first().unwrap();
        let last = self.samples.last().unwrap();

        if time.seconds <= first.0.seconds {
            return Some(first.1);
        }
        if time.seconds >= last.0.seconds {
            return Some(last.1);
        }

        // Find the bounding samples
        let idx = self.samples.binary_search_by(|(t, _)| t.partial_cmp(&time).unwrap());
        match idx {
            Ok(i) => Some(self.samples[i].1), // Exact match
            Err(i) => {
                let idx1 = i - 1;
                let idx2 = i;

                let (t1, p1) = self.samples[idx1];
                let (t2, p2) = self.samples[idx2];
                let dt = t2.seconds - t1.seconds;
                let t = (time.seconds - t1.seconds) / dt;

                match self.algorithm {
                    InterpolationAlgorithm::Linear => {
                        Some(interpolation::linear_dvec3(p1, p2, t))
                    }
                    InterpolationAlgorithm::CatmullRom => {
                        // We need 4 points for Catmull-Rom (p0, p1, p2, p3)
                        let p0 = if idx1 > 0 { self.samples[idx1 - 1].1 } else { p1 };
                        let p3 = if idx2 + 1 < self.samples.len() { self.samples[idx2 + 1].1 } else { p2 };
                        
                        Some(interpolation::catmull_rom_dvec3(p0, p1, p2, p3, t))
                    }
                }
            }
        }
    }
}

pub struct SampledScalarProperty {
    samples: Vec<(SimulationTime, f64)>,
    pub algorithm: InterpolationAlgorithm,
}

impl SampledScalarProperty {
    pub fn new() -> Self {
        Self {
            samples: Vec::new(),
            algorithm: InterpolationAlgorithm::Linear,
        }
    }

    pub fn with_algorithm(mut self, algorithm: InterpolationAlgorithm) -> Self {
        self.algorithm = algorithm;
        self
    }

    pub fn add_sample(&mut self, time: SimulationTime, value: f64) {
        let idx = self.samples.binary_search_by(|(t, _)| t.partial_cmp(&time).unwrap());
        match idx {
            Ok(i) => self.samples[i] = (time, value),
            Err(i) => self.samples.insert(i, (time, value)),
        }
    }
}

impl Property<f64> for SampledScalarProperty {
    fn evaluate(&self, time: SimulationTime) -> Option<f64> {
        if self.samples.is_empty() {
            return None;
        }

        if self.samples.len() == 1 {
            return Some(self.samples[0].1);
        }

        let first = self.samples.first().unwrap();
        let last = self.samples.last().unwrap();

        if time.seconds <= first.0.seconds {
            return Some(first.1);
        }
        if time.seconds >= last.0.seconds {
            return Some(last.1);
        }

        let idx = self.samples.binary_search_by(|(t, _)| t.partial_cmp(&time).unwrap());
        match idx {
            Ok(i) => Some(self.samples[i].1),
            Err(i) => {
                let idx1 = i - 1;
                let idx2 = i;

                let (t1, p1) = self.samples[idx1];
                let (t2, p2) = self.samples[idx2];
                let dt = t2.seconds - t1.seconds;
                let t = (time.seconds - t1.seconds) / dt;

                match self.algorithm {
                    InterpolationAlgorithm::Linear => {
                        Some(interpolation::linear_f64(p1, p2, t))
                    }
                    InterpolationAlgorithm::CatmullRom => {
                        let p0 = if idx1 > 0 { self.samples[idx1 - 1].1 } else { p1 };
                        let p3 = if idx2 + 1 < self.samples.len() { self.samples[idx2 + 1].1 } else { p2 };
                        
                        Some(interpolation::catmull_rom_f64(p0, p1, p2, p3, t))
                    }
                }
            }
        }
    }
}
