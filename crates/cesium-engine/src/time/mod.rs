#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct SimulationTime {
    /// Absolute time represented as seconds since an epoch (e.g. Unix epoch)
    pub seconds: f64,
}

impl SimulationTime {
    pub fn new(seconds: f64) -> Self {
        Self { seconds }
    }
}

impl std::ops::Add<f64> for SimulationTime {
    type Output = Self;

    fn add(self, rhs: f64) -> Self::Output {
        Self { seconds: self.seconds + rhs }
    }
}

impl std::ops::Sub<f64> for SimulationTime {
    type Output = Self;

    fn sub(self, rhs: f64) -> Self::Output {
        Self { seconds: self.seconds - rhs }
    }
}

impl std::ops::Sub for SimulationTime {
    type Output = f64;

    fn sub(self, rhs: Self) -> Self::Output {
        self.seconds - rhs.seconds
    }
}

#[derive(Debug, Clone)]
pub struct Clock {
    pub start_time: SimulationTime,
    pub stop_time: SimulationTime,
    pub current_time: SimulationTime,
    pub multiplier: f64,
    pub is_playing: bool,
    pub should_loop: bool,
}

impl Clock {
    pub fn new(start_time: SimulationTime, stop_time: SimulationTime) -> Self {
        Self {
            start_time,
            stop_time,
            current_time: start_time,
            multiplier: 1.0,
            is_playing: true,
            should_loop: false,
        }
    }

    /// Advance the clock by `real_dt` (real-world seconds elapsed)
    pub fn tick(&mut self, real_dt: f64) {
        if !self.is_playing {
            return;
        }

        self.current_time.seconds += real_dt * self.multiplier;

        if self.multiplier > 0.0 {
            if self.current_time.seconds >= self.stop_time.seconds {
                if self.should_loop {
                    self.current_time.seconds = self.start_time.seconds + (self.current_time.seconds - self.stop_time.seconds) % (self.stop_time.seconds - self.start_time.seconds).max(0.001);
                } else {
                    self.current_time.seconds = self.stop_time.seconds;
                    self.is_playing = false;
                }
            }
        } else if self.multiplier < 0.0 {
            if self.current_time.seconds <= self.start_time.seconds {
                if self.should_loop {
                    self.current_time.seconds = self.stop_time.seconds - (self.start_time.seconds - self.current_time.seconds) % (self.stop_time.seconds - self.start_time.seconds).max(0.001);
                } else {
                    self.current_time.seconds = self.start_time.seconds;
                    self.is_playing = false;
                }
            }
        }
    }
}
