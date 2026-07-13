use std::collections::BTreeMap;
use glam::Vec2;
use winit::event::TouchPhase;
use std::time::Instant;

#[derive(Debug, Clone)]
pub struct TouchPoint {
    pub start_pos: Vec2,
    pub prev_pos: Vec2,
    pub current_pos: Vec2,
    pub start_time: Instant,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GestureMode {
    Idle,
    TapPending { tap_candidate: bool },
    Pan,
    DoubleTapPending,
    OneFingerZoom,
    TwoFinger,
}

pub struct TouchInterpreter {
    active_touches: BTreeMap<u64, TouchPoint>,
    gesture_mode: GestureMode,
    
    // Pan state
    pan_deadband_crossed: bool,

    // Tap/Double tap state
    last_tap_time: Option<Instant>,
    last_tap_pos: Option<Vec2>,
    
    // Two finger state
    prev_two_finger_dist: f32,
    prev_two_finger_angle: f32,
    prev_two_finger_mid: Vec2,
    two_finger_start_dist: f32,
    two_finger_start_time: Instant,
    
    // Twist inertia
    last_twist_time: Instant,
    twist_velocity_samples: [(f32, Instant); 6],
    twist_velocity_count: usize,
}

impl TouchInterpreter {
    pub fn new() -> Self {
        Self {
            active_touches: BTreeMap::new(),
            gesture_mode: GestureMode::Idle,
            
            pan_deadband_crossed: false,
            
            last_tap_time: None,
            last_tap_pos: None,
            
            prev_two_finger_dist: 0.0,
            prev_two_finger_angle: 0.0,
            prev_two_finger_mid: Vec2::ZERO,
            two_finger_start_dist: 0.0,
            two_finger_start_time: Instant::now(),
            
            last_twist_time: Instant::now(),
            twist_velocity_samples: [(0.0, Instant::now()); 6],
            twist_velocity_count: 0,
        }
    }

    pub fn handle_touch_event(
        &mut self,
        touch: &winit::event::Touch,
        camera: &mut crate::camera::camera::Camera,
        screen_width: f32,
        screen_height: f32,
    ) -> bool {
        let touch_id = touch.id;
        let pos = Vec2::new(touch.location.x as f32, touch.location.y as f32);
        let now = Instant::now();
        let mut redrew = false;

        match touch.phase {
            TouchPhase::Started => {
                camera.cancel_all_inertia();
                self.active_touches.insert(touch_id, TouchPoint {
                    start_pos: pos,
                    prev_pos: pos,
                    current_pos: pos,
                    start_time: now,
                });
            }
            TouchPhase::Moved => {
                if let Some(tp) = self.active_touches.get_mut(&touch_id) {
                    tp.prev_pos = tp.current_pos;
                    tp.current_pos = pos;
                }
            }
            TouchPhase::Ended | TouchPhase::Cancelled => {
                // Handled later for tap detection before removal
            }
        }

        let count = self.active_touches.len();

        // Mode Transitions for Touch Down
        if touch.phase == TouchPhase::Started {
            if count == 1 {
                // Check if this is the start of a Double-Tap-And-Drag
                if let (Some(last_time), Some(last_pos)) = (self.last_tap_time, self.last_tap_pos) {
                    if now.duration_since(last_time).as_millis() < 300 && (pos - last_pos).length() < 30.0 {
                        self.gesture_mode = GestureMode::DoubleTapPending;
                    } else {
                        self.gesture_mode = GestureMode::TapPending { tap_candidate: true };
                        self.pan_deadband_crossed = false;
                    }
                } else {
                    self.gesture_mode = GestureMode::TapPending { tap_candidate: true };
                    self.pan_deadband_crossed = false;
                }
            } else if count == 2 {
                if self.gesture_mode == GestureMode::Pan {
                    camera.cancel_all_inertia(); // End the pan without momentum when adding second finger
                }
                
                self.gesture_mode = GestureMode::TwoFinger;
                
                let pts: Vec<Vec2> = self.active_touches.values().map(|tp| tp.current_pos).collect();
                let p1 = pts[0];
                let p2 = pts[1];
                self.prev_two_finger_dist = (p1 - p2).length();
                self.prev_two_finger_angle = (p2.y - p1.y).atan2(p2.x - p1.x);
                self.prev_two_finger_mid = (p1 + p2) * 0.5;
                
                self.two_finger_start_dist = self.prev_two_finger_dist;
                self.two_finger_start_time = now;
                self.twist_velocity_count = 0;
            } else if count > 2 {
                if self.gesture_mode == GestureMode::Pan {
                    camera.end_drag();
                }
                self.gesture_mode = GestureMode::Idle;
            }
        }

        // Process Moves
        if touch.phase == TouchPhase::Moved {
            match self.gesture_mode {
                GestureMode::DoubleTapPending => {
                    if let Some(tp) = self.active_touches.get(&touch_id) {
                        let dist = (tp.current_pos - tp.start_pos).length();
                        if dist > 8.0 {
                            // Deadband crossed for 1-finger zoom
                            self.gesture_mode = GestureMode::OneFingerZoom;
                            self.last_tap_time = None;
                            self.last_tap_pos = None;
                        }
                    }
                }
                GestureMode::OneFingerZoom => {
                    if let Some(tp) = self.active_touches.get(&touch_id) {
                        let dy = tp.current_pos.y - tp.prev_pos.y;
                        // Map vertical screen movement to zoom delta
                        let zoom_delta = -dy * 0.01;
                        if zoom_delta.abs() > 0.001 {
                            camera.zoom(zoom_delta);
                            redrew = true;
                        }
                    }
                }
                GestureMode::TapPending { tap_candidate: _ } => {
                    if let Some(tp) = self.active_touches.get(&touch_id) {
                        let dist = (tp.current_pos - tp.start_pos).length();
                        if dist > 8.0 {
                            self.pan_deadband_crossed = true;
                            self.gesture_mode = GestureMode::Pan;
                            // FIX: Begin drag from *current* pos to prevent snapping
                            camera.begin_drag(tp.current_pos.x, tp.current_pos.y, screen_width, screen_height);
                        }
                    }
                }
                GestureMode::Pan => {
                    if let Some(tp) = self.active_touches.get(&touch_id) {
                        camera.drag(tp.current_pos.x, tp.current_pos.y, screen_width, screen_height);
                        redrew = true;
                    }
                }
                GestureMode::TwoFinger => {
                    let pts: Vec<&TouchPoint> = self.active_touches.values().collect();
                    if pts.len() >= 2 {
                        let p1_cur = pts[0].current_pos;
                        let p2_cur = pts[1].current_pos;
                        let p1_prev = pts[0].prev_pos;
                        let p2_prev = pts[1].prev_pos;

                        let cur_dist = (p1_cur - p2_cur).length();
                        let cur_angle = (p2_cur.y - p1_cur.y).atan2(p2_cur.x - p1_cur.x);
                        let cur_mid = (p1_cur + p2_cur) * 0.5;

                        // 1. Pinch Zoom
                        if self.prev_two_finger_dist > 1.0 && cur_dist > 1.0 {
                            let ratio = cur_dist / self.prev_two_finger_dist;
                            let zoom_delta = ratio.log2() * 4.0;
                            if zoom_delta.abs() > 0.005 {
                                camera.zoom_toward_point(zoom_delta, cur_mid.x, cur_mid.y, screen_width, screen_height);
                                redrew = true;
                            }
                        }

                        // 2. Twist
                        let mut angle_delta = cur_angle - self.prev_two_finger_angle;
                        while angle_delta > std::f32::consts::PI { angle_delta -= std::f32::consts::PI * 2.0; }
                        while angle_delta < -std::f32::consts::PI { angle_delta += std::f32::consts::PI * 2.0; }
                        if angle_delta.abs() > 0.001 {
                            camera.twist_view(-angle_delta);
                            redrew = true;
                            
                            let dt = (now - self.last_twist_time).as_secs_f32();
                            if dt > 0.001 {
                                let velocity = -angle_delta / dt;
                                let idx = self.twist_velocity_count % 6;
                                self.twist_velocity_samples[idx] = (velocity, now);
                                self.twist_velocity_count += 1;
                            }
                        }

                        // 3. Pan (Incremental)
                        let mid_delta = cur_mid - self.prev_two_finger_mid;
                        if mid_delta.length() > 0.1 {
                            camera.begin_drag(self.prev_two_finger_mid.x, self.prev_two_finger_mid.y, screen_width, screen_height);
                            camera.drag(cur_mid.x, cur_mid.y, screen_width, screen_height);
                            // We don't call end_drag() because we don't want to trigger 2-finger pan inertia here
                            // Let the ring buffer collect samples for when they release to 1-finger.
                            redrew = true;
                        }

                        // 4. Pitch (Tilt)
                        let p1_delta = p1_cur - p1_prev;
                        let p2_delta = p2_cur - p2_prev;
                        // If moving in same vertical direction and primarily vertical
                        if p1_delta.y * p2_delta.y > 0.0 {
                            let avg_y_delta = (p1_delta.y + p2_delta.y) * 0.5;
                            let avg_x_delta = (p1_delta.x + p2_delta.x).abs() * 0.5;
                            let dist_delta = (cur_dist - self.prev_two_finger_dist).abs();
                            
                            // Condition: More vertical movement than horizontal or pinch
                            if avg_y_delta.abs() > avg_x_delta && avg_y_delta.abs() > dist_delta * 2.0 {
                                camera.pitch(-avg_y_delta * 0.1);
                                redrew = true;
                            }
                        }

                        self.prev_two_finger_dist = cur_dist;
                        self.prev_two_finger_angle = cur_angle;
                        self.prev_two_finger_mid = cur_mid;
                        self.last_twist_time = now;
                    }
                }
                _ => {}
            }
        }

        // Handle Lifts (Taps & End gestures)
        if touch.phase == TouchPhase::Ended || touch.phase == TouchPhase::Cancelled {
            if let Some(tp) = self.active_touches.get(&touch_id) {
                match self.gesture_mode {
                    GestureMode::TapPending { tap_candidate } => {
                        if tap_candidate && !self.pan_deadband_crossed {
                            let duration = now.duration_since(tp.start_time);
                            if duration.as_millis() < 250 {
                                // Tap detected
                                self.last_tap_time = Some(now);
                                self.last_tap_pos = Some(tp.current_pos);
                            }
                        }
                    }
                    GestureMode::DoubleTapPending => {
                        let duration = now.duration_since(tp.start_time);
                        if duration.as_millis() < 250 {
                            // Instant Double tap! Use smooth zoom
                            camera.animate_zoom_toward_point(4.0, tp.current_pos.x, tp.current_pos.y, screen_width, screen_height);
                            redrew = true;
                        }
                        self.last_tap_time = None;
                        self.last_tap_pos = None;
                    }
                    GestureMode::Pan => {
                        camera.end_drag();
                    }
                    GestureMode::TwoFinger => {
                        // Check for two-finger tap (zoom out)
                        if self.active_touches.len() == 2 {
                            let duration = now.duration_since(self.two_finger_start_time);
                            let pts: Vec<Vec2> = self.active_touches.values().map(|tp| tp.current_pos).collect();
                            let start_pts: Vec<Vec2> = self.active_touches.values().map(|tp| tp.start_pos).collect();
                            
                            if duration.as_millis() < 250 {
                                let moved_1 = (pts[0] - start_pts[0]).length() > 15.0;
                                let moved_2 = (pts[1] - start_pts[1]).length() > 15.0;
                                
                                if !moved_1 && !moved_2 {
                                    let mid = (pts[0] + pts[1]) * 0.5;
                                    camera.animate_zoom_toward_point(-4.0, mid.x, mid.y, screen_width, screen_height);
                                    redrew = true;
                                }
                            }
                            
                            // Handle Twist Inertia
                            let window = std::time::Duration::from_millis(100);
                            let n = self.twist_velocity_count.min(6);
                            let mut valid_samples = 0;
                            let mut velocity_sum = 0.0;
                            
                            for i in 0..n {
                                let idx = if self.twist_velocity_count > 6 {
                                    (self.twist_velocity_count - n + i) % 6
                                } else {
                                    i
                                };
                                let (vel, t) = self.twist_velocity_samples[idx];
                                if now.duration_since(t) <= window {
                                    velocity_sum += vel;
                                    valid_samples += 1;
                                }
                            }
                            
                            if valid_samples > 0 {
                                let avg_vel = velocity_sum / valid_samples as f32;
                                if avg_vel.abs() > 0.5 { // threshold
                                    camera.twist_inertia_active = true;
                                    camera.twist_inertia_velocity = avg_vel;
                                }
                            }
                        }
                    }
                    _ => {}
                }
            }
            
            self.active_touches.remove(&touch_id);
            
            if self.active_touches.is_empty() {
                self.gesture_mode = GestureMode::Idle;
            } else if self.active_touches.len() == 1 {
                // We transitioned from 2 fingers to 1 finger.
                // Reset the remaining touch to prevent snapping.
                let remaining_id = *self.active_touches.keys().next().unwrap();
                let remaining_tp = self.active_touches.get_mut(&remaining_id).unwrap();
                
                remaining_tp.start_pos = remaining_tp.current_pos;
                remaining_tp.prev_pos = remaining_tp.current_pos;
                
                self.gesture_mode = GestureMode::Pan;
                self.pan_deadband_crossed = true; // prevent tap if coming back from two finger
                
                // Immediately start panning from current location
                camera.begin_drag(remaining_tp.current_pos.x, remaining_tp.current_pos.y, screen_width, screen_height);
            }
        }

        redrew
    }
}
