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
pub enum TwoFingerMode {
    Undecided,
    PinchTwist,
    Pitch,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GestureMode {
    Idle,
    TapPending { tap_candidate: bool },
    Pan,
    DoubleTapPending,
    OneFingerZoom,
    TwoFinger { mode: TwoFingerMode },
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
    two_finger_start_angle: f32,
    two_finger_start_mid: Vec2,
    
    two_finger_start_time: std::time::Instant,
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
            two_finger_start_angle: 0.0,
            two_finger_start_mid: Vec2::ZERO,
            two_finger_start_time: std::time::Instant::now(),
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
                
                self.gesture_mode = GestureMode::TwoFinger { mode: TwoFingerMode::Undecided };
                
                let pts: Vec<Vec2> = self.active_touches.values().map(|tp| tp.current_pos).collect();
                let p1 = pts[0];
                let p2 = pts[1];
                let dist = (p1 - p2).length();
                let angle = (p2.y - p1.y).atan2(p2.x - p1.x);
                let mid = (p1 + p2) * 0.5;
                
                self.prev_two_finger_dist = dist;
                self.prev_two_finger_angle = angle;
                self.prev_two_finger_mid = mid;
                
                self.two_finger_start_dist = dist;
                self.two_finger_start_angle = angle;
                self.two_finger_start_mid = mid;
                
                self.two_finger_start_time = now;
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
                        match camera.mode {
                            crate::camera::camera::CameraMode::Free => {
                                camera.drag(tp.current_pos.x, tp.current_pos.y, screen_width, screen_height);
                            }
                            crate::camera::camera::CameraMode::Tracking => {
                                let dx = (tp.current_pos.x - tp.prev_pos.x) * 0.5;
                                let dy = (tp.current_pos.y - tp.prev_pos.y) * 0.5;
                                camera.orbit_mouse(dx, dy);
                            }
                            crate::camera::camera::CameraMode::Cockpit => {
                                let dx = (tp.current_pos.x - tp.prev_pos.x) * 0.5;
                                let dy = (tp.current_pos.y - tp.prev_pos.y) * 0.5;
                                camera.look_around(dx, dy);
                            }
                        }
                        redrew = true;
                    }
                }
                GestureMode::TwoFinger { ref mut mode } => {
                    let pts: Vec<&TouchPoint> = self.active_touches.values().collect();
                    if pts.len() >= 2 {
                        let p1_cur = pts[0].current_pos;
                        let p2_cur = pts[1].current_pos;

                        let cur_dist = (p1_cur - p2_cur).length();
                        let cur_angle = (p2_cur.y - p1_cur.y).atan2(p2_cur.x - p1_cur.x);
                        let cur_mid = (p1_cur + p2_cur) * 0.5;

                        if *mode == TwoFingerMode::Undecided {
                            let dist_delta = (cur_dist - self.two_finger_start_dist).abs();
                            
                            let mut angle_delta = cur_angle - self.two_finger_start_angle;
                            while angle_delta > std::f32::consts::PI { angle_delta -= std::f32::consts::PI * 2.0; }
                            while angle_delta < -std::f32::consts::PI { angle_delta += std::f32::consts::PI * 2.0; }
                            let twist_arc_dist = self.two_finger_start_dist * angle_delta.abs(); // Arc length of twist
                            
                            let mid_delta_y = (cur_mid.y - self.two_finger_start_mid.y).abs();
                            
                            // Determine primary intent
                            if mid_delta_y > 20.0 && mid_delta_y > dist_delta * 1.5 && mid_delta_y > twist_arc_dist * 1.5 {
                                *mode = TwoFingerMode::Pitch;
                            } else if dist_delta > 15.0 || twist_arc_dist > 15.0 {
                                *mode = TwoFingerMode::PinchTwist;
                            }
                        }

                        if *mode == TwoFingerMode::PinchTwist {
                            // 1. Pinch Zoom
                            if self.prev_two_finger_dist > 1.0 && cur_dist > 1.0 {
                                let ratio = cur_dist / self.prev_two_finger_dist;
                                let zoom_delta = ratio.log2() * 4.0;
                                if zoom_delta.abs() > 0.005 {
                                    if camera.mode != crate::camera::camera::CameraMode::Cockpit {
                                        let sensitivity_scale = if camera.mode == crate::camera::camera::CameraMode::Tracking { 0.5 } else { 1.0 };
                                        camera.zoom(zoom_delta * sensitivity_scale);
                                        redrew = true;
                                    }
                                }
                            }

                            // 2. Twist (Roll)
                            let mut angle_delta = cur_angle - self.prev_two_finger_angle;
                            while angle_delta > std::f32::consts::PI { angle_delta -= std::f32::consts::PI * 2.0; }
                            while angle_delta < -std::f32::consts::PI { angle_delta += std::f32::consts::PI * 2.0; }
                            let twist_sensitivity = 1.0;
                            if angle_delta.abs() > 0.001 {
                                if camera.mode == crate::camera::camera::CameraMode::Free {
                                    camera.roll(angle_delta * twist_sensitivity);
                                    redrew = true;
                                }
                            }
                        } else if *mode == TwoFingerMode::Pitch {
                            // 3. Pitch (Two-finger swipe)
                            let mid_delta = cur_mid - self.prev_two_finger_mid;
                            if mid_delta.y.abs() > 0.1 {
                                match camera.mode {
                                    crate::camera::camera::CameraMode::Free => {
                                        let tilt_sensitivity = 0.1;
                                        camera.pitch(mid_delta.y * tilt_sensitivity);
                                    }
                                    crate::camera::camera::CameraMode::Tracking => {
                                        let tilt_sensitivity = 0.25;
                                        camera.orbit_mouse(0.0, mid_delta.y * tilt_sensitivity);
                                    }
                                    crate::camera::camera::CameraMode::Cockpit => {
                                        let tilt_sensitivity = 0.25;
                                        camera.look_around(0.0, mid_delta.y * tilt_sensitivity);
                                    }
                                }
                                redrew = true;
                            }
                        }

                        self.prev_two_finger_dist = cur_dist;
                        self.prev_two_finger_angle = cur_angle;
                        self.prev_two_finger_mid = cur_mid;
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
                            // Instant Double tap! Use stable zoom
                            camera.zoom(4.0);
                            redrew = true;
                        }
                        self.last_tap_time = None;
                        self.last_tap_pos = None;
                    }
                    GestureMode::Pan => {
                        camera.end_drag();
                    }
                    GestureMode::TwoFinger { mode: _ } => {
                        // Check for two-finger tap (zoom out)
                        if self.active_touches.len() == 2 {
                            let duration = now.duration_since(self.two_finger_start_time);
                            let pts: Vec<Vec2> = self.active_touches.values().map(|tp| tp.current_pos).collect();
                            let start_pts: Vec<Vec2> = self.active_touches.values().map(|tp| tp.start_pos).collect();
                            
                            if duration.as_millis() < 250 {
                                let moved_1 = (pts[0] - start_pts[0]).length() > 15.0;
                                let moved_2 = (pts[1] - start_pts[1]).length() > 15.0;
                                
                                if !moved_1 && !moved_2 {
                                    camera.zoom(-4.0);
                                    redrew = true;
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
