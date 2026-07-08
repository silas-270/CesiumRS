use std::collections::HashMap;
use glam::Vec2;
use winit::event::TouchPhase;

#[derive(Debug, Clone)]
pub struct TouchPoint {
    pub prev_pos: Vec2,
    pub current_pos: Vec2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GestureMode {
    None,
    OneFingerPan,
    TwoFingerGesture,
}

pub struct TouchInterpreter {
    active_touches: HashMap<u64, TouchPoint>,
    gesture_mode: GestureMode,
    prev_two_finger_dist: f32,
    prev_two_finger_angle: f32,
    prev_two_finger_mid: Vec2,
}

impl TouchInterpreter {
    pub fn new() -> Self {
        Self {
            active_touches: HashMap::new(),
            gesture_mode: GestureMode::None,
            prev_two_finger_dist: 0.0,
            prev_two_finger_angle: 0.0,
            prev_two_finger_mid: Vec2::ZERO,
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

        match touch.phase {
            TouchPhase::Started => {
                self.active_touches.insert(touch_id, TouchPoint {
                    prev_pos: pos,
                    current_pos: pos,
                });
            }
            TouchPhase::Moved => {
                if let Some(tp) = self.active_touches.get_mut(&touch_id) {
                    tp.prev_pos = tp.current_pos;
                    tp.current_pos = pos;
                }
            }
            TouchPhase::Ended | TouchPhase::Cancelled => {
                self.active_touches.remove(&touch_id);
            }
        }

        // Transition modes based on count of active touches
        let count = self.active_touches.len();
        let old_mode = self.gesture_mode;
        
        let new_mode = match count {
            0 => GestureMode::None,
            1 => GestureMode::OneFingerPan,
            2 => GestureMode::TwoFingerGesture,
            _ => GestureMode::TwoFingerGesture, // Cap at 2 fingers for gestures
        };

        self.gesture_mode = new_mode;

        // Handle transitions
        if old_mode != new_mode {
            // End old gesture state
            if old_mode == GestureMode::OneFingerPan {
                camera.end_drag();
            }

            // Start new gesture state
            if new_mode == GestureMode::OneFingerPan {
                if let Some((_, tp)) = self.active_touches.iter().next() {
                    camera.begin_drag(tp.current_pos.x, tp.current_pos.y, screen_width, screen_height);
                }
            } else if new_mode == GestureMode::TwoFingerGesture {
                let pts: Vec<Vec2> = self.active_touches.values().map(|tp| tp.current_pos).collect();
                if pts.len() >= 2 {
                    let p1 = pts[0];
                    let p2 = pts[1];
                    self.prev_two_finger_dist = (p1 - p2).length();
                    self.prev_two_finger_angle = (p2.y - p1.y).atan2(p2.x - p1.x);
                    self.prev_two_finger_mid = (p1 + p2) * 0.5;
                }
            }
        }

        // Process continuous movement
        let mut redrew = false;
        if touch.phase == TouchPhase::Moved {
            match self.gesture_mode {
                GestureMode::OneFingerPan => {
                    if let Some(tp) = self.active_touches.get(&touch_id) {
                        camera.drag(tp.current_pos.x, tp.current_pos.y, screen_width, screen_height);
                        redrew = true;
                    }
                }
                GestureMode::TwoFingerGesture => {
                    let pts: Vec<Vec2> = self.active_touches.values().map(|tp| tp.current_pos).collect();
                    if pts.len() >= 2 {
                        let p1 = pts[0];
                        let p2 = pts[1];
                        let cur_dist = (p1 - p2).length();
                        let cur_angle = (p2.y - p1.y).atan2(p2.x - p1.x);
                        let cur_mid = (p1 + p2) * 0.5;

                        // Pinch Zoom
                        if self.prev_two_finger_dist > 1.0 && cur_dist > 1.0 {
                            let ratio = cur_dist / self.prev_two_finger_dist;
                            let zoom_delta = ratio.log2() * 4.0;
                            if zoom_delta.abs() > 0.005 {
                                camera.zoom(zoom_delta);
                                redrew = true;
                            }
                        }

                        // Twist (Rotation)
                        let mut angle_delta = cur_angle - self.prev_two_finger_angle;
                        while angle_delta > std::f32::consts::PI {
                            angle_delta -= std::f32::consts::PI * 2.0;
                        }
                        while angle_delta < -std::f32::consts::PI {
                            angle_delta += std::f32::consts::PI * 2.0;
                        }
                        let twist_sensitivity = 4.0;
                        if angle_delta.abs() > 0.001 {
                            camera.orbit_mouse(angle_delta * twist_sensitivity, 0.0);
                            redrew = true;
                        }

                        // Two-finger Swipe (Tilt)
                        let mid_delta = cur_mid - self.prev_two_finger_mid;
                        let tilt_sensitivity = 0.015;
                        if mid_delta.y.abs() > 0.1 {
                            camera.orbit_mouse(0.0, mid_delta.y * tilt_sensitivity);
                            redrew = true;
                        }

                        // Store values for next frame
                        self.prev_two_finger_dist = cur_dist;
                        self.prev_two_finger_angle = cur_angle;
                        self.prev_two_finger_mid = cur_mid;
                    }
                }
                _ => {}
            }
        }

        redrew
    }
}
