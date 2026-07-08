use winit::dpi::PhysicalPosition;
use winit::event::{DeviceId, ElementState, MouseButton, WindowEvent};

#[derive(Clone, Debug)]
pub enum SimulatedAction {
    Drag {
        start_x: f64,
        start_y: f64,
        end_x: f64,
        end_y: f64,
        frames: u32,
        current_frame: u32,
    },
    Wait {
        frames: u32,
        current_frame: u32,
    },
}

pub struct Simulator {
    pub actions: Vec<SimulatedAction>,
}

impl Simulator {
    pub fn parse(action_string: &str) -> Self {
        let mut actions = Vec::new();
        // Syntax: drag:0,0->100,100:10;wait:5
        for part in action_string.split(';') {
            let part = part.trim();
            if part.is_empty() {
                continue;
            }
            if let Some(params) = part.strip_prefix("drag:") {
                let segments: Vec<&str> = params.split(':').collect();
                if segments.len() == 2 {
                    let frames: u32 = segments[1].parse().unwrap_or(1);
                    let coords: Vec<&str> = segments[0].split("->").collect();
                    if coords.len() == 2 {
                        let start_coords: Vec<&str> = coords[0].split(',').collect();
                        let end_coords: Vec<&str> = coords[1].split(',').collect();
                        if start_coords.len() == 2 && end_coords.len() == 2 {
                            actions.push(SimulatedAction::Drag {
                                start_x: start_coords[0].parse().unwrap_or(0.0),
                                start_y: start_coords[1].parse().unwrap_or(0.0),
                                end_x: end_coords[0].parse().unwrap_or(0.0),
                                end_y: end_coords[1].parse().unwrap_or(0.0),
                                frames,
                                current_frame: 0,
                            });
                        }
                    }
                }
            } else if let Some(frames_str) = part.strip_prefix("wait:") {
                let frames = frames_str.parse().unwrap_or(1);
                actions.push(SimulatedAction::Wait {
                    frames,
                    current_frame: 0,
                });
            }
        }
        Self { actions }
    }

    pub fn pump_events(&mut self) -> Vec<WindowEvent> {
        let mut events = Vec::new();
        let device_id = DeviceId::dummy();

        if let Some(action) = self.actions.first_mut() {
            match action {
                SimulatedAction::Drag {
                    start_x,
                    start_y,
                    end_x,
                    end_y,
                    frames,
                    current_frame,
                } => {
                    if *current_frame == 0 {
                        events.push(WindowEvent::CursorMoved {
                            device_id,
                            position: PhysicalPosition::new(*start_x, *start_y),
                        });
                        events.push(WindowEvent::MouseInput {
                            device_id,
                            state: ElementState::Pressed,
                            button: MouseButton::Left,
                        });
                    }

                    *current_frame += 1;

                    let t = *current_frame as f64 / *frames as f64;
                    let cur_x = *start_x + (*end_x - *start_x) * t;
                    let cur_y = *start_y + (*end_y - *start_y) * t;

                    events.push(WindowEvent::CursorMoved {
                        device_id,
                        position: PhysicalPosition::new(cur_x, cur_y),
                    });

                    if *current_frame >= *frames {
                        events.push(WindowEvent::MouseInput {
                            device_id,
                            state: ElementState::Released,
                            button: MouseButton::Left,
                        });
                    }
                }
                SimulatedAction::Wait {
                    frames: _,
                    current_frame,
                } => {
                    *current_frame += 1;
                }
            }
        }

        // Remove finished actions
        self.actions.retain(|a| match a {
            SimulatedAction::Drag {
                frames,
                current_frame,
                ..
            } => current_frame < frames,
            SimulatedAction::Wait {
                frames,
                current_frame,
                ..
            } => current_frame < frames,
        });

        events
    }
}
