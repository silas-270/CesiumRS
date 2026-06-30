import re

with open("src/render/wgpu_state.rs", "r") as f:
    content = f.read()

# 1. Add DebugVertex and imports
imports = """use winit::window::Window;
use wgpu::util::DeviceExt;
use std::sync::Arc;
use glam::{Mat4, Vec3, Vec4, Quat, EulerRot};
use crate::math::geometry::{Vertex, Ellipsoid};
use crate::math::camera::Camera;
use crate::math::quadtree::QuadtreeManager;
use egui_wgpu::Renderer as EguiRenderer;
use egui_winit::State as EguiState;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub struct DebugVertex {
    pub position: [f32; 3],
    pub color: [f32; 4],
}

impl DebugVertex {
    pub fn desc() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<DebugVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &[
                wgpu::VertexAttribute {
                    offset: 0,
                    shader_location: 0,
                    format: wgpu::VertexFormat::Float32x3,
                },
                wgpu::VertexAttribute {
                    offset: std::mem::size_of::<[f32; 3]>() as wgpu::BufferAddress,
                    shader_location: 1,
                    format: wgpu::VertexFormat::Float32x4,
                },
            ],
        }
    }
}

fn get_frustum_corners(inv_view_proj: Mat4) -> [Vec3; 8] {
    let mut corners = [Vec3::ZERO; 8];
    let ndc_corners = [
        Vec3::new(-1.0, -1.0, 0.0), // Near
        Vec3::new(1.0, -1.0, 0.0),
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(-1.0, 1.0, 0.0),
        Vec3::new(-1.0, -1.0, 1.0), // Far
        Vec3::new(1.0, -1.0, 1.0),
        Vec3::new(1.0, 1.0, 1.0),
        Vec3::new(-1.0, 1.0, 1.0),
    ];
    for i in 0..8 {
        corners[i] = inv_view_proj.project_point3(ndc_corners[i]);
    }
    corners
}

fn append_aabb_lines(vertices: &mut Vec<DebugVertex>, center: Vec3, radius: f32, color: [f32; 4]) {
    let min = center - Vec3::splat(radius);
    let max = center + Vec3::splat(radius);
    let corners = [
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(max.x, max.y, max.z),
        Vec3::new(min.x, max.y, max.z),
    ];
    let indices = [
        0, 1, 1, 2, 2, 3, 3, 0, // bottom
        4, 5, 5, 6, 6, 7, 7, 4, // top
        0, 4, 1, 5, 2, 6, 3, 7  // sides
    ];
    for &i in &indices {
        vertices.push(DebugVertex { position: corners[i].into(), color });
    }
}

fn append_frustum_lines(vertices: &mut Vec<DebugVertex>, corners: &[Vec3; 8], color: [f32; 4]) {
    let indices = [
        0, 1, 1, 2, 2, 3, 3, 0, // near
        4, 5, 5, 6, 6, 7, 7, 4, // far
        0, 4, 1, 5, 2, 6, 3, 7  // connections
    ];
    for &i in &indices {
        vertices.push(DebugVertex { position: corners[i].into(), color });
    }
}
"""

content = re.sub(
    r"^use winit::window::Window;.*?use egui_winit::State as EguiState;\n",
    imports,
    content,
    flags=re.DOTALL | re.MULTILINE
)

# 2. Update WgpuState struct
struct_update = """    pub camera: Camera,
    pub debug_mode: bool,
    pub debug_camera: Camera,
    debug_pipeline: wgpu::RenderPipeline,
    debug_vertex_buffer: wgpu::Buffer,
    num_debug_vertices: u32,
    pub egui_ctx: egui::Context,"""

content = re.sub(
    r"    pub camera: Camera,\n    pub egui_ctx: egui::Context,",
    struct_update,
    content
)

# 3. Update WgpuState::new() - Debug buffers and pipeline
pipeline_init = """        let debug_vertex_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("Debug Vertex Buffer"),
            size: std::mem::size_of::<DebugVertex>() as u64 * 100000,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let debug_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Debug Pipeline"),
            layout: Some(&render_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: "vs_debug",
                buffers: &[DebugVertex::desc()],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: "fs_debug",
                targets: &[Some(wgpu::ColorTargetState {
                    format: config.format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::LineList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Cw,
                cull_mode: None,
                polygon_mode: wgpu::PolygonMode::Line,
                unclipped_depth: false,
                conservative: false,
            },
            depth_stencil: wireframe_depth_stencil.clone(),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let ellipsoid = Ellipsoid::generate(30, 60);"""

content = re.sub(
    r"        let ellipsoid = Ellipsoid::generate\(30, 60\);",
    pipeline_init,
    content
)

# 4. WgpuState return initialization
state_init = """            camera,
            debug_mode: false,
            debug_camera: Camera::new(Vec3::new(0.0, 0.0, 25.0), Vec3::ZERO),
            debug_pipeline,
            debug_vertex_buffer,
            num_debug_vertices: 0,
            egui_ctx,"""
            
content = re.sub(
    r"            camera,\n            egui_ctx,",
    state_init,
    content
)

# 5. render method
render_start = """    pub fn render(&mut self) -> Result<(), wgpu::SurfaceError> {
        let aspect_ratio = self.size.width as f32 / self.size.height as f32;
        let main_view_proj = self.camera.get_projection_matrix(aspect_ratio) * self.camera.get_view_matrix();
        let render_camera = if self.debug_mode { &self.debug_camera } else { &self.camera };

        let camera_pos = self.camera.global_transform().0;
        self.quadtree_manager.update(camera_pos, main_view_proj);

        self.camera_uniform.update_matrix(render_camera.get_view_matrix(), render_camera.get_projection_matrix(aspect_ratio));
        self.queue.write_buffer(&self.camera_buffer, 0, bytemuck::cast_slice(&[self.camera_uniform]));

        let mut debug_vertices = Vec::new();
        if self.debug_mode {
            let inv_view_proj = main_view_proj.inverse();
            let frustum_corners = get_frustum_corners(inv_view_proj);
            append_frustum_lines(&mut debug_vertices, &frustum_corners, [1.0, 1.0, 0.0, 1.0]); // Yellow
            
            for (_tile_id, center, radius) in self.quadtree_manager.get_visible_tiles() {
                append_aabb_lines(&mut debug_vertices, center, radius, [0.0, 1.0, 0.0, 1.0]); // Green
            }
        }
        self.num_debug_vertices = debug_vertices.len() as u32;
        if self.num_debug_vertices > 0 {
            self.queue.write_buffer(&self.debug_vertex_buffer, 0, bytemuck::cast_slice(&debug_vertices));
        }

        let output = self.surface.get_current_texture()?;"""

content = re.sub(
    r"    pub fn render\(&mut self\) -> Result<\(\), wgpu::SurfaceError> \{.*?let output = self\.surface\.get_current_texture\(\)\?;",
    render_start,
    content,
    flags=re.DOTALL
)

# 6. debug rendering pass
draw_calls = """            // Draw wireframe overlay
            render_pass.set_pipeline(&self.wireframe_pipeline);
            render_pass.draw_indexed(0..self.num_indices, 0, 0..1);

            if self.debug_mode && self.num_debug_vertices > 0 {
                render_pass.set_pipeline(&self.debug_pipeline);
                render_pass.set_vertex_buffer(0, self.debug_vertex_buffer.slice(..));
                render_pass.draw(0..self.num_debug_vertices, 0..1);
            }
        }"""
        
content = re.sub(
    r"            // Draw wireframe overlay\n            render_pass\.set_pipeline\(&self\.wireframe_pipeline\);\n            render_pass\.draw_indexed\(0\.\.self\.num_indices, 0, 0\.\.1\);\n        \}",
    draw_calls,
    content
)

# 7. Egui UI
egui_ui = """        let full_output = self.egui_ctx.run(raw_input, |ctx| {
            egui::Window::new("Debug")
                .resizable(false)
                .show(ctx, |ui| {
                    ui.label(format!("Altitude: {:.4}", self.camera.altitude()));
                    
                    let mut is_debug = self.debug_mode;
                    if ui.checkbox(&mut is_debug, "Debug Mode (Dual Camera)").changed() {
                        self.debug_mode = is_debug;
                        if is_debug {
                            let main_pos = self.camera.local_pos;
                            let forward = self.camera.local_ori * Vec3::Z;
                            let up = self.camera.local_ori * Vec3::Y;
                            self.debug_camera = Camera::new(main_pos - forward * 5.0 + up * 5.0, main_pos);
                        }
                    }

                    if self.debug_mode {
                        ui.separator();
                        ui.label("Main Camera Override:");
                        ui.horizontal(|ui| {
                            ui.label("Pos:");
                            ui.add(egui::DragValue::new(&mut self.camera.local_pos.x).speed(0.1));
                            ui.add(egui::DragValue::new(&mut self.camera.local_pos.y).speed(0.1));
                            ui.add(egui::DragValue::new(&mut self.camera.local_pos.z).speed(0.1));
                        });
                        
                        let (yaw, pitch, roll) = self.camera.local_ori.to_euler(EulerRot::YXZ);
                        let mut yaw_deg = yaw.to_degrees();
                        let mut pitch_deg = pitch.to_degrees();
                        let mut roll_deg = roll.to_degrees();

                        ui.horizontal(|ui| {
                            ui.label("Rot:");
                            ui.add(egui::DragValue::new(&mut pitch_deg).speed(1.0).prefix("P: "));
                            ui.add(egui::DragValue::new(&mut yaw_deg).speed(1.0).prefix("Y: "));
                            ui.add(egui::DragValue::new(&mut roll_deg).speed(1.0).prefix("R: "));
                        });
                        
                        if pitch_deg != pitch.to_degrees() || yaw_deg != yaw.to_degrees() || roll_deg != roll.to_degrees() {
                            self.camera.local_ori = Quat::from_euler(EulerRot::YXZ, yaw_deg.to_radians(), pitch_deg.to_radians(), roll_deg.to_radians());
                        }
                    }

                    ui.separator();
                    let visible_tiles = self.quadtree_manager.get_visible_tiles();
                    ui.label(format!("Visible Tiles: {}", visible_tiles.len()));
                    
                    ui.separator();
                    ui.label("First 5 Visible Tiles:");
                    for (tile, _, _) in visible_tiles.iter().take(5) {
                        ui.label(format!("  Z: {}, X: {}, Y: {}", tile.z, tile.x, tile.y));
                    }
                });
        });"""

content = re.sub(
    r"        let full_output = self\.egui_ctx\.run\(raw_input, \|ctx\| \{.*?                \}\);\n        \}\);",
    egui_ui,
    content,
    flags=re.DOTALL
)

with open("src/render/wgpu_state.rs", "w") as f:
    f.write(content)
