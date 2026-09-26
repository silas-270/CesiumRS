use crate::render::celestial;
use glam::Mat4;

#[repr(C)]
#[derive(Debug, Copy, Clone, bytemuck::Pod, bytemuck::Zeroable)]
pub(super) struct CameraUniform {
    view_proj: [[f32; 4]; 4],
    inv_view_proj: [[f32; 4]; 4],
    camera_pos: [f32; 4],
    /// `[altitude_scalar, saturation, contrast, brightness]`.
    ///
    /// The first of these is named for the sun for historical reasons but has nothing to
    /// do with it: it runs from 1.0 on the runway to 0.0 at cruise, and it is what drains
    /// the world of colour and form as the flight climbs. Daylight lives in the three
    /// fields below.
    sun_params: [f32; 4],
    /// `xyz` is a unit vector toward the sun in world space; `w` is the sine of its
    /// elevation above the local horizon.
    sun_dir: [f32; 4],
    /// `xyz` toward the moon; `w` is the lit fraction of its disc.
    moon_dir: [f32; 4],
    /// `rgb` is the key light's hue, normalised; `w` is its strength.
    light_color: [f32; 4],
}

impl CameraUniform {
    pub(super) fn new() -> Self {
        Self {
            view_proj: glam::Mat4::IDENTITY.to_cols_array_2d(),
            inv_view_proj: glam::Mat4::IDENTITY.to_cols_array_2d(),
            camera_pos: [0.0; 4],
            sun_params: [1.0, 0.0, 0.0, 0.0],
            sun_dir: [0.0, 1.0, 0.0, 1.0],
            moon_dir: [0.0, -1.0, 0.0, 0.0],
            light_color: [1.0, 1.0, 1.0, 1.0],
        }
    }

    /// The camera-relative view-projection the shaders see this frame.
    pub(super) fn view_proj(&self) -> Mat4 {
        Mat4::from_cols_array_2d(&self.view_proj)
    }

    /// Sine of the sun's elevation, as carried in `sun_dir.w`.
    #[allow(dead_code)]
    pub(super) fn sun_elevation(&self) -> f32 {
        self.sun_dir[3]
    }

    pub(super) fn update_matrix(
        &mut self,
        view: Mat4,
        proj: Mat4,
        camera_pos_dvec: glam::DVec3,
        sun_intensity: f32,
        color_grading: [f32; 3],
    ) {
        let view_proj = proj * view;
        self.view_proj = view_proj.to_cols_array_2d();
        self.inv_view_proj = view_proj.inverse().to_cols_array_2d();
        self.camera_pos = [
            camera_pos_dvec.x as f32,
            camera_pos_dvec.y as f32,
            camera_pos_dvec.z as f32,
            1.0,
        ];
        // The sky follows the flight, not the clock: `sun_intensity` is the depth scalar
        // (1 on the runway, 0 at cruise), so the sun sets on the way up and rises again
        // on the way down, identically.
        let sky = celestial::compute(
            sun_intensity,
            glam::Vec3::new(
                camera_pos_dvec.x as f32,
                camera_pos_dvec.y as f32,
                camera_pos_dvec.z as f32,
            ),
        );

        // The map drains toward greyscale on the way to cruise, and comes back on the way
        // down. That drain follows the *light*, not the altitude: it keeps the caller's
        // grading while the sun is up and through sunset, and fades it out across civil
        // and nautical twilight into the grey night. It used to run linearly with depth,
        // which pulled almost half the colour out of the map by the time the sun reached
        // the horizon — golden hour came out a grey-orange wash. Contrast is deliberately
        // left alone: grading pulls contrast toward a mid-grey pivot, and against a
        // near-black basemap that *raises* the dark pixels.
        const CRUISE_SATURATION: f32 = -1.0; // fully grey
        let colour = celestial::smoothstep(
            celestial::NIGHT_ELEVATION + 0.02,
            celestial::DUSK_ELEVATION,
            sky.sun_elevation,
        );
        self.sun_params = [
            sun_intensity,
            CRUISE_SATURATION + (color_grading[0] - CRUISE_SATURATION) * colour,
            color_grading[1],
            color_grading[2],
        ];

        self.sun_dir = [sky.sun_dir.x, sky.sun_dir.y, sky.sun_dir.z, sky.sun_elevation];
        self.moon_dir = [
            sky.moon_dir.x,
            sky.moon_dir.y,
            sky.moon_dir.z,
            sky.moon_illumination,
        ];
        self.light_color = [
            sky.light_color.x,
            sky.light_color.y,
            sky.light_color.z,
            sky.light_strength,
        ];
    }
}
