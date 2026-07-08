// Tile addressing types and Web Mercator coordinate conversion.

pub fn web_mercator_y_to_lat(y: f32, z: u8) -> f32 {
    let n = (1_u32 << z) as f32;
    let phi = (std::f32::consts::PI * (1.0 - 2.0 * y / n)).sinh().atan();
    phi.to_degrees()
}

pub(super) const MAX_ZOOM: u8 = 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct TileId {
    pub z: u8,
    pub x: u32,
    pub y: u32,
}

impl TileId {
    pub fn parent(&self) -> Option<TileId> {
        if self.z == 0 {
            None
        } else {
            Some(TileId {
                z: self.z - 1,
                x: self.x / 2,
                y: self.y / 2,
            })
        }
    }
}
