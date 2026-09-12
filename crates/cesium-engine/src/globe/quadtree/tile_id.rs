// Tile addressing types and Web Mercator coordinate conversion.

pub fn web_mercator_y_to_lat(y: f32, z: u8) -> f32 {
    let n = (1_u32 << z) as f32;
    let phi = (std::f32::consts::PI * (1.0 - 2.0 * y / n)).sinh().atan();
    phi.to_degrees()
}

pub(super) const MAX_ZOOM: u8 = 20;

/// The geographic rectangle a tile covers, in **degrees**, pole stretch included.
///
/// # Invariant I-5 — one source of tile bounds
///
/// [`tile_bounds`] is the *only* place these four numbers are derived. Both the
/// culling path (`QuadtreeNode`) and the drawn geometry (`TileMesh::generate`)
/// call it, so the rectangle the horizon test evaluates and the rectangle the
/// renderer actually fills are bit-identical. A mismatch here is not a rounding
/// curiosity: it is a metre-scale sliver of un-culled-but-undrawn surface at every
/// tile edge, i.e. a false negative — a hole in the globe.
///
/// If you need tile bounds anywhere else, call this. Do not re-derive them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TileBounds {
    pub lon_min: f64,
    pub lon_max: f64,
    pub lat_min: f64,
    pub lat_max: f64,
}

impl TileBounds {
    pub fn center_lon(&self) -> f64 {
        (self.lon_min + self.lon_max) * 0.5
    }

    pub fn center_lat(&self) -> f64 {
        (self.lat_min + self.lat_max) * 0.5
    }
}

/// The rendered extent of `id`, in degrees.
///
/// The arithmetic is deliberately performed in **f32 and then promoted**, not in
/// f64. That is not sloppiness — it is I-5. `web_mercator_y_to_lat` is f32 and is
/// shared with the mesh generator; promoting it here (rather than computing an f64
/// twin) is what makes the two agree exactly. Promoting *the function* instead
/// would move tile edges by ~1 m and both call sites would have to move together;
/// see `docs/culling-math.md` §11.3 item 5.
///
/// The `y == 0` / `y == 2^z − 1` rows are stretched to ±90° so the polar caps are
/// covered, matching `TileMesh::generate`'s pole-cap rows.
pub fn tile_bounds(id: &TileId) -> TileBounds {
    let z_pow = (1_u32 << id.z) as f32;

    let lon_min = -180.0_f32 + (id.x as f32) * 360.0 / z_pow;
    let lon_max = -180.0_f32 + ((id.x + 1) as f32) * 360.0 / z_pow;

    let mut lat_max = web_mercator_y_to_lat(id.y as f32, id.z);
    let mut lat_min = web_mercator_y_to_lat((id.y + 1) as f32, id.z);

    if id.y == 0 {
        lat_max = 90.0;
    }
    if id.y == (1_u32 << id.z) - 1 {
        lat_min = -90.0;
    }

    TileBounds {
        lon_min: lon_min as f64,
        lon_max: lon_max as f64,
        lat_min: lat_min as f64,
        lat_max: lat_max as f64,
    }
}

/// The *un*-stretched extent of `id`: identical to [`tile_bounds`] except that the
/// polar rows keep their true Mercator latitude instead of being pulled to ±90°.
///
/// Used only by the LOD radius, which deliberately sizes a polar cap by the ground
/// it really covers rather than by the stretched rectangle (see
/// `docs/culling-math.md` §6.3 / §8.4). **Never** use this for culling: the drawn
/// patch is the stretched one.
pub fn tile_bounds_unstretched(id: &TileId) -> TileBounds {
    let z_pow = (1_u32 << id.z) as f32;
    TileBounds {
        lon_min: (-180.0_f32 + (id.x as f32) * 360.0 / z_pow) as f64,
        lon_max: (-180.0_f32 + ((id.x + 1) as f32) * 360.0 / z_pow) as f64,
        lat_min: web_mercator_y_to_lat((id.y + 1) as f32, id.z) as f64,
        lat_max: web_mercator_y_to_lat(id.y as f32, id.z) as f64,
    }
}

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
