// Tile addressing types and Web Mercator coordinate conversion.

/// Web-Mercator row `y` (fractional) to latitude in degrees, in **f64**.
///
/// This is the definition every tile boundary in the engine is derived from —
/// culling rectangle, drawn mesh and sub-box splits alike. See [`tile_bounds`] and
/// invariant **I-5**.
///
/// # Why f64 and not f32
///
/// `docs/culling-math.md` §11.3 item 5 offers "leave it f32" as a conservative
/// fallback, on the grounds that the mesh and the quadtree both called the f32
/// version and had to move together. They no longer have separate call sites:
/// [`tile_bounds`] is the one source, so there is nothing left to get out of step.
///
/// Keeping f32 was measured to be the last remaining source of false negatives.
/// The f32 longitude bound `-180 + x·360/2^z` has an ulp of 1.53·10⁻⁵° at |lon| ≈
/// 150°, i.e. **1.7 m of ground**, and the latitude a few tenths of a metre. The
/// engine's tiling stays a partition under that quantisation — adjacent tiles share
/// the *same* f32 boundary value, so nothing is undrawn — but every tile edge sits
/// up to 1.7 m away from its true Web-Mercator position. Below z ≈ 19 that is far
/// inside a tile and invisible; at z = 19–20, where a tile is 76 m and 38 m wide, it
/// is a real displacement, and it was responsible for **all 7 177** residual false
/// negatives in the 100 000-cell fuzz sweep and all 4 in the near-ground sweep
/// (every one of them at deepest zoom 19 or 20, with the camera 10–100 m up).
///
/// In f64 the boundary lands where Web Mercator says it does, which is also where
/// every imagery server puts it.
pub fn web_mercator_y_to_lat_f64(y: f64, z: u8) -> f64 {
    let n = (1_u64 << z) as f64;
    (std::f64::consts::PI * (1.0 - 2.0 * y / n))
        .sinh()
        .atan()
        .to_degrees()
}

/// f32 convenience wrapper over [`web_mercator_y_to_lat_f64`].
///
/// Retained for callers outside the culling path that want an f32 latitude. **Do
/// not** use it to derive a tile boundary: go through [`tile_bounds`].
pub fn web_mercator_y_to_lat(y: f32, z: u8) -> f32 {
    web_mercator_y_to_lat_f64(y as f64, z) as f32
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

/// The rendered extent of `id`, in degrees. **All f64** — see
/// [`web_mercator_y_to_lat_f64`] for why that matters at z ≥ 19.
///
/// Adjacent tiles share their boundary *value*, not merely a boundary formula:
/// `tile_bounds(x).lon_max` and `tile_bounds(x+1).lon_min` are the same expression
/// on the same operands, so the tiling is an exact partition with no seams.
///
/// The `y == 0` / `y == 2^z − 1` rows are stretched to ±90° so the polar caps are
/// covered, matching `TileMesh::generate`'s pole-cap rows.
pub fn tile_bounds(id: &TileId) -> TileBounds {
    let n = (1_u64 << id.z) as f64;

    let lon_min = -180.0 + (id.x as f64) * 360.0 / n;
    let lon_max = -180.0 + ((id.x + 1) as f64) * 360.0 / n;

    let mut lat_max = web_mercator_y_to_lat_f64(id.y as f64, id.z);
    let mut lat_min = web_mercator_y_to_lat_f64((id.y + 1) as f64, id.z);

    if id.y == 0 {
        lat_max = 90.0;
    }
    if id.y == (1_u32 << id.z) - 1 {
        lat_min = -90.0;
    }

    TileBounds {
        lon_min,
        lon_max,
        lat_min,
        lat_max,
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
    let n = (1_u64 << id.z) as f64;
    TileBounds {
        lon_min: -180.0 + (id.x as f64) * 360.0 / n,
        lon_max: -180.0 + ((id.x + 1) as f64) * 360.0 / n,
        lat_min: web_mercator_y_to_lat_f64((id.y + 1) as f64, id.z),
        lat_max: web_mercator_y_to_lat_f64(id.y as f64, id.z),
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
