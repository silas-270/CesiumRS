//! One decoded elevation tile: the Terrarium decoder (B1), the sample grid, and
//! the 16x16 min/max mip (B3) of `docs/terrain-plan.md` §5.
//!
//! # Units
//!
//! Everything in **this** module is **metres**, the unit the source encodes and the
//! only unit `i16` is a sensible container for. The conversion to the engine's
//! megametres happens exactly once, at the [`super::height_cache`] boundary — see
//! `quadtree/surface.rs`'s module doc for why that seam is where it is.

use crate::globe::tiles::config::OceanPolicy;

/// Terrarium tiles are 256x256. Not a preference — the source's size.
pub const HEIGHT_TILE_DIM: usize = 256;
/// `256 * 256`.
pub const HEIGHT_TILE_TEXELS: usize = HEIGHT_TILE_DIM * HEIGHT_TILE_DIM;
/// Edge length of the min/max mip — `docs/terrain-plan.md` §3.3 and §5 B3.
pub const HEIGHT_MIP_DIM: usize = 16;
/// `16 * 16`.
pub const HEIGHT_MIP_CELLS: usize = HEIGHT_MIP_DIM * HEIGHT_MIP_DIM;
/// Texels per mip cell, both axes: `256 / 16`.
pub const HEIGHT_MIP_BLOCK: usize = HEIGHT_TILE_DIM / HEIGHT_MIP_DIM;

/// A decoded height tile: 256x256 samples in metres, plus the extrema and the
/// min/max pyramid Phase D will cull with.
///
/// `i16` metres, not `f32`: half the bytes, and finer than the underlying DEM, which
/// is SRTM-class (~30 m posting) almost everywhere. It also spans −32768 … 32767 m,
/// comfortably outside the −11 km … +9 km the real Earth occupies.
pub struct HeightTile {
    /// Row-major, `data[y * 256 + x]`, metres, ocean policy already applied.
    pub data: Box<[i16; HEIGHT_TILE_TEXELS]>,
    /// Minimum over **all 65 536** texels.
    pub h_min: i16,
    /// Maximum over **all 65 536** texels.
    pub h_max: i16,
    /// Per-16x16-block minima, row-major over the 16x16 mip grid.
    min_mip: Box<[i16; HEIGHT_MIP_CELLS]>,
    /// Per-16x16-block maxima, same indexing.
    max_mip: Box<[i16; HEIGHT_MIP_CELLS]>,
}

/// Decodes one Terrarium PNG's RGBA bytes.
///
/// # The encoding
///
/// `h_metres = R·256 + G + B/256 − 32768`. R and G carry whole metres, B carries
/// 1/256 of a metre. The arithmetic below is done in exact `i32` 1/256-metre units and
/// rounded once at the end, rather than going through `f32`: the decode is then
/// bit-reproducible on every platform, which is what lets the fixture tests pin exact
/// extrema instead of ranges.
///
/// # Extrema
///
/// Taken over **all 65 536 texels**, deliberately, not over the 17x17 grid
/// `TileMesh` will sample. Phase D fits this tile's bounding box to `[h_min, h_max]`,
/// and a maximum computed on a subgrid misses summits between grid lines — which is
/// an under-estimate of the box, which is a false negative, which by invariant I-7
/// is a hole in the globe.
pub fn decode_terrarium(
    width: u32,
    height: u32,
    rgba: &[u8],
    ocean: OceanPolicy,
) -> Result<HeightTile, String> {
    if width as usize != HEIGHT_TILE_DIM || height as usize != HEIGHT_TILE_DIM {
        return Err(format!(
            "terrarium tile must be {HEIGHT_TILE_DIM}x{HEIGHT_TILE_DIM}, got {width}x{height}"
        ));
    }
    if rgba.len() < HEIGHT_TILE_TEXELS * 4 {
        return Err(format!(
            "terrarium tile is {} bytes, need {}",
            rgba.len(),
            HEIGHT_TILE_TEXELS * 4
        ));
    }

    let mut data = Box::new([0i16; HEIGHT_TILE_TEXELS]);
    for (i, slot) in data.iter_mut().enumerate() {
        let px = &rgba[i * 4..i * 4 + 3];
        *slot = decode_texel(px[0], px[1], px[2], ocean);
    }
    Ok(HeightTile::from_samples(data))
}

/// One texel, in 1/256-metre integer units rounded to the nearest metre.
///
/// Rounding is half-up (`+128` before the floor division), deterministic and the same
/// on every input; `div_euclid` rather than `/` so negatives round the same direction
/// as positives instead of truncating toward zero. The clamp to `i16` can only fire on
/// bytes the real source never emits (`h > 32767 m`), and exists so a corrupt tile
/// cannot wrap a summit into a trench.
#[inline]
fn decode_texel(r: u8, g: u8, b: u8, ocean: OceanPolicy) -> i16 {
    // (R·256 + G)·256 + B, in 1/256 m, with the 32768 m bias removed.
    let sixteenths = ((r as i32) * 256 + g as i32) * 256 + b as i32 - 32768 * 256;
    let metres = (sixteenths + 128).div_euclid(256).clamp(-32768, 32767) as i16;
    match ocean {
        OceanPolicy::ClampToZero => metres.max(0),
        OceanPolicy::Raw => metres,
    }
}

impl HeightTile {
    /// Builds the extrema and the min/max mip over a finished sample grid.
    pub fn from_samples(data: Box<[i16; HEIGHT_TILE_TEXELS]>) -> Self {
        let mut min_mip = Box::new([i16::MAX; HEIGHT_MIP_CELLS]);
        let mut max_mip = Box::new([i16::MIN; HEIGHT_MIP_CELLS]);

        for y in 0..HEIGHT_TILE_DIM {
            let cy = y / HEIGHT_MIP_BLOCK;
            for x in 0..HEIGHT_TILE_DIM {
                let h = data[y * HEIGHT_TILE_DIM + x];
                let cell = cy * HEIGHT_MIP_DIM + x / HEIGHT_MIP_BLOCK;
                if h < min_mip[cell] {
                    min_mip[cell] = h;
                }
                if h > max_mip[cell] {
                    max_mip[cell] = h;
                }
            }
        }

        // The tile extrema are the extrema of the mip, which is exactly the extrema of
        // all 65 536 texels: every texel belongs to exactly one block.
        let h_min = *min_mip.iter().min().expect("mip is non-empty");
        let h_max = *max_mip.iter().max().expect("mip is non-empty");

        Self {
            data,
            h_min,
            h_max,
            min_mip,
            max_mip,
        }
    }

    /// A tile of exact zeros — sea level everywhere, no relief.
    ///
    /// This is what `offline_mode` serves (`docs/terrain-plan.md` §5 acceptance), so
    /// every headless test can run with terrain enabled and no network and see a globe
    /// geometrically identical to the flat one.
    pub fn flat_zero() -> Self {
        Self::from_samples(Box::new([0i16; HEIGHT_TILE_TEXELS]))
    }

    /// One sample, in metres. `x`/`y` are clamped, so edge lookups are legal.
    #[inline]
    pub fn sample(&self, x: usize, y: usize) -> i16 {
        let x = x.min(HEIGHT_TILE_DIM - 1);
        let y = y.min(HEIGHT_TILE_DIM - 1);
        self.data[y * HEIGHT_TILE_DIM + x]
    }

    /// Bilinear height in **metres** at tile-local `(u, v)` ∈ `[0,1]²`, `v` running
    /// with the tile's rows (v = 0 is the top row, matching image and texture order).
    ///
    /// f64 throughout, per invariant I-2. Texel *centres* sit at `(i + 0.5)/256`, so
    /// the grid coordinate is `u·256 − 0.5`; a half-texel border at each edge clamps to
    /// the edge sample rather than extrapolating past it.
    pub fn sample_bilinear(&self, u: f64, v: f64) -> f64 {
        let n = HEIGHT_TILE_DIM as f64;
        let gx = (u.clamp(0.0, 1.0) * n - 0.5).clamp(0.0, n - 1.0);
        let gy = (v.clamp(0.0, 1.0) * n - 0.5).clamp(0.0, n - 1.0);

        let x0 = gx.floor();
        let y0 = gy.floor();
        let fx = gx - x0;
        let fy = gy - y0;
        let (x0, y0) = (x0 as usize, y0 as usize);

        let h00 = self.sample(x0, y0) as f64;
        let h10 = self.sample(x0 + 1, y0) as f64;
        let h01 = self.sample(x0, y0 + 1) as f64;
        let h11 = self.sample(x0 + 1, y0 + 1) as f64;

        let top = h00 + (h10 - h00) * fx;
        let bottom = h01 + (h11 - h01) * fx;
        top + (bottom - top) * fy
    }

    /// Minimum height over one 16x16-texel mip cell, metres.
    ///
    /// Phase D's occlusion march (§3.3) reads **this** side: the occluder must be a
    /// *lower* bound on the terrain, because only something that is definitely there
    /// can definitely block. Reading [`Self::mip_max`] here instead over-occludes, and
    /// over-occlusion is precisely the false negative this engine exists to prevent.
    #[inline]
    pub fn mip_min(&self, cx: usize, cy: usize) -> i16 {
        self.min_mip[cy.min(HEIGHT_MIP_DIM - 1) * HEIGHT_MIP_DIM + cx.min(HEIGHT_MIP_DIM - 1)]
    }

    /// Maximum height over one 16x16-texel mip cell, metres.
    ///
    /// The *occludee* side of §3.3, and the `h_max` side of §3.1's bounding boxes: if
    /// even a tile's highest possible point is hidden, all of it is.
    #[inline]
    pub fn mip_max(&self, cx: usize, cy: usize) -> i16 {
        self.max_mip[cy.min(HEIGHT_MIP_DIM - 1) * HEIGHT_MIP_DIM + cx.min(HEIGHT_MIP_DIM - 1)]
    }
}

impl std::fmt::Debug for HeightTile {
    /// Deliberately does not print 65 536 samples.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HeightTile")
            .field("h_min", &self.h_min)
            .field("h_max", &self.h_max)
            .finish_non_exhaustive()
    }
}
