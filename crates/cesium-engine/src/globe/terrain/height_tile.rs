//! One decoded elevation tile: the Terrarium decoder (B1), the sample grid, and
//! the 16x16 min/max mip (B3) of `docs/terrain-plan.md` §5.
//!
//! # Units
//!
//! Everything in **this** module is **metres**, the unit the source encodes. The
//! conversion to the engine's
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

/// Texels between two consecutive samples of the **drawn mesh** across one tile —
/// `256 / 16`, i.e. exactly one mip block — and therefore the decimation
/// [`HeightTile::detail`] measures the field against. E1 of `docs/terrain-plan.md` §8.
///
/// Not a coincidence worth leaving unstated: `TileEngineConfig::mesh_segments` ships at
/// **16**, so `TileMesh::generate_on` lays 17 samples across a tile and the span between
/// two of them is `256/16 = 16` texels. The deviation of the field from its own 16:1
/// decimation is then *exactly* the deviation of the drawn surface from the DEM — the
/// tile's geometric error, not a proxy for it.
///
/// At `mesh_segments = 32` the mesh is finer than this and the number over-states the
/// error (refines slightly early, costs tiles, never shape); at `mesh_segments = 8` it
/// under-states it. Stated rather than asserted because the decode has no access to the
/// configuration — the same situation, and the same remedy, as `INHERIT_SEGMENTS`'s in
/// [`super::heightfield`].
pub const HEIGHT_DETAIL_STEP: usize = HEIGHT_TILE_DIM / 16;

/// Levels below this tile's own for which [`HeightTile::detail_below`] stores a measured
/// error — **F5** of `docs/terrain-plan.md` §9.
///
/// A descendant `k` levels below this tile draws the same 17×17 mesh lattice over
/// `4^k`-times less ground, so it decimates this tile's texels `HEIGHT_DETAIL_STEP / 2^k`:1.
/// At `k = 4` that is 1:1 — the mesh lands on every texel and draws the data exactly — so
/// `k ∈ {1, 2, 3}` is the whole of what there is to store, and everything deeper is
/// **exactly** zero rather than approximately so.
pub const HEIGHT_DETAIL_LEVELS: u32 = 3;

/// Entries in [`HeightTile::detail_below`]'s pyramid: `4 + 16 + 64`.
///
/// One per descendant at each stored level — `4^k` sub-tiles at level `k` — because a whole
/// -tile maximum is the wrong window for a descendant. F5 measured that: scored against the
/// error the mesh really leaves, a whole-tile number at the right lattice over-states by a
/// median 1.25× at z16 and **2.0×** at z17 and z18, and over-stating is what refines a level
/// too deep across a whole near field.
pub const HEIGHT_DETAIL_PYRAMID_CELLS: usize = 4 + 16 + 64;

/// A decoded height tile: 256x256 samples in metres, plus the extrema and the
/// min/max pyramid Phase D will cull with.
///
/// # Precision
///
/// Samples are stored as `u16` steps across **this tile's own** height range —
/// `h = base + q · scale` — which is 2 bytes a texel like the whole-metre `i16` this
/// replaced, but sub-metre: a z15 tile spanning 1 500 m of Alps resolves 2.3 cm, a flat
/// one far less. Whole metres were not enough: the source carries 1/256 m and its z15
/// texels step 10-20 cm apart over a runway, so rounding turned gently sloping flat
/// ground into 1 m terraces a texel (~3 m) wide — an 18° ramp every few metres, and the
/// "uneven ground in even regions" seen from low altitude.
///
/// The extrema, mips and error terms stay in whole metres, rounded **outward** (min
/// down, max and errors up), so every bound still contains the samples it describes.
pub struct HeightTile {
    /// Row-major, `data[y * 256 + x]`, quantised; see [`Self::sample`].
    data: Box<[u16; HEIGHT_TILE_TEXELS]>,
    /// Metres at `q = 0`.
    base: f32,
    /// Metres per quantisation step.
    scale: f32,
    /// Minimum over **all 65 536** texels, rounded down to whole metres.
    pub h_min: i16,
    /// Maximum over **all 65 536** texels, rounded up to whole metres.
    pub h_max: i16,
    /// Per-16x16-block minima, row-major over the 16x16 mip grid.
    min_mip: Box<[i16; HEIGHT_MIP_CELLS]>,
    /// Per-16x16-block maxima, same indexing.
    max_mip: Box<[i16; HEIGHT_MIP_CELLS]>,
    /// This tile's **measured geometric error**, metres — E1 of
    /// `docs/terrain-plan.md` §8. See [`Self::detail`].
    detail: i16,
    /// The same measurement for each descendant one, two and three levels down — F5 of
    /// `docs/terrain-plan.md` §9. See [`Self::detail_below`] for the layout.
    detail_below: Box<[i16; HEIGHT_DETAIL_PYRAMID_CELLS]>,
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

    let mut metres = vec![0f32; HEIGHT_TILE_TEXELS];
    for (i, slot) in metres.iter_mut().enumerate() {
        let px = &rgba[i * 4..i * 4 + 3];
        *slot = decode_texel(px[0], px[1], px[2], ocean);
    }
    Ok(HeightTile::from_metres(&metres))
}

/// One texel in metres, at the source's full 1/256 m precision.
///
/// The value is formed in exact `i32` 1/256-metre units and divided once; every result
/// is exactly representable in `f32` (at most 2^23 units), so the decode is
/// bit-reproducible on every platform. The clamp to ±32 km can only fire on bytes the
/// real source never emits, and exists so a corrupt tile cannot produce an absurd height.
#[inline]
fn decode_texel(r: u8, g: u8, b: u8, ocean: OceanPolicy) -> f32 {
    // (R·256 + G)·256 + B, in 1/256 m, with the 32768 m bias removed.
    let units = ((r as i32) * 256 + g as i32) * 256 + b as i32 - 32768 * 256;
    let metres = (units.clamp(-32768 * 256, 32767 * 256) as f32) / 256.0;
    match ocean {
        OceanPolicy::ClampToZero => metres.max(0.0),
        OceanPolicy::Raw => metres,
    }
}

/// The lattice one axis of the mesh samples this tile on, as `(lo, hi, weight)` per
/// texel — see [`HeightTile::detail`].
///
/// The lattice lines are texels `0, 16, 32, … 240, 255`: seventeen of them, the last
/// pulled onto the tile's final texel rather than off the end of it, which makes the last
/// interval fifteen texels wide instead of sixteen. Interpolating with each interval's
/// *own* width keeps `I(h)` an exact piecewise-linear function through the lattice
/// samples, so a field that is already linear measures exactly zero deviation — the
/// property the whole measurement rests on, and the one an assumed-uniform spacing would
/// quietly break in the last row and column.
/// `step` is [`HEIGHT_DETAIL_STEP`] for the tile's own mesh and `HEIGHT_DETAIL_STEP / 2^k`
/// for a descendant `k` levels down — F5. A descendant's window always starts on a multiple
/// of its own `step` (the window is `i · 256/2^k` texels wide and `step` divides that), so
/// the *global* lattice below **is** that descendant's own mesh lattice restricted to its
/// window, and there is one lattice rather than one per sub-tile.
fn detail_lattice(step: usize) -> [(usize, usize, f64); HEIGHT_TILE_DIM] {
    let step = step.max(1);
    let mut out = [(0usize, 0usize, 0.0f64); HEIGHT_TILE_DIM];
    let last = HEIGHT_TILE_DIM - 1;
    for (x, slot) in out.iter_mut().enumerate() {
        let lo = (x / step) * step;
        let hi = (lo + step).min(last);
        let span = hi - lo;
        let w = if span == 0 {
            0.0
        } else {
            (x - lo) as f64 / span as f64
        };
        *slot = (lo, hi, w);
    }
    out
}

/// [`HeightTile::detail`], computed once over a finished sample grid.
///
/// `ceil` rather than round: the number is used as an upper bound on the drawn surface's
/// error, and rounding a 0.4 m deviation to zero would report a field as flat that is not.
fn measure_detail(data: &[f32]) -> i16 {
    measure_detail_over(
        data,
        &detail_lattice(HEIGHT_DETAIL_STEP),
        [0, HEIGHT_TILE_DIM, 0, HEIGHT_TILE_DIM],
    )
}

/// [`measure_detail`] on a `lattice` already built, restricted to the texel window
/// `[x0, x1) × [y0, y1)` — the shape F5's pyramid needs, and the shape
/// [`measure_detail`] is now one call of.
fn measure_detail_over(
    data: &[f32],
    lattice: &[(usize, usize, f64); HEIGHT_TILE_DIM],
    [x0, x1, y0, y1]: [usize; 4],
) -> i16 {
    let mut worst = 0.0f64;
    for y in y0..y1.min(HEIGHT_TILE_DIM) {
        let (ly0, ly1, wy) = lattice[y];
        for x in x0..x1.min(HEIGHT_TILE_DIM) {
            let (lx0, lx1, wx) = lattice[x];
            let h00 = data[ly0 * HEIGHT_TILE_DIM + lx0] as f64;
            let h10 = data[ly0 * HEIGHT_TILE_DIM + lx1] as f64;
            let h01 = data[ly1 * HEIGHT_TILE_DIM + lx0] as f64;
            let h11 = data[ly1 * HEIGHT_TILE_DIM + lx1] as f64;
            let top = h00 + (h10 - h00) * wx;
            let bottom = h01 + (h11 - h01) * wx;
            let interpolated = top + (bottom - top) * wy;
            let dev = (data[y * HEIGHT_TILE_DIM + x] as f64 - interpolated).abs();
            if dev > worst {
                worst = dev;
            }
        }
    }
    worst.ceil().min(i16::MAX as f64) as i16
}

/// **F5** — [`HeightTile::detail_below`]'s pyramid, computed once over a finished grid.
///
/// Three passes over the 65 536 texels, one per stored level, each restricted to the
/// `4^k` sub-tiles of that level in turn. The whole pyramid is the same arithmetic
/// [`measure_detail`] already does, at three finer lattices and over smaller windows, so
/// the number a descendant reads is the number its own mesh really leaves — not a scaled
/// guess and not its ancestor's whole-tile maximum. F5's table scores both of those
/// against this one.
fn measure_detail_pyramid(
    data: &[f32],
) -> Box<[i16; HEIGHT_DETAIL_PYRAMID_CELLS]> {
    let mut out = Box::new([0i16; HEIGHT_DETAIL_PYRAMID_CELLS]);
    for k in 1..=HEIGHT_DETAIL_LEVELS {
        let lattice = detail_lattice(HEIGHT_DETAIL_STEP >> k);
        let side = 1usize << k;
        let span = HEIGHT_TILE_DIM / side;
        let base = pyramid_base(k);
        for iy in 0..side {
            for ix in 0..side {
                out[base + iy * side + ix] = measure_detail_over(
                    data,
                    &lattice,
                    [ix * span, (ix + 1) * span, iy * span, (iy + 1) * span],
                );
            }
        }
    }
    out
}

/// Where level `k`'s `4^k` entries start in the flat pyramid: `0`, `4`, `20`.
#[inline]
const fn pyramid_base(k: u32) -> usize {
    match k {
        1 => 0,
        2 => 4,
        _ => 20,
    }
}

impl HeightTile {
    /// Builds a tile from whole-metre samples — the fixtures' and tests' constructor.
    pub fn from_samples(data: Box<[i16; HEIGHT_TILE_TEXELS]>) -> Self {
        let metres: Vec<f32> = data.iter().map(|&h| h as f32).collect();
        Self::from_metres(&metres)
    }

    /// Quantises `metres` (row-major, 65 536 samples) into this tile's own range and
    /// builds the extrema, the min/max mip and the error terms over the **stored**
    /// (dequantised) values, so every bound describes exactly what is sampled later.
    pub fn from_metres(metres: &[f32]) -> Self {
        assert_eq!(metres.len(), HEIGHT_TILE_TEXELS, "a height tile is 256x256");
        let (lo, hi) = metres
            .iter()
            .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), &h| (lo.min(h), hi.max(h)));
        // The step is a power-of-two multiple of the source's 1/256 m, so values on that
        // grid — every Terrarium sample, every whole metre — are stored exactly whenever
        // the range allows: a tile spanning under 256 m (most of z15) keeps the source's
        // full precision, one spanning 1 500 m of Alps is held to 1/32 m.
        let base = lo;
        let needed = (hi - lo) * 256.0 / u16::MAX as f32;
        let mut k = 1.0f32;
        while k < needed {
            k *= 2.0;
        }
        let scale = k / 256.0;

        let mut data = Box::new([0u16; HEIGHT_TILE_TEXELS]);
        let mut stored = vec![0f32; HEIGHT_TILE_TEXELS];
        for (i, &h) in metres.iter().enumerate() {
            let q = ((h - base) / scale).round().clamp(0.0, u16::MAX as f32) as u16;
            data[i] = q;
            stored[i] = base + q as f32 * scale;
        }

        let mut min_f = [f32::INFINITY; HEIGHT_MIP_CELLS];
        let mut max_f = [f32::NEG_INFINITY; HEIGHT_MIP_CELLS];
        for y in 0..HEIGHT_TILE_DIM {
            let cy = y / HEIGHT_MIP_BLOCK;
            for x in 0..HEIGHT_TILE_DIM {
                let h = stored[y * HEIGHT_TILE_DIM + x];
                let cell = cy * HEIGHT_MIP_DIM + x / HEIGHT_MIP_BLOCK;
                min_f[cell] = min_f[cell].min(h);
                max_f[cell] = max_f[cell].max(h);
            }
        }
        let down = |h: f32| h.floor().clamp(i16::MIN as f32, i16::MAX as f32) as i16;
        let up = |h: f32| h.ceil().clamp(i16::MIN as f32, i16::MAX as f32) as i16;
        let min_mip = Box::new(min_f.map(down));
        let max_mip = Box::new(max_f.map(up));

        // The tile extrema are the extrema of the mip, which is exactly the extrema of
        // all 65 536 texels: every texel belongs to exactly one block.
        let h_min = *min_mip.iter().min().expect("mip is non-empty");
        let h_max = *max_mip.iter().max().expect("mip is non-empty");

        // The error terms describe the terrain's shape, so they are measured on the
        // decoded heights rather than the stored ones: quantisation noise is not shape,
        // and `ceil` would turn a centimetre of it into a metre of error.
        let detail = measure_detail(metres);
        let detail_below = measure_detail_pyramid(metres);

        Self {
            data,
            base,
            scale,
            h_min,
            h_max,
            min_mip,
            max_mip,
            detail,
            detail_below,
        }
    }

    /// Every sample in metres, row-major.
    pub fn samples(&self) -> impl Iterator<Item = f32> + '_ {
        self.data.iter().map(|&q| self.base + q as f32 * self.scale)
    }

    /// The tile's **measured geometric error** in metres: how far its own height field
    /// departs from the surface a mesh laid across it actually draws — E1 of
    /// `docs/terrain-plan.md` §8, and the whole content of the terrain LOD term.
    ///
    /// # What is measured
    ///
    /// `max over all 65 536 texels of |h − I(h)|`, where `I(h)` is the piecewise-bilinear
    /// interpolation of the same field decimated [`HEIGHT_DETAIL_STEP`]:1 — the 17×17
    /// lattice `TileMesh::generate_on` builds its vertices on. So this is not a proxy for
    /// the drawn surface's error, it *is* that error, evaluated against the finest data
    /// there is for this tile.
    ///
    /// # Why this beats a level-based error
    ///
    /// Cesium's heightmap path (`getEstimatedLevelZeroGeometricErrorForAHeightmap`) is
    /// `2πa / (65 · 2^z)` — a function of the level and nothing else, so the Bay of Bengal
    /// and the Karakoram at the same zoom are given the same error and refine at the same
    /// distance. Here the number comes off the data: this tile's own roughness at exactly
    /// the scale the mesh fails to resolve. It costs one extra pass over a grid the decode
    /// already walks and no memory beyond this `i16`.
    ///
    /// # A maximum over the tile, on purpose
    ///
    /// One cliff in a corner gives the whole tile a large error and refines all of it.
    /// That is the direction a *bound* on the drawn surface's error has to round — and it
    /// is the opposite of [`Self::mip_min`]'s problem, where a minimum over a whole tile
    /// averaged a ridge away (`docs/terrain-plan.md` §7b). An error that is too large
    /// costs tiles; an error that is too small costs shape, silently.
    ///
    /// # Units
    ///
    /// Metres, like everything else in this module, and **without** vertical
    /// exaggeration — `HeightTileManager::height_bounds_for` multiplies it in on the way
    /// out, at the same seam as every other altitude.
    #[inline]
    pub fn detail(&self) -> i16 {
        self.detail
    }

    /// **F5** — [`Self::detail`] for the descendant `k` levels below this tile whose share
    /// of its texel grid is sub-tile `(ix, iy)` of the `2^k × 2^k` grid. Metres.
    ///
    /// `k = 0` is [`Self::detail`] itself. `k > `[`HEIGHT_DETAIL_LEVELS`] returns **exactly
    /// zero**, and that is a fact rather than a cut-off: at `k = 4` the mesh's lattice lands
    /// on every texel of its window, so it draws the data it has and there is no error left
    /// to report. `ix`/`iy` out of range are clamped, so a caller that has rounded a UV is
    /// answered rather than panicking.
    ///
    /// # This number is tied to `mesh_segments = 16`, exactly as [`HEIGHT_DETAIL_STEP`] is
    ///
    /// The stored lattices are `8`, `4` and `2` texels, which are the descendant's mesh
    /// spacing only while the mesh lays 17 samples across a tile. At 32 the whole ladder
    /// shifts one level and every entry over-states; at 8 it under-states. Same situation
    /// and same remedy as `HEIGHT_DETAIL_STEP`'s, and §9 F1's reason for leaving
    /// `mesh_segments` at 16 is unchanged by F5.
    #[inline]
    pub fn detail_below(&self, k: u32, ix: u32, iy: u32) -> i16 {
        if k == 0 {
            return self.detail;
        }
        if k > HEIGHT_DETAIL_LEVELS {
            return 0;
        }
        let side = 1u32 << k;
        let ix = ix.min(side - 1) as usize;
        let iy = iy.min(side - 1) as usize;
        self.detail_below[pyramid_base(k) + iy * side as usize + ix]
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
    pub fn sample(&self, x: usize, y: usize) -> f32 {
        let x = x.min(HEIGHT_TILE_DIM - 1);
        let y = y.min(HEIGHT_TILE_DIM - 1);
        self.base + self.data[y * HEIGHT_TILE_DIM + x] as f32 * self.scale
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

    /// `(min, max)` in metres over every texel the `[u0,u1] × [v0,v1]` rectangle
    /// touches — Phase D1's bounding-volume source.
    ///
    /// The rectangle is rounded **outward** to whole mip cells, so the answer is an
    /// upper bound on the true extrema over it and never an under-estimate: an
    /// under-estimate is a box that does not contain its own geometry, i.e. a false
    /// negative, and by I-7 a hole in the globe (I-6).
    ///
    /// # The one-cell halo
    ///
    /// [`Self::sample_bilinear`] places texel *centres* at `(i + 0.5)/256`, so a sample
    /// taken exactly on a cell boundary reads one texel on each side of it — the far one
    /// belonging to the neighbouring mip cell. The covered cell range is therefore
    /// grown by one cell on every side, which is 16 texels where half a texel would do:
    /// 32× more slack than the argument needs, and still only 1/16 of the tile, against
    /// the alternative of a boundary case that is right in every test and wrong in the
    /// one frame a summit sits on a cell edge.
    ///
    /// The halo does not break the containment that makes the inheritance margin zero
    /// below z15: `floor` is monotone, so a dyadic sub-rectangle's grown cell range is
    /// still a subset of its parent's grown range.
    ///
    /// The whole tile (`0,0 → 1,1`) returns exactly [`Self::h_min`] and [`Self::h_max`],
    /// which is what a tile at or above the source's deepest level asks for.
    /// The largest height **range** over any aligned quarter-window of any of the four
    /// edges of the `[u0,u1] × [v0,v1]` rectangle, in metres — **D1's follow-up**, the
    /// tight replacement for bounding C3's skirt by the whole tile's range.
    ///
    /// # What it bounds and why the windows are quarters
    ///
    /// C3's crack is `max_i |h[i] − lerp(h[i₀], h[i₁])|` along one edge, where `i₀`/`i₁`
    /// are that edge's samples `k` grid steps apart and `k ∈ {2, 4}`
    /// (`SKIRT_COARSENINGS`). A linear interpolant of two samples never leaves their
    /// interval, so the deviation over one window cannot exceed the **range of the field
    /// over that window** — and `i₀ = (i/k)·k` makes the windows *aligned*, so the
    /// `k = 4` windows are the edge's four quarters and every `k = 2` window nests inside
    /// one of them. Four quarters per edge therefore cover both coarsenings exactly.
    ///
    /// The old bound was the range over the **whole tile**, which is what
    /// `docs/terrain-plan.md` §7 records as making a node's interval **1.53×** the mesh
    /// interval it has to contain (Everest z12: a 741 m real skirt bounded by a 4 700 m
    /// span). A summit in the middle of a tile inflates that bound and cannot affect any
    /// edge's interpolation at all; this reads only the edges.
    ///
    /// Each window is queried as a degenerate (zero-width) rectangle on the edge line
    /// itself. [`Self::mip_extrema_over`] rounds outward to whole mip cells and adds its
    /// one-cell halo, so the answer is an upper bound on the field along that line —
    /// which is the direction I-6 needs: a skirt allowance that is too small is a box
    /// that does not contain its own geometry.
    pub fn edge_window_range(&self, u0: f64, v0: f64, u1: f64, v1: f64) -> i32 {
        /// Aligned windows per edge — see this method's doc comment.
        const WINDOWS: usize = 4;
        let mut worst = 0i32;
        for w in 0..WINDOWS {
            let t0 = w as f64 / WINDOWS as f64;
            let t1 = (w + 1) as f64 / WINDOWS as f64;
            let ua = u0 + (u1 - u0) * t0;
            let ub = u0 + (u1 - u0) * t1;
            let va = v0 + (v1 - v0) * t0;
            let vb = v0 + (v1 - v0) * t1;
            for (a0, b0, a1, b1) in [
                (ua, v0, ub, v0), // north edge
                (ua, v1, ub, v1), // south edge
                (u0, va, u0, vb), // west edge
                (u1, va, u1, vb), // east edge
            ] {
                let (lo, hi) = self.mip_extrema_texel_halo(a0, b0, a1, b1);
                worst = worst.max(hi as i32 - lo as i32);
            }
        }
        worst
    }

    /// [`Self::mip_extrema_over`] with the halo grown by **two texels** instead of a
    /// whole mip cell.
    ///
    /// The whole-cell halo that method uses is deliberately 32× more slack than the
    /// bilinear argument needs, and it says so — which is free when the rectangle is a
    /// whole tile and expensive when it is a one-texel-wide line. [`Self::sample_bilinear`]
    /// places texel centres at `(i + 0.5)/256`, so a read at parameter `t` touches texels
    /// with indices in `t·256 ± 1.5`; inflating the parameter by `2/256` before flooring
    /// to cells covers that with half a texel to spare, and usually lands in the *same*
    /// mip cell rather than the next one.
    ///
    /// Used by [`Self::edge_window_range`], where the across-edge direction is one texel
    /// wide and the whole-cell halo would otherwise make it 32 texels deep — which on the
    /// Everest fixture is most of the difference between the old bound and the real skirt.
    pub fn mip_extrema_texel_halo(&self, u0: f64, v0: f64, u1: f64, v1: f64) -> (i16, i16) {
        const TEXEL_HALO: f64 = 2.0 / HEIGHT_TILE_DIM as f64;
        self.mip_cell_extrema(
            u0.min(u1) - TEXEL_HALO,
            v0.min(v1) - TEXEL_HALO,
            u0.max(u1) + TEXEL_HALO,
            v0.max(v1) + TEXEL_HALO,
            0,
        )
    }

    /// The shared body of the two extrema queries: `(min, max)` over every mip cell the
    /// rectangle touches, with `halo` extra cells on each side.
    fn mip_cell_extrema(&self, u0: f64, v0: f64, u1: f64, v1: f64, halo: isize) -> (i16, i16) {
        let n = HEIGHT_MIP_DIM as f64;
        let last = HEIGHT_MIP_DIM as isize - 1;
        let cell = |t: f64, h: isize| -> usize {
            let x = (t.clamp(0.0, 1.0) * n).floor() as isize;
            (x + h).clamp(0, last) as usize
        };
        let (cx0, cx1) = (cell(u0, -halo), cell(u1, halo));
        let (cy0, cy1) = (cell(v0, -halo), cell(v1, halo));

        let mut lo = i16::MAX;
        let mut hi = i16::MIN;
        for cy in cy0..=cy1 {
            for cx in cx0..=cx1 {
                let c = cy * HEIGHT_MIP_DIM + cx;
                lo = lo.min(self.min_mip[c]);
                hi = hi.max(self.max_mip[c]);
            }
        }
        (lo, hi)
    }

    pub fn mip_extrema_over(&self, u0: f64, v0: f64, u1: f64, v1: f64) -> (i16, i16) {
        self.mip_cell_extrema(u0.min(u1), v0.min(v1), u0.max(u1), v0.max(v1), 1)
    }
}

impl std::fmt::Debug for HeightTile {
    /// Deliberately does not print 65 536 samples.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HeightTile")
            .field("h_min", &self.h_min)
            .field("h_max", &self.h_max)
            .field("detail", &self.detail)
            .finish_non_exhaustive()
    }
}
