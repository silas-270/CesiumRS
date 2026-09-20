# Terrain decoder fixtures

Five unmodified Terrarium elevation tiles, fetched once from
`https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png`
(AWS Open Data, no key) on 2026-09-20 and committed so that
`src/testing/terrain/test_height_tiles.rs` can pin the decoder's output
**without ever touching the network**.

| file | tile | covers | why it is here |
|---|---|---|---|
| `everest_z12_3037_1716.png` | z12/3037/1716 | Himalaya, 27.99 N 86.93 E | the highest relief the source has |
| `zugspitze_z12_2172_1433.png` | z12/2172/1433 | Alps, 47.42 N 10.99 E | mid-latitude alpine relief |
| `monterey_coast_z12_661_1599.png` | z12/661/1599 | Monterey coast, 36.6 N 121.9 W | land and sea in one tile — the only fixture where the ocean policy changes `h_min` but not `h_max` |
| `pacific_z12_341_2048.png` | z12/341/2048 | open Pacific, 0 N 150 W | pure bathymetry; the case `OceanPolicy::ClampToZero` exists for |
| `dead_sea_z12_2451_1670.png` | z12/2451/1670 | Dead Sea, 31.5 N 35.5 E | uniformly −412 m: both the "flat tile" case and the documented price of clamping |

372 kB total. Do not re-fetch or re-encode them: the pinned extrema in the test
are the decoder's regression baseline, and a refreshed source tile would move
them silently.

## `pyramid_extrema.csv` — the Phase D1 margin corpus

788 rows, `z,x,y,h_min_m,h_max_m`, one per tile: the **raw** full-tile extrema in
metres, before any [`OceanPolicy`] is applied, fetched from the same source on
2026-09-20.

It exists because `docs/terrain-plan.md` §7's D1 margin — how far a node's
inherited height interval must be widened while its own height tile is still in
flight — is a *measurement*, not an argument, and the measurement needs
parent/child pairs across the whole level range rather than the five isolated z12
tiles above. The corpus walks the z1…z15 chain over 16 regions (Everest,
Zugspitze, Mont Blanc, Monterey, Denali, Aconcagua, Elbrus, Fuji, Aoraki,
Toubkal, Grand Canyon, Trolltunga, Kilimanjaro, the Netherlands, the open
Pacific and eastern Greenland) and adds all four children at each step, giving
720 parent/child pairs.

Extrema rather than tiles: at z ≤ 15 both `height_bounds_for` and the margin
requirement read a tile's *whole-tile* min and max, so 4 bytes per tile carries
everything the measurement uses. 60 tiles' worth of PNGs would have been 4.8 MB
for the same numbers.

`testing::terrain::test_terrain_visibility::d1_inherit_margin_covers_the_corpus`
re-derives the per-level table from this file and fails if
`HEIGHT_INHERIT_MARGIN_M` stops covering it. It touches no network, like
everything else here.
