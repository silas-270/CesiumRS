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
