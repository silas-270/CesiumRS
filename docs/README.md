# CesiumRS documentation

Each file covers one subsystem and describes what the code does **and why**. Where a
number or rule is not obvious, the reason is written next to it, because it is usually
what stops a plausible-looking change from being wrong.

Start with [architecture.md](architecture.md); it is the map the others hang off.

| File | What it covers |
|---|---|
| [architecture.md](architecture.md) | The crates, the engine/extension split, threads and command channels, one frame from start to finish, the render passes, desktop versus Android, Cargo features |
| [tiles-and-lod.md](tiles-and-lod.md) | Imagery sources and map styles, the offline vector map, fetching, caches, meshes, which texture a tile shows, the LOD rule and where `lod_factor` comes from, atmospheric fog, prefetch |
| [terrain.md](terrain.md) | Terrarium height tiles, the height cache, the flat/terrain surface-model split, relief meshes and skirts, height-aware culling, culling behind mountains, terrain LOD, the ground reference, runway corridors |
| [culling-implementation.md](culling-implementation.md) | Which globe tiles get drawn: frames, the per-node test sequence, the invariants, and the culling harness |
| [culling-math.md](culling-math.md) | The derivations and proofs behind the culling tests |
| [camera.md](camera.md) | Camera modes, the projection and near plane, input, collision with the ground |
| [models.md](models.md) | The A350 exterior and the 787 flight deck: loading, scale, placement, ground contact, the cockpit displays |
| [lighting.md](lighting.md) | The sun, the moon, the sky, twilight, haze, object lighting, and how the flight's progress drives all of them |
| [labels.md](labels.md) | City labels: the database, selection, style, and how they are drawn among the 3D models |
| [flight-plan.md](flight-plan.md) | How two airports and a duration become a route, a vertical profile, speeds and attitude |
| [route-line.md](route-line.md) | The polyline along the ground track, its full / windowed / hidden modes, and how it follows the terrain |
| [kotlin-integration.md](kotlin-integration.md) | Calling the engine from Kotlin/Android: JNA headless API, JNI live bridge, NDK build |
| [testing.md](testing.md) | The kinds of tests, the culling and LOD harnesses, the capture harnesses, the harness apps |

`culling-implementation.md` is the reference and `culling-math.md` holds the proofs. A bare
section reference such as §3.4 in either file, or in a code comment, points into
`culling-math.md`.
