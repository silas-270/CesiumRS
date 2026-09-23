#!/usr/bin/env python3
"""
generate_world_svg.py

Downloads Natural Earth 1:10m vector data and compiles it into a Web Mercator
(EPSG:3857) SVG world map for CesiumRS's offline vector mode.

Layers, bottom to top:
- ocean (background rect), land, urban areas, lakes, rivers, coastline,
  state/province borders, country borders, airports

Conventions the engine's `SvgTileRenderer` relies on (see svg_renderer.rs):
- The viewBox is SIZE x SIZE (default 2^22) with integer coordinates, so every
  vertex is exact in f32 (~10 m at the equator).
- Stroke widths and dash lengths are in *output pixels*, not SVG units: a 1 px
  border stays 1 px at every zoom level.
- A path's `id` ending in `.z<N>` hides it on tiles below zoom N (the engine's
  512 px tiles). Natural Earth's own `min_zoom` is for 256 px tiles, one level
  deeper, hence the `- 1` in `engine_min_zoom`.
- Airports are zero-area stroke dots (round caps), so they keep a fixed size.

Palette (dark flight-tracker look):
- Ocean #0b0f19, land #1a2231, urban #222b3b, lakes #0f1a2c, rivers and lake
  shores #2a4466, coastline #2b3a50, state borders #3a465c (dashed), country borders #4a5a70
  (dashed), airports #8a9bb3.
"""

import argparse
import gzip
import json
import math
import os
import urllib.request
from collections import defaultdict

NE = "https://raw.githubusercontent.com/martynafford/natural-earth-geojson/master/10m"
SOURCES = {
    "ne_10m_land": f"{NE}/physical/ne_10m_land.json",
    "ne_10m_coastline": f"{NE}/physical/ne_10m_coastline.json",
    "ne_10m_lakes": f"{NE}/physical/ne_10m_lakes.json",
    "ne_10m_lakes_europe": f"{NE}/physical/ne_10m_lakes_europe.json",
    "ne_10m_lakes_north_america": f"{NE}/physical/ne_10m_lakes_north_america.json",
    "ne_10m_rivers_lake_centerlines": f"{NE}/physical/ne_10m_rivers_lake_centerlines.json",
    "ne_10m_rivers_europe": f"{NE}/physical/ne_10m_rivers_europe.json",
    "ne_10m_rivers_north_america": f"{NE}/physical/ne_10m_rivers_north_america.json",
    "ne_10m_admin_0_boundary_lines_land": f"{NE}/cultural/ne_10m_admin_0_boundary_lines_land.json",
    "ne_10m_admin_1_states_provinces_lines": f"{NE}/cultural/ne_10m_admin_1_states_provinces_lines.json",
    "ne_10m_urban_areas": f"{NE}/cultural/ne_10m_urban_areas.json",
    "ne_10m_airports": f"{NE}/cultural/ne_10m_airports.json",
}

CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".cache")
MAX_MERC_LAT = 85.05112877980659

# (layer, CSS class body). Order is draw order.
LAYERS = [
    ("land", "fill: #1a2231; fill-rule: evenodd;"),
    ("urban", "fill: #222b3b; fill-rule: evenodd;"),
    ("lakes", "fill: #0f1a2c; fill-rule: evenodd; stroke: #2a4466; stroke-width: 1;"),
    ("rivers", "fill: none; stroke: #2a4466; stroke-width: 1.2; stroke-linecap: round; stroke-linejoin: round;"),
    ("coast", "fill: none; stroke: #2b3a50; stroke-width: 1; stroke-linejoin: round;"),
    ("states", "fill: none; stroke: #3a465c; stroke-width: 0.8; stroke-dasharray: 2,2; stroke-linejoin: round;"),
    ("borders", "fill: none; stroke: #4a5a70; stroke-width: 1.2; stroke-dasharray: 4,3; stroke-linecap: round; stroke-linejoin: round;"),
    ("airports", "fill: none; stroke: #8a9bb3; stroke-width: 4; stroke-linecap: round;"),
]


def to_merc(lon: float, lat: float, size: int) -> tuple[int, int]:
    """WGS84 lon/lat -> integer Web Mercator canvas coordinates in [0, size]."""
    lat = max(min(lat, MAX_MERC_LAT), -MAX_MERC_LAT)
    x = (lon + 180.0) / 360.0 * size
    y_merc = math.log(math.tan(math.pi / 4.0 + math.radians(lat) / 2.0))
    y = (1.0 - y_merc / math.pi) / 2.0 * size
    return round(x), round(y)


def fetch_geojson(name: str) -> dict:
    """Download GeoJSON or read it from the local cache."""
    os.makedirs(CACHE_DIR, exist_ok=True)
    cache_path = os.path.join(CACHE_DIR, f"{name}.json")
    if not os.path.exists(cache_path):
        print(f"[{name}] Downloading {SOURCES[name]}...")
        req = urllib.request.Request(SOURCES[name], headers={"User-Agent": "CesiumRS-Tool/1.0"})
        with urllib.request.urlopen(req) as resp:
            data = resp.read()
        with open(cache_path, "wb") as f:
            f.write(data)
    with open(cache_path, "r", encoding="utf-8") as f:
        return json.load(f)


def engine_min_zoom(ne_min_zoom, floor: int = 0) -> int:
    """Natural Earth min_zoom (256 px tiles) -> engine zoom (512 px tiles)."""
    if ne_min_zoom is None:
        return floor
    return max(floor, min(9, math.ceil(float(ne_min_zoom)) - 1))


def run_to_path(coords, size: int, close: bool) -> str | None:
    """One ring or line -> `M x y l dx dy ...` with repeated points dropped."""
    pts = []
    for c in coords:
        p = to_merc(c[0], c[1], size)
        if not pts or p != pts[-1]:
            pts.append(p)
    if close and len(pts) > 1 and pts[0] == pts[-1]:
        pts.pop()
    if len(pts) < (3 if close else 2):
        return None
    out = [f"M{pts[0][0]} {pts[0][1]}l"]
    for (ax, ay), (bx, by) in zip(pts, pts[1:]):
        out.append(f"{bx - ax} {by - ay}")
    d = " ".join(out).replace(" -", "-").replace("l ", "l")
    return d + ("z" if close else "")


def geometry_paths(geom: dict, size: int) -> list[str]:
    t, c = geom["type"], geom["coordinates"]
    if t == "Polygon":
        rings, close = c, True
    elif t == "MultiPolygon":
        rings, close = [r for poly in c for r in poly], True
    elif t == "LineString":
        rings, close = [c], False
    elif t == "MultiLineString":
        rings, close = c, False
    else:
        return []
    return [d for d in (run_to_path(r, size, close) for r in rings) if d]


def build_world_svg(size: int) -> str:
    # layer -> engine min zoom -> path data pieces
    buckets: dict[str, dict[int, list[str]]] = defaultdict(lambda: defaultdict(list))

    def add(layer: str, dataset: str, floor: int = 0, default_ne=None):
        print(f"Processing {dataset} -> {layer}...")
        for feat in fetch_geojson(dataset)["features"]:
            geom = feat.get("geometry")
            if not geom:
                continue
            ne_mz = feat["properties"].get("min_zoom", default_ne)
            # Land's "100" bucket is a handful of tiny islets; show them late, not never.
            z = engine_min_zoom(min(float(ne_mz), 9.0) if ne_mz is not None else None, floor)
            buckets[layer][z].extend(geometry_paths(geom, size))

    add("land", "ne_10m_land")
    add("urban", "ne_10m_urban_areas", floor=3)
    add("lakes", "ne_10m_lakes")
    add("lakes", "ne_10m_lakes_europe")
    add("lakes", "ne_10m_lakes_north_america")
    add("rivers", "ne_10m_rivers_lake_centerlines", floor=2)
    add("rivers", "ne_10m_rivers_europe")
    add("rivers", "ne_10m_rivers_north_america")
    add("coast", "ne_10m_coastline")
    add("states", "ne_10m_admin_1_states_provinces_lines", floor=3)
    add("borders", "ne_10m_admin_0_boundary_lines_land")

    print("Processing ne_10m_airports -> airports...")
    for feat in fetch_geojson("ne_10m_airports")["features"]:
        lon, lat = feat["geometry"]["coordinates"][:2]
        x, y = to_merc(lon, lat, size)
        z = max(4, min(8, int(feat["properties"]["scalerank"]) - 1))
        buckets["airports"][z].append(f"M{x} {y}h1")

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {size} {size}" width="{size}" height="{size}">',
        "<style>",
    ]
    parts += [f".{name} {{ {css} }}" for name, css in LAYERS]
    parts += ["</style>", f'<rect fill="#0b0f19" width="{size}" height="{size}"/>']
    for name, _ in LAYERS:
        for z in sorted(buckets[name]):
            d = " ".join(buckets[name][z])
            parts.append(f'<path id="{name}.z{z}" class="{name}" d="{d}"/>')
    parts.append("</svg>")
    return "\n".join(parts)


def main():
    parser = argparse.ArgumentParser(description="Generate Web Mercator SVG world map from Natural Earth")
    parser.add_argument(
        "--out",
        default="assets/maps/world_vector_dark.svg.gz",
        help="Output path; a .gz suffix writes it gzipped (default: assets/maps/world_vector_dark.svg.gz)",
    )
    parser.add_argument("--size", type=int, default=1 << 22, help="viewBox square dimension (default: 2^22)")
    args = parser.parse_args()

    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    svg = build_world_svg(args.size).encode("utf-8")
    gz = gzip.compress(svg, 9)
    with open(out_path, "wb") as f:
        f.write(gz if out_path.endswith(".gz") else svg)

    print(f"\n[SUCCESS] Wrote {out_path}")
    print(f"Size: {len(svg) / 1024:.1f} KB uncompressed ({len(gz) / 1024:.1f} KB gzipped)")


if __name__ == "__main__":
    main()
