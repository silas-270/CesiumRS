#!/usr/bin/env python3
"""
generate_world_svg.py

Downloads Natural Earth 1:50m vector data (land, lakes, country borders)
and compiles them into an optimized Web Mercator (EPSG:3857) SVG map
for CesiumRS's offline vector mode.

Palette matches a dark-matter / flight-tracker aesthetic:
- Ocean / Background: #0b0f19
- Land: #1a2231
- Inland Lakes: #0b0f19 (matching ocean)
- Country Boundaries: #3b4758 (fine dashed lines)
- Coastlines: #263345 (subtle edge definition)
"""

import argparse
import gzip
import json
import math
import os
import sys
import urllib.request

SOURCES = {
    "land": "https://raw.githubusercontent.com/martynafford/natural-earth-geojson/master/50m/physical/ne_50m_land.json",
    "lakes": "https://raw.githubusercontent.com/martynafford/natural-earth-geojson/master/50m/physical/ne_50m_lakes.json",
    "borders": "https://raw.githubusercontent.com/martynafford/natural-earth-geojson/master/50m/cultural/ne_50m_admin_0_boundary_lines_land.json",
}

CACHE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".cache")
MAX_MERC_LAT = 85.05112877980659


def latlon_to_merc(lat: float, lon: float, size: float) -> tuple[float, float]:
    """Convert WGS84 lat/lon to Web Mercator pixel coordinates in [0, size] x [0, size]."""
    lat = max(min(lat, MAX_MERC_LAT), -MAX_MERC_LAT)
    # Longitude -180..180 -> 0..size
    x = (lon + 180.0) / 360.0 * size

    # Latitude -> Web Mercator y
    lat_rad = math.radians(lat)
    y_merc = math.log(math.tan(math.pi / 4.0 + lat_rad / 2.0))
    y = (1.0 - y_merc / math.pi) / 2.0 * size
    return round(x, 1), round(y, 1)


def fetch_geojson(name: str, url: str) -> dict:
    """Download GeoJSON or read from local cache."""
    os.makedirs(CACHE_DIR, exist_ok=True)
    cache_path = os.path.join(CACHE_DIR, f"{name}.json")

    if os.path.exists(cache_path):
        print(f"[{name}] Loading cached GeoJSON from {cache_path}...")
        with open(cache_path, "r", encoding="utf-8") as f:
            return json.load(f)

    print(f"[{name}] Downloading from {url}...")
    req = urllib.request.Request(url, headers={"User-Agent": "CesiumRS-Tool/1.0"})
    with urllib.request.urlopen(req) as resp:
        data = resp.read()

    with open(cache_path, "wb") as f:
        f.write(data)

    return json.loads(data.decode("utf-8"))


def polygon_to_svg_paths(geometry: dict, size: float) -> list[str]:
    """Convert GeoJSON Polygon or MultiPolygon to SVG path data."""
    gtype = geometry["type"]
    coords = geometry["coordinates"]
    polys = [coords] if gtype == "Polygon" else coords

    subpaths = []
    for poly in polys:
        for ring in poly:
            if len(ring) < 3:
                continue
            cmds = []
            for i, pt in enumerate(ring):
                lon, lat = pt[0], pt[1]
                x, y = latlon_to_merc(lat, lon, size)
                cmd = "M" if i == 0 else "L"
                cmds.append(f"{cmd}{x} {y}")
            cmds.append("Z")
            subpaths.append(" ".join(cmds))
    return subpaths


def lines_to_svg_paths(geometry: dict, size: float) -> list[str]:
    """Convert GeoJSON LineString or MultiLineString to SVG path data."""
    gtype = geometry["type"]
    coords = geometry["coordinates"]
    lines = [coords] if gtype == "LineString" else coords

    subpaths = []
    for line in lines:
        if len(line) < 2:
            continue
        cmds = []
        for i, pt in enumerate(line):
            lon, lat = pt[0], pt[1]
            x, y = latlon_to_merc(lat, lon, size)
            cmd = "M" if i == 0 else "L"
            cmds.append(f"{cmd}{x} {y}")
        subpaths.append(" ".join(cmds))
    return subpaths


def build_world_svg(size: int = 4096) -> str:
    """Build complete SVG world map."""
    print("Generating Web Mercator SVG world map...")

    land_data = fetch_geojson("ne_50m_land", SOURCES["land"])
    lakes_data = fetch_geojson("ne_50m_lakes", SOURCES["lakes"])
    borders_data = fetch_geojson("ne_50m_borders", SOURCES["borders"])

    # 1. Landmasses
    print("Processing landmasses...")
    land_paths = []
    for feat in land_data.get("features", []):
        geom = feat.get("geometry")
        if geom:
            land_paths.extend(polygon_to_svg_paths(geom, size))

    # 2. Lakes
    print("Processing lakes...")
    lake_paths = []
    for feat in lakes_data.get("features", []):
        geom = feat.get("geometry")
        if geom:
            lake_paths.extend(polygon_to_svg_paths(geom, size))

    # 3. Country Borders
    print("Processing country borders...")
    border_paths = []
    for feat in borders_data.get("features", []):
        geom = feat.get("geometry")
        if geom:
            border_paths.extend(lines_to_svg_paths(geom, size))

    land_d = " ".join(land_paths)
    lakes_d = " ".join(lake_paths)
    borders_d = " ".join(border_paths)

    svg_parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {size} {size}" width="{size}" height="{size}">',
        '  <defs>',
        '    <style>',
        '      .ocean { fill: #0b0f19; }',
        '      .land { fill: #1a2231; stroke: #263345; stroke-width: 0.5; stroke-linejoin: round; }',
        '      .lake { fill: #0b0f19; stroke: #1e2838; stroke-width: 0.4; }',
        '      .border { fill: none; stroke: #3b4758; stroke-width: 0.75; stroke-dasharray: 3,2; stroke-linecap: round; stroke-linejoin: round; }',
        '    </style>',
        '  </defs>',
        f'  <!-- Background / Ocean -->',
        f'  <rect class="ocean" width="{size}" height="{size}" />',
        f'  <!-- Landmasses -->',
        f'  <path class="land" d="{land_d}" />',
        f'  <!-- Major Lakes -->',
        f'  <path class="lake" d="{lakes_d}" />',
        f'  <!-- Country Borders -->',
        f'  <path class="border" d="{borders_d}" />',
        '</svg>',
    ]

    return "\n".join(svg_parts)


def main():
    parser = argparse.ArgumentParser(description="Generate Web Mercator SVG world map from Natural Earth")
    parser.add_argument(
        "--out",
        default="assets/maps/world_vector_dark.svg",
        help="Output SVG file path (default: assets/maps/world_vector_dark.svg)",
    )
    parser.add_argument("--size", type=int, default=4096, help="SVG viewBox square dimension (default: 4096)")
    args = parser.parse_args()

    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    svg = build_world_svg(args.size)

    with open(out_path, "w", encoding="utf-8") as f:
        f.write(svg)

    raw_kb = len(svg.encode("utf-8")) / 1024
    gz_bytes = gzip.compress(svg.encode("utf-8"))
    gz_kb = len(gz_bytes) / 1024

    print(f"\n[SUCCESS] Wrote SVG to {out_path}")
    print(f"Size: {raw_kb:.1f} KB uncompressed ({gz_kb:.1f} KB gzipped)")


if __name__ == "__main__":
    main()
