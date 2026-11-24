import struct
import time

from meridianatc.weather.enroute import (
    METERS_TO_FEET,
    HazardCell,
    RadarPoint,
    avoidance_plan,
    scan_route,
    sigmet_cells,
    xplane_cells,
)


def test_xplane_radar_cells_require_tall_or_strong_precipitation():
    now = time.time()
    points = [
        RadarPoint(50.0, 8.0, 1000.0, 12000.0, .8, .5, now),
        RadarPoint(50.01, 8.0, 1000.0, 32000.0, .8, .8, now),
    ]
    cells = xplane_cells(points, altitude_ft=20000.0, now=now)
    assert len(cells) == 1
    assert cells[0].top_ft >= 32000.0
    assert cells[0].source == "xplane-radar"


def test_xplane_radar_packet_layout_is_tightly_packed_xp12():
    packet = struct.pack("<4sxffffff", b"RADR", 50.0, 8.0,
                         1000.0, 9000.0, .7, .8)
    assert len(packet) == 29
    _tag, lat, lon, base_m, top_m, clouds, precip = struct.unpack(
        "<4sxffffff", packet)
    assert (round(lat, 1), round(lon, 1)) == (50.0, 8.0)
    assert base_m * METERS_TO_FEET > 3200.0
    assert round(top_m) == 9000
    assert abs(clouds - .7) < 1e-6
    assert abs(precip - .8) < 1e-6


def test_scan_route_finds_forward_cell_not_behind_cell():
    ahead = HazardCell(50.5, 8.0, 5.0, .8, "test", "cell")
    behind = HazardCell(49.5, 8.0, 5.0, .8, "test", "cell")
    hits, coverage = scan_route([ahead, behind], 50.0, 8.0, 0.0)
    assert hits == [ahead]
    assert coverage > 0.0


def test_avoidance_plan_gives_side_degrees_and_severe_clearance():
    cell = HazardCell(50.4, 8.02, 5.0, .85, "test", "severe cell")
    plan = avoidance_plan([cell], 50.0, 8.0, 0.0, 30000.0)
    assert plan.threatening
    assert plan.side in ("left", "right")
    assert 20 <= plan.degrees <= 90
    assert plan.miles_nm >= 15
    assert 0.0 <= plan.heading_true < 360.0


def test_sigmet_polygon_is_explicit_convection_and_altitude_filtered():
    rows = [{
        "hazard": "CONV",
        "altitudeLow1": 5000,
        "altitudeHi1": 39000,
        "geometry": {"type": "Polygon", "coordinates": [[
            [8.0, 50.0], [8.5, 50.0], [8.5, 50.5], [8.0, 50.5], [8.0, 50.0],
        ]]},
    }]
    cells = sigmet_cells(rows, altitude_ft=30000.0)
    assert len(cells) == 1
    assert cells[0].contains(50.25, 8.25)
    assert not sigmet_cells(rows, altitude_ft=45000.0)
