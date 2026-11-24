import struct
import time

from meridianatc.weather.enroute import (
    METERS_TO_FEET,
    HazardCell,
    RadarPoint,
    _course_geometry_cost,
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
    # The packet is longitude first, despite the usual aviation habit of
    # writing positions latitude/longitude.
    packet = struct.pack("<4sxffffff", b"RADR", 8.0, 50.0,
                         1000.0, 9000.0, .7, .8)
    assert len(packet) == 29
    assert len(struct.pack("<4sx10s", b"RADR", b"10")) == 15
    _tag, lon, lat, base_m, top_m, clouds, precip = struct.unpack(
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


def test_metar_cells_require_explicit_convection_not_plain_rain():
    from meridianatc.weather.enroute import metar_convective_cells

    rows = [
        {"lat": 50.0, "lon": 8.0, "icaoId": "EDDF",
         "rawOb": "EDDF 242050Z 12008KT 9999 FEW040 10/05 Q1015"},
        {"lat": 50.1, "lon": 8.1, "icaoId": "EDDF",
         "rawOb": "EDDF 242050Z 12008KT 9999 -RA SCT020 10/05 Q1015"},
        {"lat": 50.2, "lon": 8.2, "icaoId": "EDDF",
         "rawOb": "EDDF 242050Z 12008KT 9999 TSRA SCT020CB 10/05 Q1015"},
        {"lat": 50.3, "lon": 8.3, "icaoId": "EDDF",
         "rawOb": "EDDF 242050Z 12008KT 9999 VCTS 10/05 Q1015"},
    ]
    cells = metar_convective_cells(rows)
    assert len(cells) == 2
    assert all(c.source == "awc-metar" for c in cells)


def test_awc_dict_coordinates_and_expired_sigmet():
    now = time.time()
    row = {
        "hazard": "TS",
        "validTimeFrom": now - 100,
        "validTimeTo": now + 100,
        "base": 0,
        "top": 40000,
        "coords": [
            {"lon": 8.0, "lat": 50.0}, {"lon": 8.5, "lat": 50.0},
            {"lon": 8.5, "lat": 50.5}, {"lon": 8.0, "lat": 50.0},
        ],
        "rawSigmet": "EMBD TS FCST",
    }
    assert len(sigmet_cells([row], altitude_ft=30000)) == 1
    expired = dict(row, validTimeTo=now - 1000)
    assert not sigmet_cells([expired], altitude_ft=30000, now=now)


def test_xplane_high_altitude_overlap_not_demanding_tops_above_aircraft():
    now = time.time()
    points = [RadarPoint(50.0, 8.0, 1000.0, 34000.0, .8, .5, now)]
    assert xplane_cells(points, altitude_ft=35000.0, now=now)
    below = [RadarPoint(50.0, 8.0, 1000.0, 25000.0, .8, .5, now)]
    assert not xplane_cells(below, altitude_ft=35000.0, now=now)


def test_metar_embedded_cb_cloud_group_is_explicit_convection():
    from meridianatc.weather.enroute import metar_convective_cells
    rows = [{"lat": 50.0, "lon": 8.0, "icaoId": "EDDF",
             "rawOb": "EDDF 242050Z 12008KT 9999 SCT020CB 10/05 Q1015"}]
    assert len(metar_convective_cells(rows)) == 1


def test_pirep_requires_whole_weather_words():
    from meridianatc.weather.enroute import pirep_cells
    negative = [{"lat": 50.0, "lon": 8.0, "rawOb": "UA /OV X/TM 1/FL300/TP B738/TB NEG/RM KNOTS"}]
    positive = [{"lat": 50.0, "lon": 8.0, "rawOb": "UUA /OV X/TM 1/FL300/TP B738/TB SEV/RM HAIL TS"}]
    assert not pirep_cells(negative)
    assert len(pirep_cells(positive)) == 1


def test_scan_route_uses_sigmet_polygon_vertices_not_only_centroid():
    polygon = (
        (50.0, 8.3), (50.2, 8.3), (52.0, 10.0), (52.0, 10.2), (50.0, 8.3))
    cell = HazardCell(51.0, 9.2, 5.0, .8, "test", "broad polygon",
                      polygon=polygon)
    hits, _coverage = scan_route([cell], 50.0, 8.0, 0.0, ahead_nm=80.0)
    assert hits == [cell]


def test_avoidance_heading_does_not_turn_into_the_next_scanned_cell():
    # The nearest cell is slightly right of a northbound course, so the
    # first side choice is left; the second cell sits on that leftward
    # heading. The executable course must use the rightward escape instead.
    cells = [
        HazardCell(50.0 + 20.0 / 60.0, 8.0 + 3.0 / 38.0, 2.0, .5,
                   "test", "nearest-right"),
        HazardCell(50.0 + 35.0 / 60.0, 8.0 - 18.0 / 38.0, 2.0, .5,
                   "test", "leftward-course"),
    ]
    plan = avoidance_plan(cells, 50.0, 8.0, 0.0, 30000.0)
    assert plan.side == "right"
    assert abs(plan.heading_true - 30.0) < 1e-6


def test_avoidance_never_scores_inside_a_cell_as_marginal_clip():
    # The rightward course is mathematically just inside the far cell's body,
    # while the leftward course only clips the nearer cell's safety clearance.
    # Those are not two degrees of the same penalty: the body is forbidden.
    cells = [
        HazardCell(0.33206489936391514, -0.029051914249219386,
                   4.0, .5, "test", "left-clearance-clip"),
        HazardCell(1.0103629710818451, 0.5833333333333333,
                   4.0, .5, "test", "right-body penetration"),
    ]
    plan = avoidance_plan(cells, 0.0, 0.0, 0.0, 30000.0)
    assert plan.side == "left"
    assert abs(plan.heading_true - 330.0) < 1e-6



def test_course_geometry_ignores_circle_behind_the_ray():
    # The disk is close enough behind the aircraft that pointwise forward/lateral
    # approximations can mistake it for a future hit, but a northbound ray never
    # intersects it.
    cell = HazardCell(-1.0 / 60.0, 0.1, 5.0, .8, "test", "behind")
    assert _course_geometry_cost(cell, 0.0, 0.0, 0.0, 80.0) == 0.0


def test_course_geometry_uses_polygon_edges_not_inflated_vertices():
    # Every vertex is east of the northbound track and outside the clearance
    # margin. Treating each vertex as a circle with the whole cell radius would
    # invent a penetration that the polygon itself never has.
    polygon = ((20.0 / 60.0, 10.0 / 60.0),
               (20.0 / 60.0, 25.0 / 60.0),
               (30.0 / 60.0, 25.0 / 60.0))
    cell = HazardCell(.35, .3, 12.0, .5, "test", "polygon", polygon=polygon)
    assert _course_geometry_cost(cell, 0.0, 0.0, 0.0, 80.0) == 0.0


def test_inside_a_cell_still_avoids_the_second_cell_while_escaping():
    cells = [
        HazardCell(0.0, 0.0, 5.0, .8, "test", "current-cell"),
        HazardCell(.25, -26.0 / 60.0, 4.0, .8, "test", "leftward-cell"),
    ]
    plan = avoidance_plan(cells, 0.0, 0.0, 0.0, 30000.0)
    assert plan.side == "right"



def test_course_geometry_wraps_longitude_across_the_antimeridian():
    cell = HazardCell(0.0, -179.93, 2.0, .8, "test", "dateline")
    cost = _course_geometry_cost(cell, 0.0, 179.9, 90.0, 80.0)
    assert cost > 10.0


def test_concave_polygon_reentry_is_penetration_not_escape():
    # A U-shaped polygon open northward. From the western arm an eastbound
    # course leaves into the notch but then enters the eastern arm; westbound
    # is the clean escape.
    xy = [(-30, -20), (30, -20), (30, 20), (10, 20), (10, 0),
          (-10, 0), (-10, 20), (-30, 20)]
    polygon = tuple((y / 60.0, x / 60.0) for x, y in xy)
    cell = HazardCell(0.0, 0.0, 30.0, .8, "test", "concave",
                      polygon=polygon)
    lat, lon = 10.0 / 60.0, -20.0 / 60.0
    east = _course_geometry_cost(cell, lat, lon, 90.0, 80.0)
    west = _course_geometry_cost(cell, lat, lon, 270.0, 80.0)
    assert east > 10.0
    assert west < 5.0
