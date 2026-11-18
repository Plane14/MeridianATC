"""The aircraft state the controller reasons about."""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field


@dataclass
class AircraftState:
    """One snapshot of the aircraft, in the units ATC thinks in.

    Angles are degrees, altitudes feet, speeds knots. Whatever the source
    reports internally is converted before it gets here, so nothing downstream
    has to know that SimConnect prefers radians.
    """

    # --- position ---
    latitude: float = 0.0
    longitude: float = 0.0
    altitude_ft: float = 0.0            # true altitude, MSL
    indicated_altitude_ft: float = 0.0  # what the altimeter reads
    altitude_agl_ft: float = 0.0
    heading_true: float = 0.0
    heading_magnetic: float = 0.0
    track_true: float = 0.0

    # --- motion ---
    ground_speed_kt: float = 0.0
    indicated_airspeed_kt: float = 0.0
    true_airspeed_kt: float = 0.0
    vertical_speed_fpm: float = 0.0
    on_ground: bool = True

    # --- radios ---
    com1_active: float = 0.0
    com1_standby: float = 0.0
    com2_active: float = 0.0
    com2_standby: float = 0.0
    com1_receive: bool = True
    com1_transmit: bool = True
    com2_receive: bool = False
    transponder_code: str = "1200"
    transponder_mode: int = 0           # 0 off, 1 standby, 3 on, 4 alt, 5 ident

    # --- identity ---
    callsign: str = ""                  # ATC ID, e.g. DAL1234 or N172SP
    airline: str = ""
    flight_number: str = ""
    tail_number: str = ""
    type_code: str = ""                 # ICAO type, e.g. B739
    title: str = ""                     # sim's own aircraft title

    # --- configuration ---
    engines_running: bool = False
    parking_brake: bool = False
    gear_down: bool = True
    # IS TAIL DRAGGER (SimConnect) / acf_is_taildragger (X-Plane). None
    # means the source never answered, and callers then fall back to the
    # type code and title (meridianatc.atc.tailwheel.is_tailwheel).
    tail_dragger: bool | None = None
    # Free-castoring nosewheel (Cirrus, Grumman AA, ...): the wheel
    # trails, it does not steer, and the aeroplane turns on its brakes.
    # X-Plane answers via acf_nw_steerdeg1 == 0; SimConnect exposes no
    # flag, so None there and callers fall back to the type table
    # (meridianatc.atc.tailwheel.is_castoring).
    castoring_wheel: bool | None = None
    # The airframe is fitted for water. A landplane's float variant still
    # reports the landplane's code -- a C172 on floats is "C172" -- so
    # the source finds the floats themselves: X-Plane's gear-type table,
    # the aircraft's own title, or water beneath it. The ATC side reads
    # it through aircraft_class.amphibious(type, on_floats).
    on_floats: bool = False
    # SimConnect's SURFACE TYPE under the aircraft (0 concrete, 1 grass,
    # 2 water, ...); X-Plane does not fill it.
    surface_type: int = 0
    flaps_index: int = 0
    # The two switches the crew talk about. Nothing on the radio needs them --
    # a controller never mentions the seatbelt sign -- but the first officer
    # and the cabin do, and saying "the captain has switched off the seatbelt
    # sign" while it is still on is the loudest way this program can show it
    # is reading a script rather than the aeroplane.
    seatbelt_sign: bool = False
    beacon_light: bool = False
    # What kind of aeroplane the model is, for the first officer's
    # checklists (meridianatc.atc.aircraft_class): ENGINE TYPE (-1 until the
    # simulator has said), NUMBER OF ENGINES and CATEGORY.
    engine_type: int = -1
    engine_count: int = 0
    category: str = ""
    # What a checklist answer is checked against (meridianatc.atc.checklist).
    # A default here is not evidence: the first officer reads a variable
    # again on its own before objecting, and says nothing if it has no
    # answer.
    landing_light: bool = False
    taxi_light: bool = False
    nav_light: bool = False
    strobe_light: bool = False
    kohlsman_hpa: float = 0.0
    # FUEL TANK SELECTOR:1, the simulator's tank enumeration (0 off, 1 all,
    # 2 left, 3 right, ... 16 both).
    fuel_selector: int = -1
    mixture_pct: float = 0.0
    autopilot: bool = False
    gps_drives_nav1: bool = False
    pitot_heat: bool = False
    fuel_total_gal: float = 0.0
    spoilers_armed: bool = False
    avionics_master: bool = False
    battery_master: bool = False
    # COM STATUS:1 -- 0 working, 1 not fitted, 2 no electricity, 3 failed --
    # and the main bus in volts; -1 for either until the simulator has said.
    com1_status: int = -1
    main_bus_volts: float = -1.0
    # TRANSPONDER STATE:1: 0 off, 1 standby, 2 test, 3 on, 4 altitude;
    # -1 until the simulator has said.
    transponder_state: int = -1
    # The aeroplane's own speeds, where the model declares them (zero where
    # it does not), for the rotate callout.
    design_takeoff_kt: float = 0.0
    min_rotation_kt: float = 0.0
    # The model's own performance figures, where it declares them (MSFS
    # "Design Performance" variables, all zero where it does not): speeds
    # in knots, rates feet a minute, weights pounds, lengths feet. These
    # are the aeroplane's, not a class's -- the first officer's targets
    # prefer them over any table.
    estimated_cruise_kt: float = 0.0
    typical_descent_fpm: float = 0.0
    design_vs0_kt: float = 0.0
    design_vs1_kt: float = 0.0
    design_vc_kt: float = 0.0
    min_drag_kt: float = 0.0
    barber_pole_kt: float = 0.0
    barber_pole_mach: float = 0.0
    mach_max_operate: float = 0.0
    total_weight_lb: float = 0.0
    max_gross_weight_lb: float = 0.0
    atc_heavy: bool = False
    suggested_runway_takeoff_ft: float = 0.0
    suggested_runway_landing_ft: float = 0.0
    # Rotorcraft: engine torque, per cent, and the lateral cyclic trim --
    # the two figures a collective limit and a wings-level hover need.
    # UNVERIFIED on a running 2024.
    eng_torque_pct: float = 0.0
    rotor_lateral_trim_pct: float = 0.0
    # The engine-instrument reads the autonomous engine watch
    # (atc.powerplant + autonomous/x_the_engines) works against: the
    # gas-producer/fan per cent (TURB ENG N1), the power-turbine per
    # cent where the model splits it out (TURB ENG N2), the turbine
    # temperature Celsius (TURB ENG ITT -- the ITT/MGT/TOT the type's
    # limits are written against), and the piston crankshaft RPM.
    # Zero where the model does not publish them; a missing read is
    # never a limit.
    eng_n1_pct: float = 0.0
    eng_n2_pct: float = 0.0
    eng_itt_c: float = 0.0
    eng_rpm: float = 0.0
    # The cooling watch's instruments: the hottest cylinder-head
    # temperature the engines report (Celsius -- the SimConnect gauge
    # reads engine one's; X-Plane's is the max over the running set),
    # the model's own CHT red line where the sim publishes one, the
    # hottest oil temperature, and how far open the cowl-flap levers
    # are, per cent. Zero where the model reports none of them; a
    # missing read is never a limit.
    eng_cht_c: float = 0.0
    eng_cht_limit_c: float = 0.0
    eng_oil_temp_c: float = 0.0
    eng_cowl_pct: float = 0.0
    # What the first officer's callouts watch (meridianatc.atc.callouts): the
    # bank, signed degrees, right positive; NAV1's localizer and glideslope
    # and how far off them the needles are (+/-127 and +/-119, full scale);
    # the speed set on the autopilot, 0 where there is none.
    bank_deg: float = 0.0
    # The airframe's own readings the autonomous envelope protection works
    # off: the normal load factor (G FORCE / gforce_normal -- 1.0 is also
    # what a model that never reports it leaves here, and a parked
    # aeroplane honestly reads one), the pitch attitude the rotation and
    # the go-around aim at, and the autopilot's own maximum bank setting,
    # which the FO reads back after asking the AP to limit it.
    g_force: float = 1.0
    pitch_deg: float = 0.0
    ap_max_bank_deg: float = 0.0
    # The stall watch's (atc/autonomous/x_the_stall): the horn or shaker
    # where the model wires one, the angle of attack, and the angle the
    # model says the warning sounds at -- 0 where it does not publish,
    # which the watch treats as unpublished, not as never stalling.
    stall_warning: bool = False
    aoa_deg: float = 0.0
    stall_aoa_deg: float = 0.0
    nav1_has_localizer: bool = False
    nav1_cdi: float = 127.0
    nav1_has_glideslope: bool = False
    nav1_gsi: float = 119.0
    # What NAV1 is tuned to, MHz -- what the first officer set for the
    # approach, so it can tell a needle that is alive from one that is not.
    nav1_freq: float = 0.0
    ap_airspeed_kt: float = 0.0
    # AUTOTHROTTLE ACTIVE: whether the sim's own autothrottle is driving
    # the levers -- the FO's throttle servo reads this before touching them.
    autothrottle: bool = False
    # What the first officer notices (meridianatc.atc.notices): engine 1's
    # throttle, per cent; the fuel either side, gallons; the total by
    # weight, pounds.
    throttle_pct: float = 0.0
    fuel_left_gal: float = 0.0
    fuel_right_gal: float = 0.0
    fuel_total_lb: float = 0.0
    # What the tanks hold and what cannot be used, in gallons, and whether
    # the model runs the new [FUEL_SYSTEM] section or the legacy [FUEL]
    # one -- the difference between which SimVars carry the fuel.
    fuel_capacity_gal: float = 0.0
    fuel_unusable_gal: float = 0.0
    new_fuel_system: bool = False
    # ESTIMATED FUEL FLOW in pounds an hour, the model's own figure for
    # what cruise burns, where it declares one.
    est_fuel_flow_pph: float = 0.0
    # AIRSPEED MACH -- the number the high-altitude overspeed watch needs,
    # which knots alone cannot give.
    airspeed_mach: float = 0.0
    # Combustion per engine, for engines two through four (engine one's is
    # engines_running). None where the model does not publish them, so a
    # multi that reports nothing is not treated as having failed.
    eng2_combustion: bool | None = None
    eng3_combustion: bool | None = None
    eng4_combustion: bool | None = None
    # What a helicopter's instruments say (MSFS 2024 "Helicopter
    # Variables"): the collective lever, per cent depressed; the main
    # rotor's per cent of rated RPM; how many main rotors the model has;
    # the tail rotor pedals, per cent; the rotor brake and clutch; and
    # the governor's target, per cent of nominal RPM. UNVERIFIED on a
    # running 2024: the percent-over-100 scaling and which add-ons
    # publish them at all.
    collective_position: float = 0.0
    rotor_rpm_pct: float = 0.0
    num_main_rotors: int = 0
    tail_rotor_pedal_position: float = 0.0
    rotor_brake_active: bool = False
    rotor_brake_handle_pos: float = 0.0
    rotor_brake_available: bool = False
    rotor_clutch_active: bool = False
    rotor_gov_target_pct: float = 0.0
    # The pushback service's own words for what it is doing: whether the
    # stand can have a tug at all, whether one has been asked for and is
    # still coming, and whether it is on the nosewheel now. ``available``
    # is three-state -- None where the simulator never says, which is not
    # a refusal; X-Plane publishes only the attachment, so the other two
    # stay at their defaults there.
    pushback_available: bool | None = None
    pushback_wait: bool = False
    pushback_attached: bool = False

    # --- ambient conditions the sim is actually rendering ---
    ambient_wind_direction: float = 0.0
    ambient_wind_velocity: float = 0.0
    ambient_temperature_c: float = 15.0
    ambient_visibility_m: float = 16000.0
    ambient_humidity_pct: float = 60.0
    sea_level_pressure_hpa: float = 1013.25
    # What the weather is doing to the airframe, as the sim renders it:
    # in cloud or not, what is falling, how much ice the airframe has
    # picked up, and the temperature the probe sees moving -- TOTAL AIR
    # TEMPERATURE is warmer than ambient at speed, which is exactly the
    # figure an icing call wants.
    ambient_in_cloud: bool = False
    ambient_precip_state: int = 0      # AMBIENT PRECIP STATE bitmask
    structural_ice_pct: float = 0.0    # STRUCTURAL ICE PCT
    total_air_temperature_c: float = 15.0
    # The wind in the aeroplane's own axes, knots: X lateral (the
    # crosswind component on the nose), Y vertical (an up or downdraft is
    # the only convective evidence the sim publishes), Z longitudinal
    # (headwind +, tailwind -).
    aircraft_wind_x_kt: float = 0.0
    aircraft_wind_y_kt: float = 0.0
    aircraft_wind_z_kt: float = 0.0
    # PRESSURE ALTITUDE in feet: the density altitude's coarse part, which
    # with the temperature is the whole of "hot and high".
    pressure_altitude_ft: float = 0.0

    # --- bookkeeping ---
    timestamp: float = field(default_factory=time.monotonic)
    connected: bool = False
    source: str = "none"
    # Whether the simulator is paused or the user is in its menus -- which
    # is to say, whether anything is happening in the aeroplane at all.
    # Nothing that watches for a change should believe one while this is set:
    # the aircraft did not take off, it was moved by a menu.
    paused: bool = False
    # Set briefly after a menu/map teleport or slew: the position jumped
    # rather than was flown to, so change-watchers hold for the window.
    relocated: bool = False
    unavailable: tuple[str, ...] = ()

    # ------------------------------------------------------------------

    @property
    def airborne(self) -> bool:
        return not self.on_ground

    @property
    def moving(self) -> bool:
        return self.ground_speed_kt > 1.0

    @property
    def climbing(self) -> bool:
        return self.vertical_speed_fpm > 300

    @property
    def descending(self) -> bool:
        return self.vertical_speed_fpm < -300

    @property
    def position(self) -> tuple[float, float]:
        return (self.latitude, self.longitude)

    @property
    def has_position(self) -> bool:
        """Whether the position looks real rather than an uninitialised zero."""
        return not (abs(self.latitude) < 1e-6 and abs(self.longitude) < 1e-6)

    def distance_to(self, lat: float, lon: float) -> float:
        from ..navdata.build import haversine_nm

        return haversine_nm(self.latitude, self.longitude, lat, lon)

    def bearing_to(self, lat: float, lon: float) -> float:
        from ..navdata.build import bearing_deg

        return bearing_deg(self.latitude, self.longitude, lat, lon)

    def relative_bearing(self, lat: float, lon: float) -> float:
        """Bearing to a point relative to the nose, 0-359."""
        return (self.bearing_to(lat, lon) - self.heading_true + 360.0) % 360.0

    def oclock(self, lat: float, lon: float) -> int:
        """Clock position of a point, as ATC gives traffic."""
        relative = self.relative_bearing(lat, lon)
        hour = int(round(relative / 30.0)) % 12
        return 12 if hour == 0 else hour


def normalize_angle(value: float, source_is_radians: bool | None = None) -> float:
    """Return an angle in degrees, tolerating a source that reports radians.

    SimConnect's native unit for headings is radians, but wrappers vary in
    whether they convert. A heading is indistinguishable from radians only when
    it is under about 6.3, which is a real heading of 006 -- rare enough that
    guessing wrong costs little, and detectable when the caller knows.
    """
    if value is None:
        return 0.0
    value = float(value)
    if source_is_radians is True or (
        source_is_radians is None and -2 * math.pi <= value <= 2 * math.pi
    ):
        value = math.degrees(value)
    return value % 360.0


_FLOAT_WORDS = ("float", "amphib", "seaplane", "pontoon")


def floats_said(title: str) -> bool:
    """Whether the aircraft's own title calls it float-fitted.

    The float variant of a landplane keeps the landplane's type code --
    a C172 on floats still says "C172" -- so the sim's own name for the
    airframe is what gives the floats away.
    """
    text = (title or "").lower()
    return any(word in text for word in _FLOAT_WORDS)
