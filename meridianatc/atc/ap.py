"""The First Officer's hands: every command the aeroplane is sent.

One place, so the phase machine in :mod:`meridianatc.atc.autonomous` never has
to know which of the three ways a command can reach the sim:

* **Events.** The SDK's key events (``AP_HDG_HOLD_ON``, ``GEAR_DOWN``,
  ``THROTTLE_IDLE`` ...), mapped once each and sent as a DWORD. What the
  autopilot's mode knobs and the aircraft's switches are.

* **Variables.** ``SimConnect_SetDataOnSimObject`` through
  :meth:`SimConnectSource.set_variable`, for what no event expresses: the
  throttle lever's per-cent, a control surface's position where an event
  would only step it.

* **Axes.** The ``*_SET`` control events (``ELEVATOR_SET``,
  ``AILERON_SET``, ``RUDDER_SET``), which take a position in the
  simulator's own -16383..16384 scale -- what the FO's hand-flying needs
  for the moments the autopilot cannot fly: rotation, the flare, the
  go-around's initial pitch.

Two autopilot styles, as ``immersion.autonomous_ap_mode`` sets them:

* ``managed`` -- what an airline flies: flight-level change for the climb
  and the descent (``FLIGHT_LEVEL_CHANGE_ON``), NAV for the route, APR for
  the approach, and the autothrottle where the model has one
  (``AUTO_THROTTLE_TO_GA`` for the go-around's power).

* ``direct`` -- what a light aeroplane's steam gauge AP does: heading
  hold, altitude hold, vertical speed, indicated airspeed, each set
  explicitly.

Every event name goes through :meth:`SimConnectSource._send`, which maps
it once and answers False for a name the simulator does not have, so an
aircraft or sim that lacks a mode simply keeps the previous one rather
than throwing mid-flight. Nothing here reads the state; the caller does,
and asks again next tick.
"""

from __future__ import annotations

import logging

log = logging.getLogger(__name__)

# The simulator's own control scale: -16383 full negative, +16384 full
# positive, 0 centred. ELEVATOR_SET's -16K is full down, so a positive
# value here is a pull.
AXIS_MIN = -16383
AXIS_MAX = 16384


def _axis(position: float) -> int:
    """A -1..1 control position in the simulator's axis units."""
    return int(round(max(-1.0, min(1.0, position)) * AXIS_MAX))


class FlightDirector:
    """Sends the aeroplane where it is told, by the best means it has."""

    def __init__(self, sim, mode: str = "managed"):
        self.sim = sim
        self.mode = "managed" if mode == "managed" else "direct"
        # The logical modes already asked for, so a 1 Hz loop does not
        # resend what the sim already has. Cleared by ``reset`` between
        # legs; individual modes are also dropped when disengaged.
        self._sent: set[str] = set()

    # -- plumbing ---------------------------------------------------------

    def reset(self) -> None:
        """A new leg: nothing is assumed engaged until it is asked for."""
        self._sent.clear()

    def send(self, name: bytes, value: int = 0) -> bool:
        """One event, through the source's map-once-and-probe path."""
        if self.sim is None or not getattr(self.sim, "connected", False):
            return False
        try:
            return bool(self.sim._send(name, value))
        except Exception as exc:
            log.debug("sim event %s failed: %s", name, exc)
            return False

    def once(self, key: str, name: bytes, value: int = 0) -> bool:
        """An event sent at most once until ``reset`` or ``forget``."""
        if key in self._sent:
            return True
        if self.send(name, value):
            self._sent.add(key)
            return True
        return False

    def forget(self, *keys: str) -> None:
        for key in keys:
            self._sent.discard(key)

    def write(self, variable: str, value: float,
              unit: str | None = None) -> bool:
        """A sim variable, where the source can write one.

        ``unit`` is handed through where the source takes one -- the
        simulator's lever and surface variables live in named units, and
        the wrong one is a value off by a hundred.
        """
        if self.sim is None or not hasattr(self.sim, "set_variable"):
            return False
        try:
            if unit is None:
                return bool(self.sim.set_variable(variable, value))
            return bool(self.sim.set_variable(variable, value, unit=unit))
        except TypeError:
            try:
                return bool(self.sim.set_variable(variable, value))
            except Exception as exc:
                log.debug("simvar %s failed: %s", variable, exc)
                return False
        except Exception as exc:
            log.debug("simvar %s failed: %s", variable, exc)
            return False

    # -- the master --------------------------------------------------------

    def autopilot(self, on: bool = True) -> bool:
        """The AP master switch, set explicitly rather than toggled.

        ``AP_MASTER`` is a toggle: asking for "on" with the AP already
        engaged would switch it off. ``AUTOPILOT_ON``/``AUTOPILOT_OFF``
        are the explicit setters; the toggle stands as the last fallback
        for a sim that never mapped them.
        """
        ok = self.send(b"AUTOPILOT_ON" if on else b"AUTOPILOT_OFF", 0)
        return ok or self.send(b"AP_MASTER", 0)

    def flight_director(self, on: bool = True) -> bool:
        """The flight director, set rather than toggled where a variable
        or explicit event exists; ``TOGGLE_FLIGHT_DIRECTOR`` last."""
        ok = self.write("AUTOPILOT FLIGHT DIRECTOR ACTIVE", 1.0 if on else 0.0)
        return ok or self.send(b"TOGGLE_FLIGHT_DIRECTOR", 0)

    def yaw_damper(self, on: bool = True) -> bool:
        return self.send(b"YAW_DAMPER_ON" if on else b"YAW_DAMPER_OFF", 0)

    # -- lateral -----------------------------------------------------------

    def heading(self, degrees: float) -> bool:
        """Command a heading through the heading bug and HDG hold.

        ``AP_HEADING_BUG_SET_EX1`` is the MSFS 2024 setter: the parameter
        is the heading scaled by 16384/360 with the slot index in [1],
        which this single-DWORD send leaves at its default. The older
        ``AP_HDG_VAR_SET`` name is tried after it for sims that still map
        it, and ``HEADING_BUG_SET`` last of all.
        """
        bug = int(round((degrees % 360.0) * AXIS_MAX / 360.0))
        ok = self.send(b"AP_HEADING_BUG_SET_EX1", bug)
        if not ok:
            ok = self.send(b"AP_HDG_VAR_SET", int(round(degrees)) % 360)
        if not ok:
            ok = self.send(b"HEADING_BUG_SET", int(round(degrees)) % 360)
        self.send(b"AP_HDG_HOLD_ON", 0)
        return ok

    def nav(self, on: bool = True) -> bool:
        """NAV1 lateral guidance: the flight plan, legs and all. A NAV
        hold with the receiver still following a VOR tracks the beacon,
        not the route -- ``GPS DRIVES NAV1`` puts the flight plan on the
        needle before the hold is asked to fly it."""
        if on:
            self.write("GPS DRIVES NAV1", 1.0)
        return self.send(b"AP_NAV1_HOLD_ON" if on else b"AP_NAV1_HOLD_OFF", 0)

    def nav_source(self, index: int = 1) -> bool:
        """Which NAV receiver the holds follow (1 or 2)."""
        return self.send(b"AP_NAV_SELECT_SET", int(index))

    def bank_hold(self, on: bool = True) -> bool:
        return self.send(b"AP_BANK_HOLD_ON" if on else b"AP_BANK_HOLD_OFF", 0)

    def max_bank(self, degrees: float) -> bool:
        """The autopilot's own bank limit, degrees.

        ``AP_MAX_BANK_ANGLE_SET`` sets the angle directly, overriding the
        model's bank-mode index; the sim's answer is read back on
        ``AUTOPILOT MAX BANK``. UNVERIFIED on a running 2024 and ignored
        by models whose AP has no bank control at all -- a sim or a
        radio stack without the event answers False and nothing else
        changes, which is the failure this API asks for.
        """
        return self.send(b"AP_MAX_BANK_ANGLE_SET", int(round(degrees)))

    # -- vertical ----------------------------------------------------------

    def altitude(self, feet: float) -> bool:
        """The altitude bug, in feet (``AP_ALT_VAR_SET_ENGLISH``)."""
        self.send(b"AP_ALT_VAR_SET_ENGLISH", int(round(feet)))
        return True

    def alt_hold(self, on: bool = True) -> bool:
        return self.send(b"AP_ALT_HOLD_ON" if on else b"AP_ALT_HOLD_OFF", 0)

    def vertical_speed(self, fpm: float) -> bool:
        """A vertical-speed command, feet a minute."""
        self.send(b"AP_VS_VAR_SET_ENGLISH", int(round(fpm)))
        return True

    def vs_hold(self, on: bool = True) -> bool:
        return self.send(b"AP_VS_ON" if on else b"AP_VS_OFF", 0)

    def level_change(self, on: bool = True) -> bool:
        """The managed climb/descent: power for the profile, airspeed held."""
        name = (b"FLIGHT_LEVEL_CHANGE_ON" if on
                else b"FLIGHT_LEVEL_CHANGE_OFF")
        return self.send(name, 0)

    def att_hold(self, on: bool = True) -> bool:
        """Wing leveler with pitch hold: wings level, attitude kept."""
        return self.send(b"AP_ATT_HOLD_ON" if on else b"AP_ATT_HOLD_OFF", 0)

    def wing_leveler(self, on: bool = True) -> bool:
        return self.send(b"AP_WING_LEVELER_ON" if on else b"AP_WING_LEVELER_OFF", 0)

    def pitch_leveler(self, on: bool = True) -> bool:
        return self.send(b"AP_PITCH_LEVELER_ON" if on else b"AP_PITCH_LEVELER_OFF", 0)

    def radio_altitude(self, on: bool = True) -> bool:
        return self.send(b"AP_ALT_RADIO_MODE_ON" if on else b"AP_ALT_RADIO_MODE_OFF", 0)

    # -- speed -------------------------------------------------------------

    def speed(self, knots: float) -> bool:
        """Indicated airspeed bug.

        The SDK documents ``AP_SPD_VAR_SET``'s value in whole knots and
        ``AP_SPD_VAR_SET_EX1``'s in hundredths of a knot -- the EX1
        variant keeps the decimal part the plain event would round
        away. Both are tried, the precise one first, because a sim
        without it should still get its 140 rather than 1.4.
        """
        self.send(b"AP_SPD_VAR_SET", int(round(knots)))
        self.send(b"AP_SPD_VAR_SET_EX1", int(round(knots * 100.0)))
        return True

    def airspeed_hold(self, on: bool = True) -> bool:
        return self.send(b"AP_AIRSPEED_ON" if on else b"AP_AIRSPEED_OFF", 0)

    def mach_hold(self, on: bool = True) -> bool:
        return self.send(b"AP_MACH_ON" if on else b"AP_MACH_OFF", 0)

    # -- approach ----------------------------------------------------------

    def localizer(self, on: bool = True) -> bool:
        if on:
            # The needles, not the flight plan: a GPS still driving NAV1
            # has the hold tracking legs instead of the localizer.
            self.write("GPS DRIVES NAV1", 0.0)
        return self.send(b"AP_LOC_HOLD_ON" if on else b"AP_LOC_HOLD_OFF", 0)

    def approach(self, on: bool = True) -> bool:
        """LOC and glideslope together, armed or engaged."""
        if on:
            # As the localizer: APR follows the CDI's source, and the
            # source is the radio, not the GPS flight plan.
            self.write("GPS DRIVES NAV1", 0.0)
        return self.send(b"AP_APR_HOLD_ON" if on else b"AP_APR_HOLD_OFF", 0)

    # -- takeoff and go-around power ---------------------------------------

    def autothrottle_arm(self) -> bool:
        """Arm the autothrottle, once: ``AUTO_THROTTLE_ARM`` is a toggle,
        so asking twice is asking to disarm."""
        return self.once("autothrottle_armed", b"AUTO_THROTTLE_ARM", 0)

    def autothrottle_disconnect(self) -> bool:
        """Disconnect, explicitly: the disconnect event only ever
        switches the autothrottle off."""
        self.forget("autothrottle_armed")
        return self.send(b"AUTO_THROTTLE_DISCONNECT", 0)

    def toga(self, on: bool = True) -> bool:
        """Autothrottle takeoff/go-around mode where the model has one.

        ``AUTO_THROTTLE_TO_GA`` cycles ranges on every press, so it is
        pressed once per re-arm -- ``forget("toga")`` lets the next
        go-around press it again.
        """
        if on:
            return self.once("toga", b"AUTO_THROTTLE_TO_GA", 0)
        self.forget("toga")
        return True

    def go_around(self) -> bool:
        """The TO/GA press itself, where the model has the switch: the
        autopilot's go-around mode and the autothrottle's, together."""
        return self.toga(True)

    def throttle(self, position: float) -> bool:
        """The thrust levers, 0..1 of travel: 0 idle, 1 takeoff. Every
        engine's lever variable first -- a twin whose second lever was
        never written runs at whatever the pilot left it -- the shared
        axis event after it, so a model that takes the variables does
        not also get the event's step."""
        position = max(0.0, min(1.0, position))
        ok = False
        for engine in range(1, 5):
            # The simulator's lever variable is "Percent over 100" --
            # the same fraction this method takes, so that unit is asked
            # for first. A source without named units (X-Plane's table
            # takes the 0..100 it always did) answers False and gets the
            # percent value it expects. Guessing the scale got a 0.57
            # lever read back as 5700% and the engine pinned to the
            # stop on every taxi call.
            name = f"GENERAL ENG THROTTLE LEVER POSITION:{engine}"
            if self.write(name, position, unit="Percent over 100") \
                    or self.write(name, position * 100.0):
                ok = True
        if ok:
            return True
        # ``AXIS_THROTTLE_SET`` puts the engine index in parameter [0],
        # which a one-DWORD send cannot reach -- the value would land in
        # the index slot. ``THROTTLE_SET``'s single value is the right
        # shape here; the numbered axes cover a model that takes only
        # per-engine commands -- and those axes are signed like a
        # joystick's, where the unipolar value asks 50% for idle.
        value = int(round(position * 16383.0))
        ok = self.send(b"THROTTLE_SET", value)
        axis = int(round(-16383.0 + position * 32766.0))
        for engine in range(1, 5):
            if self.send(f"AXIS_THROTTLE{engine}_SET".encode(), axis):
                ok = True
        return ok

    def prop(self, position: float) -> bool:
        """The propeller levers, 0..1 of travel: 0 feathered, 1 full
        forward. Written the way the throttles are -- every engine's
        lever variable, so a twin whose second prop was left at low
        pitch follows the first instead of governing that engine to a
        propeller that cannot make power. A model that takes no lever
        variable gets the events, like the throttles' fallback: without
        them the prop levers stay wherever the pilot's quadrant is
        parked and the engines make no thrust anyone can taxi on."""
        position = max(0.0, min(1.0, position))
        ok = False
        for engine in range(1, 5):
            name = f"GENERAL ENG PROPELLER LEVER POSITION:{engine}"
            if self.write(name, position, unit="Percent over 100") \
                    or self.write(name, position * 100.0):
                ok = True
        if ok:
            return True
        # The prop events are signed like the stick's axes: -16383 the
        # floor, +16383 full forward. At full forward -- the only place
        # a prop lever is ever asked to go -- either scale ends at the
        # same stop.
        axis = int(round(-16383.0 + position * 32766.0))
        ok = self.send(b"PROP_PITCH_SET", axis)
        for engine in range(1, 5):
            if self.send(f"PROP_PITCH{engine}_SET".encode(), axis):
                ok = True
        if self.send(b"AXIS_PROPELLER_SET", axis):
            ok = True
        return ok

    def mixture(self, position: float) -> bool:
        """The mixture levers, 0..1 of travel: 0 cut-off, 1 full rich.
        Every engine -- writing only the first leaves a twin's second
        engine on whatever the pilot's quadrant last said. The events
        stand under the variables for the same reason the throttle's
        do: a lever the variable cannot reach still has to move."""
        position = max(0.0, min(1.0, position))
        ok = False
        for engine in range(1, 5):
            name = f"GENERAL ENG MIXTURE LEVER POSITION:{engine}"
            if self.write(name, position, unit="Percent over 100") \
                    or self.write(name, position * 100.0):
                ok = True
        if ok:
            return True
        # ``MIXTURE_SET`` and its numbered forms take 0..16383.
        value = int(round(position * 16383.0))
        ok = self.send(b"MIXTURE_SET", value)
        for engine in range(1, 5):
            if self.send(f"MIXTURE{engine}_SET".encode(), value):
                ok = True
        return ok

    def throttle_idle(self) -> bool:
        return self.throttle(0.0)

    def throttle_cut(self) -> bool:
        ok = self.throttle(0.0)
        self.send(b"THROTTLE_CUT", 0)
        return ok

    def takeoff_power(self) -> bool:
        """Full forward, once per re-arm: ``THROTTLE_FULL`` re-reads the
        throttle ranges on every press on the models that honour it, and
        a 4 Hz loop would keep re-flooring the levers."""
        return self.once("takeoff_power", b"THROTTLE_FULL", 0)

    # -- flight controls, for the moments the AP cannot fly -----------------

    def aileron(self, position: float) -> bool:
        """-1 full left .. +1 full right."""
        return self.send(b"AILERON_SET", _axis(position))

    def elevator(self, position: float) -> bool:
        """-1 full down .. +1 full up (a pull is positive)."""
        return self.send(b"ELEVATOR_SET", _axis(position))

    def rudder(self, position: float) -> bool:
        """-1 full left .. +1 full right; ignored where gyro controls
        are enabled, which is the pilot's own business then."""
        return self.send(b"AXIS_RUDDER_SET", _axis(position))

    def steering(self, position: float) -> bool:
        """The nosewheel, -1 full left .. +1 full right.

        Rudder alone cannot turn a stopped aeroplane: on most singles
        the nosewheel is steered by ``STEERING_SET`` (-16383..16383),
        not by the rudder axis -- which is why taxi tracks wandered.
        """
        value = int(round(max(-1.0, min(1.0, position)) * 16383.0))
        ok = self.send(b"STEERING_SET", value)
        return self.send(b"AXIS_STEERING_SET", _axis(position)) or ok

    def center_controls(self) -> bool:
        ok = self.send(b"CENTER_AILER_RUDDER", 0)
        self.send(b"ELEVATOR_SET", 0)
        return ok

    # -- configuration ------------------------------------------------------

    def flaps(self, position: float) -> bool:
        """Flap handle, 0 retracted .. 1 full, nearest detent.

        ``FLAPS_SET`` takes 0..16383 up-to-down; the signed axis event
        ``AXIS_FLAPS_SET`` (-16383 up .. +16383 down) is the fallback,
        and the old code's unsigned value left the retracted end of it
        sitting at mid-flap.
        """
        position = max(0.0, min(1.0, position))
        ok = self.send(b"FLAPS_SET", int(round(position * 16383.0)))
        if not ok:
            ok = self.send(b"AXIS_FLAPS_SET",
                           int(round(-16383.0 + position * 32766.0)))
        return ok

    def gear(self, down: bool) -> bool:
        return self.send(b"GEAR_DOWN" if down else b"GEAR_UP", 0)

    def gear_set(self, position: float) -> bool:
        """``GEAR_SET`` takes 0 up or 1 down, not an axis value: the old
        ``position * 16384`` asked the gear for position 16384, which no
        gear understands. Callers pass a fraction; the handle is binary.
        """
        return self.send(b"GEAR_SET", 1 if position >= 0.5 else 0)

    def spoilers(self, position: float) -> bool:
        """Speedbrake handle, 0 retracted .. 1 deployed
        (``SPOILERS_SET``: 0..16383)."""
        position = max(0.0, min(1.0, position))
        return self.send(b"SPOILERS_SET", int(round(position * 16383.0)))

    def cowl_flaps(self, position: float, engine: int | None = None) -> bool:
        """The cowl-flap levers, 0 closed .. 1 full open.

        Per-engine ``RECIP ENG COWL FLAP POSITION:n`` percent first --
        the variable exists only on air-cooled piston models that carry
        the levers, which is exactly the set the caller wants to reach --
        then the ``COWLFLAPn_SET`` axis events (0..16383) for a model
        that maps only those. ``engine=None`` writes all four lever
        slots; a model with fewer engines ignores the rest.
        """
        position = max(0.0, min(1.0, position))
        ok = False
        engines = range(engine, engine + 1) if engine is not None \
            else range(1, 5)
        for index in engines:
            name = f"RECIP ENG COWL FLAP POSITION:{index}"
            if self.write(name, position, unit="Percent over 100") \
                    or self.write(name, position * 100.0):
                ok = True
        if ok:
            return True
        value = int(round(position * 16383.0))
        engines = range(engine, engine + 1) if engine is not None \
            else range(1, 5)
        for index in engines:
            if self.send(f"COWLFLAP{index}_SET".encode(), value):
                ok = True
        return ok

    def spoilers_arm(self, on: bool = True) -> bool:
        return self.send(b"SPOILERS_ARM_ON" if on else b"SPOILERS_ARM_OFF", 0)

    # -- ground --------------------------------------------------------------

    def parking_brake(self, on: bool) -> bool:
        return self.send(b"PARKING_BRAKE_SET", 1 if on else 0)

    def brakes(self, position: float) -> bool:
        """The toe brakes, 0 released .. 1 held hard.

        ``BRAKES_LEFT``/``BRAKES_RIGHT`` are step events -- they take no
        value, so the old axis-style ``16384`` was noise and the brakes
        stayed wherever they happened to be. The absolute positions go
        through ``AXIS_LEFT_BRAKE_SET``/``AXIS_RIGHT_BRAKE_SET``
        (-16383 released .. +16383 full), and where a model maps neither
        the plain ``BRAKES`` event at least presses both together.
        """
        position = max(0.0, min(1.0, position))
        return self.brakes_split(position, position)

    def brakes_split(self, left_pos: float, right_pos: float) -> bool:
        """The toe brakes, independently -- differential braking.

        0 released .. 1 held hard on each pedal. Equal demands are what
        :meth:`brakes` already does; the split exists for the work only
        one wheel can do: a castered tailwheel's tight turn is flown on
        the inside brake, and a correction on rollout wants one pedal,
        not both (meridianatc.atc.tailwheel.brake_assist).
        """
        left_pos = max(0.0, min(1.0, left_pos))
        right_pos = max(0.0, min(1.0, right_pos))
        left = self.send(b"AXIS_LEFT_BRAKE_SET",
                         int(round(-16383.0 + left_pos * 32766.0)))
        right = self.send(b"AXIS_RIGHT_BRAKE_SET",
                          int(round(-16383.0 + right_pos * 32766.0)))
        if not (left or right):
            # No absolute brake axes: a hold can still be asked for by
            # stepping, which is better than rolling through the stop.
            for _ in range(0 if max(left_pos, right_pos) < 0.5 else 6):
                self.send(b"BRAKES", 0)
        return left or right

    def pushback(self) -> bool:
        return self.send(b"TOGGLE_PUSHBACK", 0)

    def pushback_stop(self) -> bool:
        """Let the tug go. Only a source whose push is open-ended (the
        X-Plane ground-ops commands) maps the event; a push that lets go
        on its own answers False and needs no telling."""
        return self.send(b"PUSHBACK_STOP", 0)

    # -- electrics and switches ------------------------------------------------

    def battery(self, on: bool = True) -> bool:
        """Every battery, the all-units setter first so a second bus
        is not left dead where the model has one."""
        ok = self.send(b"MASTER_BATTERY_SET", 1 if on else 0)
        return self.send(b"BATTERY1_SET", 1 if on else 0) or ok

    def avionics(self, on: bool = True) -> bool:
        return self.send(b"AVIONICS_MASTER_SET", 1 if on else 0)

    def pitot_heat(self, on: bool = True) -> bool:
        return self.send(b"PITOT_HEAT_ON" if on else b"PITOT_HEAT_OFF", 0)

    def seatbelts(self, on: bool = True) -> bool:
        return self.send(b"CABIN_SEATBELTS_ALERT_SWITCH_TOGGLE", 0)

    def beacon(self, on: bool = True) -> bool:
        return self.send(b"BEACON_LIGHTS_ON" if on else b"BEACON_LIGHTS_OFF", 0)

    def nav_lights(self, on: bool = True) -> bool:
        return self.send(b"NAV_LIGHTS_ON" if on else b"NAV_LIGHTS_OFF", 0)

    def strobes(self, on: bool = True) -> bool:
        return self.send(b"STROBES_ON" if on else b"STROBES_OFF", 0)

    def taxi_light(self, on: bool = True) -> bool:
        return self.send(b"TAXI_LIGHTS_ON" if on else b"TAXI_LIGHTS_OFF", 0)

    def landing_light(self, on: bool = True) -> bool:
        return self.send(b"LANDING_LIGHTS_ON" if on else b"LANDING_LIGHTS_OFF", 0)

    def transponder(self, code: str) -> bool:
        """Squawk through the source's own BCD packing."""
        if hasattr(self.sim, "set_squawk"):
            try:
                return bool(self.sim.set_squawk(code))
            except Exception:
                log.debug("the simulator refused squawk %s", code,
                          exc_info=True)
                return False
        return False

    def kohlsman(self, hpa: float) -> bool:
        """The altimeter setting. ``KOHLSMAN_SET``'s value is millibars
        times 16 -- 1013.25 hPa goes in as 16212 -- so a bare 1013 would
        set 63 hPa, not "close enough"."""
        return self.send(b"KOHLSMAN_SET", int(round(hpa * 16.0)))

    def starter(self, on: bool = True, engine: int = 1) -> bool:
        return self.send(f"STARTER{engine}_SET".encode(), 1 if on else 0)

    def all_starters(self, on: bool = True) -> bool:
        """``TOGGLE_ALL_STARTERS`` would flip running starters off as
        readily as it turns stopped ones on, so the explicit ``SET``
        forms go first and the toggle is only the last resort."""
        ok = self.send(b"STARTER_SET", 1 if on else 0)
        if not ok:
            ok = self.send(b"TOGGLE_ALL_STARTERS", 0)
        return ok


__all__ = ["FlightDirector", "AXIS_MAX", "AXIS_MIN"]
