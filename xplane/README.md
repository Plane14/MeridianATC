# MeridianATC-XP (X-Plane 12)

A native, main-thread XPMP2 renderer for MeridianATC's invented traffic. This
plugin does not generate traffic. The Python traffic sender is a separate next
step; the existing XPlaneSource reads the user's aircraft independently.

## Build

Clone with `git clone --recurse-submodules`, or run `git submodule update --init
--recursive` in an existing checkout. XPMP2 is pinned to release **v3.7.0**
(`fb543f9d063e7fc61e08f9d21c6e68136606fc23`). SDK **4.3.0** headers and libraries
are vendored in `plugin/lib/SDK`. nlohmann/json **3.12.0** and its MIT licence
are in `plugin/lib`. XPMP2 sound/FMOD is disabled (no proprietary SDK required).
XPLM feature macros are enabled through XPLM430 for both library and plugin.

Requires CMake 3.20+, C++17, and a 64-bit compiler. On Windows use an **x64 Native
Tools Command Prompt for VS 2022** (MSVC Build Tools and Windows SDK installed):

```console
cmake -S xplane/plugin -B xplane/plugin/build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build xplane/plugin/build --parallel 2
ctest --test-dir xplane/plugin/build --output-on-failure
```

On Ubuntu install `build-essential cmake libgl-dev libglu1-mesa-dev`, then run
the same commands (omit `-G Ninja` if Ninja isn't installed). On macOS install
Xcode command-line tools and CMake; add
`-DCMAKE_OSX_ARCHITECTURES="x86_64;arm64"` for a universal build. The vendored
Mac framework aliases are expanded to real files/directories so Windows ZIP
extraction cannot leave broken symlinks. Mac builds/signing are not covered by
CI. Override SDK location with `-DXPlaneSDK_DIR=/absolute/path/to/SDK` if needed.

CI builds Release on Windows/MSVC and Ubuntu/GCC, runs native protocol/motion/
socket tests, and uploads platform-specific **MeridianATC** folders. Test only:
`cmake -S xplane/plugin -B xplane/plugin/build-tests -DMERIDIAN_BUILD_PLUGIN=OFF`
then build and run CTest in that directory.

## Install / CSL models

Copy the generated **whole** `xplane/plugin/build/MeridianATC` directory into
`X-Plane 12/Resources/plugins/`. Keep the layout:

```text
MeridianATC/
  win_x64/MeridianATC.xpl       # Windows build
  lin_x64/MeridianATC.xpl       # Linux build
  mac_x64/MeridianATC.xpl       # macOS build
  Resources/                  # complete XPMP2 Resources, including Contrail/
    CSL/README
  Licenses/
  meridianatc-xp.ini           # optional, created by you
```

Each build supplies its own platform binary; folders can be merged for a fat
plugin. Resources include Doc8643.txt, related.txt, relOp.txt, Obj8DataRefs.txt,
MapIcons.png and the Contrail directory. No CSL aircraft models are bundled.
Install an XPMP2-compatible package such as **Bluebell**, preserving its
`xsb_aircraft.txt`, OBJ and texture hierarchy, under `Resources/CSL`. Respect
the model authors' licences; model packages are not automatically downloaded.

CSL roots are loaded in order: this plugin's `Resources/CSL`, then X-Plane's
`Resources/plugins/LiveTraffic/Resources/CSL`, then
`Resources/plugins/xPilot/Resources/CSL`, then configured paths in INI order.
Missing roots are skipped and canonical duplicate paths are loaded only once.
XPMP2 performs matching and fallback; an aircraft with a usable fallback is
not considered unmatched. With no usable CSL, traffic is still tracked and
reported in `unmatched`, but cannot be drawn. **Reload CSL** discovers newly
installed models and re-creates traffic; XPMP2 doesn't unload existing packages,
so replacing/removing models requires disabling/re-enabling the plugin.

## Configuration / controls

Optional `MeridianATC/meridianatc-xp.ini` (read on enable; restart/re-enable after
editing; repeated `csl=` lines allowed; relative CSL paths are plugin-relative):

```ini
port=49010
labels=1
tcas=1
csl=D:/FlightModels/Bluebell
csl=Resources/ExtraCSL
```

Only `0` and `1` are valid booleans. Port is 1..65535. Missing values use the
above defaults. Invalid values are logged and ignored. `#`/`;` whole-line
comments and section headers are ignored. Paths containing spaces need no quotes.
The UDP address is always **127.0.0.1**, not configurable to remote interfaces.
Binding fails safely if the port is already occupied.

**Plugins > MeridianATC** offers show traffic, labels, TCAS control, reload CSL,
and log status. Menu toggles last until disable/re-enable (not saved to INI).
Hiding traffic keeps snapshots but removes aircraft from rendering/TCAS. TCAS
control is requested via XPMPMultiplayerEnable and may be refused when another
plugin owns AI targets; see Log.txt. Drawing doesn't require TCAS control.

Read-only integer datarefs:

* `meridianatc/xp/connected`: 1 while an accepted engine packet was seen within
  10 seconds and the plugin is enabled; otherwise 0.
* `meridianatc/xp/aircraft`: number of IDs in the current snapshot, including
  unmatched/hidden aircraft; 0 after clear/timeout/disable.

Disable/stop destroys aircraft, the flight loop, socket, and XPMP2 state.

## UDP protocol v1

UTF-8 JSON, **one object per UDP datagram**, at most **65507 bytes**. Every
message must have integer `v:1` and string `t`. Send from a persistent localhost
UDP socket and receive status on that same source socket. No framing, ACK,
authentication or retransmission is used. Unknown additional fields are ignored.
Malformed/unsupported packets are silently dropped without changing the snapshot.
All aircraft in a traffic datagram are validated before applying it; one bad
aircraft rejects the whole datagram. Limit: 512 aircraft and 12 JSON nesting
levels. Up to 64 datagrams are handled per flight loop to bound main-thread work.

### Engine -> plugin

Full replacement snapshot (empty `ac:[]` is valid and removes all traffic):

```json
{"v":1,"t":"traffic","seq":1,"ac":[{"id":"DLH4AB","type":"A320","op":"DLH","livery":"","reg":"D-AIAB","lat":48.1,"lon":11.7,"alt_ft":12000,"hdg":359,"pitch":2,"roll":0,"gs_kt":250,"vs_fpm":600,"gnd":false,"gear":0.0,"flaps":0.0,"lights":{"taxi":0,"land":0,"bcn":1,"strb":1,"nav":1},"squawk":"2000"}]}
```

* `seq`: nonnegative unsigned 64-bit integer, strictly increasing per session.
  Duplicate/out-of-order snapshots are ignored and do not refresh timeouts.
* `id`: stable callsign/identity, nonempty, max 64 bytes, unique in snapshot.
* `type`: nonempty ICAO model designator, max 4 bytes.
* `op`: airline ICAO max 3 bytes; `livery`: XPMP2 livery selector max 128 bytes;
  `reg`: registration max 32 bytes. These three fields may be omitted (empty).
* `lat`/`lon`: WGS84 degrees, [-90,90]/[-180,180]. `alt_ft`: feet **MSL**,
  [-2000,100000]. `hdg`: true degrees [-360,360], normalized modulo 360.
* `pitch`: degrees [-90,90]; `roll`: degrees [-180,180].
* `gs_kt`: ground speed knots [0,2000]; `vs_fpm`: feet/min [-30000,30000].
* `gnd`: JSON boolean; ground positions are terrain-probed and clamped to terrain.
* `gear`/`flaps`: ratios [0,1]. `lights` requires all five keys shown, each a
  boolean or numeric 0/1. `squawk`: exactly four octal digits as a **string**.
* All fields except `op`, `livery`, `reg` are required. Numbers must be finite.
  Text fields reject control characters. XPMP2 info-text fields may truncate
  registration/callsign to their SDK-sized buffers; internal IDs remain intact.

Snapshot IDs absent from the next accepted snapshot are deleted immediately.
All aircraft are deleted **10 seconds after the last accepted traffic snapshot**
(hello packets do not extend aircraft lifetime). Timeout/clear resets sequence
tracking so a restarted sender can begin at zero. If restarting earlier, send
clear from the same socket or wait 10 seconds. Only one source address/port owns
the session; a new source can take over after 10 seconds without accepted owner
packets. Other sources' hello requests still get replies but do not take over.

```json
{"v":1,"t":"clear"}
```

Immediately removes all aircraft and resets sequence tracking.

```json
{"v":1,"t":"hello"}
```

Requests immediate status. Send hello/traffic periodically to maintain connected
status. No sender timestamp field is required: monotonic local arrival times
drive a 200 ms interpolation buffer. Heading/longitude interpolate on the short
arc (including north/dateline wrap). Beyond the newest snapshot, GS/heading/VS
extrapolation is capped at 2 seconds; aircraft then freeze until update/timeout.
Ground VS is ignored. Thrust is estimated from ground speed, not an engine-power
measurement. A steady snapshot rate of 5–10 Hz is recommended.

### Plugin -> engine

Sent to every hello's source immediately, plus once a second to the active
engine while an accepted packet has arrived within the last 10 seconds:

```json
{"v":1,"t":"status","plugin":"MeridianATC-XP","version":"0.1.0","xpmp2":"3.7.0","xplane":120000,"csl_models":123,"aircraft":1,"drawn":["DLH4AB"],"unmatched":[]}
```

`xplane` is XPLMGetVersions' integer simulator version (example only).
`csl_models` is XPMP2's installed-model count. `aircraft` counts snapshot IDs.
`drawn` contains currently visible/render-enabled XPMP2 IDs (not a guarantee
of pixels on screen: frustum/distance/loading can prevent visible rendering).
`unmatched` contains IDs whose XPMP2 aircraft could not be constructed (normally
no usable CSL). Hidden traffic isn't unmatched. Model loading is asynchronous;
initial status may precede the first draw. Status also goes to Log.txt when
requested through the menu.

## Limitations / verification

No Python traffic sender, model redistribution, remote networking, audio, or
automatic AI-control arbitration is included. Simulator rendering, ground probes,
labels and TCAS require live X-Plane validation; protocol/motion/socket unit
tests do not prove these visuals. macOS binaries need normal local signing/
quarantine handling. Windows requires the MSVC runtime used by the build.
