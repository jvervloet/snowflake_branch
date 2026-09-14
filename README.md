# Snowflake Branch

ESP32-S3 firmware driving a 6-branch, 25-LED-per-branch WS2812 snowflake
sculpture (150 LEDs total), with a push button cycling through several
animation modes.

## Hardware

- **LEDs**: 6 independent WS2812 strips ("branches"), 25 LEDs each, wired
  center-to-tip (LED 1 closest to the snowflake's center).
  - Branches on GPIO 4, 5, 6, 7 are driven over RMT.
  - Branches on GPIO 15, 16 are driven over SPI (`SPI2_HOST`/`SPI3_HOST`),
    since the ESP32-S3's RMT peripheral only supports 4 simultaneous TX
    channels.
- **Button**: momentary push button on GPIO 18, wired to ground with the
  internal pull-up enabled (active-low). Each press advances to the next
  animation mode.

Each branch's wiring order (see `branch_led_pos[]` in `main/main.c`):

| LEDs | Segment |
|---|---|
| 1–5 | straight out from center |
| 6–9 | branch left |
| 10–13 | branch right |
| 14–16 | straight out |
| 17–19 | branch left |
| 20–22 | branch right |
| 23–25 | straight out, to the tip |

`branch_angle_deg[]` places each of the 6 branches 60 degrees apart,
counterclockwise starting from the top (confirmed against the physical
build with the channel test mode below).

## Animation modes

Cycled in order with each button press:

1. **White sparkle** (default at power-on) — all LEDs breathe white between
   10–20% brightness; random LEDs briefly sparkle brighter.
2. **Red/blue spin** — a red/blue half-and-half split spins around the
   center.
3. **Snowfall** — sparse white "drops," each 2–3 columns wide with a
   fading trail, fall from the top of an internal 40x40 image grid to the
   bottom.
4. **Radial pulse** — a few cool-white rings expand outward from the
   center to the branch tips.
5. **Green square** — a static test image: the largest square that fits
   inside the circle traced by the farthest LED, rendered on the same
   40x40 image grid as the snowfall/pulse modes.
6. **Channel test** — a wiring/calibration aid: branch (strip) *N* lights
   its first *N* LEDs, counting from the center, and nothing else. Branch 1
   shows 1 LED, branch 2 shows 2, ... branch 6 shows 6. Count the lit LEDs
   on each physical branch to work out which `strip_gpio[]` entry (and so
   which GPIO) actually drives it.
7. **Off**.

Pressing the button while a mode is running pauses that mode's state
(fade phase, active sparkles/drops/rings) rather than resetting it, so
cycling through Off and back resumes where it left off.

## LED position mapping

Every LED's `(x, y)` position (in `main/main.c`, `led_pos[]`) is computed
once at startup by rotating the single-branch template
(`branch_led_pos[]`) to each branch's angle (`branch_angle_deg[]`). Modes
that display a 2D image (snowfall, the green square) sample a shared
40x40 grid built from those positions (`build_led_positions()`); modes
that need exact position (the red/blue spin, the radial pulse) use the
coordinates directly.

The coordinates are unitless — only the relative shape matters, not any
real-world size.

[`docs/branch_map.html`](docs/branch_map.html) is a standalone diagram
(open it directly in a browser) that mirrors `branch_led_pos[]` and
`branch_angle_deg[]` exactly, showing every LED numbered on one branch and
the full 6-branch layout with the image grid overlaid. If those tables in
`main.c` are ever corrected, update the matching arrays in this file too
so the diagram doesn't drift from the firmware.

## Building and flashing

Requires [ESP-IDF](https://github.com/espressif/esp-idf) (this project
was built against v5.5).

```sh
. $IDF_PATH/export.sh
idf.py build
idf.py -p PORT flash monitor
```
