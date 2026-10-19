# Low-speed turn angle baseline (2026-10-19)

Source route: `000000fb--9b2ec00d7f` (segments 0-5).
Vehicle: Tesla Model Y, C3XL, branch `dev20261018-slgame-jetlink`.
Raw trace: `2026-10-19-000000fb.csv` in this directory.

## Recorded standard

| Turn | Peak sample | Speed at peak | Driver peak wheel angle | Controller target at peak | Model curvature at peak | Camera lane confidence at peak |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| Left | t_route 211.535 s (t_turn 6.535 s) | 4.99 m/s | **114.0 deg** | 57.3 deg | -0.0274 1/m | left 0.012, right 0.019 |
| Right | t_route 223.386 s (t_turn 7.386 s) | 2.64 m/s | **-203.6 deg** | -125.5 deg | 0.0670 1/m | left 0.018, right 0.007 |

Both demonstrations reached roughly a 180-degree heading change. Camera
odometry integrated about -179 deg for the left maneuver and +166 deg for the
right maneuver. The lane model was below the 0.5 confidence gate throughout
both apexes, so lane-line completion could not be used to hold the angle.

## Why this standard matters

The driver had to add substantial steering in both turns:

- Left: driver press began at 37.4 deg; the wheel reached 114.0 deg while the
  controller target was only 57.3 deg.
- Right: driver press began near 0 deg; the wheel reached -203.6 deg while the
  controller target was -125.5 deg.

The model action head was therefore under-commanding low-speed turns even
though the vehicle was already in a turn maneuver. The control fix uses this
table as a low-speed reference only when the main camera does not have
trustworthy lane lines. The reference is scaled to the wheel angle actually
observed plus a 5% lead, so it ramps with the driver/vehicle instead of
snapping the wheel to the peak angle on turn entry. With confident lane lines,
the existing camera-based completion and centering path remains authoritative.

On 2026-10-20 the driver-signaled turn was changed to start from the lamp alone:
no wheel angle is needed to confirm the request. The reference therefore starts
at 30% of the recorded angle and lets the measured wheel angle take over. The
same change adds an 18 km/h (5.0 m/s) ceiling while the lamp is on at or below
30 km/h, including the approach from a standstill. It only lowers the existing
speed target, never requests acceleration, and releases at 1.0 m/s² after the
lamp clears or the wheel has stayed centred for 0.75 s.

Offline replay of this route through the new controller produced a 114.4 deg
left target at the left peak and a 203.6 deg right target at the right peak,
matching the recorded driver values.

On 2026-10-20 the controller test suite was run on the device environment and
passed `21/21` tests, including both recorded-angle targets, counter-steer
rejection, confident-lane handoff, and the lateral-acceleration limit.

## Acceptance

- Left demonstration: peak wheel angle at least 100 deg for a comparable
  180-degree low-speed maneuver.
- Right demonstration: peak wheel angle at least 185 deg for a comparable
  low-speed maneuver.
- No lane straddle when the camera reports a confident lane after the turn.
- Curvature must still pass the existing lateral-acceleration limit, steering
  rate limit, and Panda safety checks.

The trace columns are:

`direction, t_route, t_turn, v_ego, steering_angle_deg, steering_target_deg,
desired_curvature, model_curvature, driver_pressed, selfdrive_active,
lat_active, left_lane_prob, right_lane_prob, left_lane_y, right_lane_y`

`t_route` is seconds from the start of route `000000fb`; `t_turn` is seconds
from the start of the corresponding recording window.
