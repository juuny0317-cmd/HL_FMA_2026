# Course 07 parking overlay — geometry review

## Active CSV parking replacement

`parallel_parking_source.csv` is the recorded source for the active Course 07
parallel parking. Run `python3 operations/build_course_07_csv_parking.py` to
produce `course_07_p*_t*_f*_parallel_csv.csv` and
`course_07_parallel_csv_manifest.json`. The builder trims the yellow shared
approach to `P_entry` point 49 near the prior cyan route start, then replaces
the P1 and P2 reverse/exit branches with the source CSV paths. It moves the
four parallel FSM events and direction changes onto the new geometry. The same
build moves `HILL_STOP` exactly 1.5 m upstream along each variant and inserts
a waypoint at the new event location. The
mission bundle and Foxglove replay use these generated files. The older
`*_fair_fsm_preview.csv` files below remain the baseline for other zones.

The generated route is a software geometry/FSM artifact. Parking-bay clearance
and driven performance have not been verified in the field.

This directory is the course geometry review set. It preserves the normal
parts of `course_07`, uses the two T-parking alternatives, and applies the
reviewed reverse-parallel-parking maneuver to both parallel branches:

```text
MAIN_00
  -> choose T_1 or T_2
  -> MAIN_01
  -> choose
       P1: BYPASS_FORWARD -> PARK_REVERSE -> PARK_EXIT_FORWARD -> FINAL_COMMON
       P2: BYPASS_FORWARD -> PARK_REVERSE -> PARK_EXIT_FORWARD -> POST_EXIT
  -> choose FINAL_1 or FINAL_2
```

- `course_07_parking_overlay_master.csv` is the single branch-aware route
  definition. `branch_group`, `branch_option`, and the parallel segment suffix
  distinguish the alternatives and their motion phases.
- `course_09_indexed_points.csv` maps every recorded point to the RViz label
  `C09-000` through `C09-195` in the course-07 map frame.
- `course_09_fsm_annotations.csv` is the user-confirmed source of truth for
  mission boundaries, stop/decision intervals, branch-specific direction
  changes, and finish events. `HILL_STOP` is anchored at `C09-002` with a
  3-second hold.
- `course_07_fsm_route_events.json` maps those source annotations to route
  indices and `s_m` values in all eight flattened variants.
- `course_07_p*_t*_f*_fsm_preview.csv` adds `fsm_zone`, event, condition,
  hold-time, and direction-source columns for later FSM implementation. It is
  the input to the full-route fairing step.
- `course_07_p*_t*_f*_fair_fsm_preview.csv` is the final RViz/Foxglove
  geometry. Outside parallel parking it retains the natural-cubic route. The
  P1 uses a tangent-constrained cubic bypass and a regularized degree-7 curve
  fitted to the recorded parking path. P2 copies that complete P1 reverse/exit
  path without changing its shape. P2 also reuses an exact prefix of the P1
  first-forward path. Its reverse start is the point on that path that best
  aligns the copied parking tail with `C09-167`, the matching parking location
  on the original forward-only P2 route. Both branches finish on the same
  1.40 m straight alignment tail whose vehicle heading is perpendicular to the
  route tangent immediately before `PARALLEL_PARK_START`.
  Each branch follows the same parking path forward on exit. After returning to
  its reverse-start point, P2 uses a 4.00 m tangent-constrained transition onto
  the P1 first-forward path, follows the remaining P1 path exactly, and then
  continues through `FINAL_COMMON`. Direction cusps are split and spacing is at
  most 0.10 m.
- `course_07_p*_t*_f*_preview.csv` are the eight flattened combinations for RViz
  geometry inspection.
- `course_07_parking_smoothed_master.csv` and
  `course_07_p*_t*_f*_smoothed_preview.csv` apply a 1.50 m minimum-radius bound to
  every moving parking phase before the full-route fairing pass.
- `course_07_parking_smoothing_manifest.json` contains the measured curvature
  and deviation of every corrected phase.
- `course_07_full_route_fairing_manifest.json` records the full-route fairing
  limits and measured result for all eight variants.
- `course_07_parking_overlay_manifest.json` records source ranges, composition,
  datum, and join distances.

The preview files deliberately use `target_speed_mps=0.000` everywhere and are
not drive-ready. The hill stop and parallel forward/reverse phases are authored,
but the bypass corridor, parking-box fit, vehicle swept volume, speeds, and the
cone-based branch selector still require field validation before a vehicle run.

Parallel-parking anchors:

```text
P1: C09-106 -> bypass -> C09-143 -> reverse through C09-131
    -> 1.40 m straight aligned endpoint -> forward to C09-143
P2: course_07:704 vicinity -> exact prefix of the P1 bypass
    -> reverse start projected from the C09-167 parking reference
    -> exact translated copy of the complete P1 reverse path
    -> copied 1.40 m straight aligned endpoint -> forward along the same copy
    -> 4.00 m transition -> exact P1 first-forward suffix -> FINAL_COMMON
```

The generator fits the P1 path once as a curvature-continuous degree-7 Bezier
instead of connecting noisy recorded points directly. P2 then translates every
point of that finished P1 tail by one common vector, so curvature, length,
heading change, and the 1.40 m alignment segment remain identical. Comparison
with the original forward-only P2 route uses `C09-167` as the completed-parking
reference. Placing the copied P1 tail's aligned endpoint there puts its reverse
start within 0.25 m of the P1 first-forward path. The generator projects that
start onto the path and uses the exact path prefix before reversing, so the
final parked endpoint remains within the same tolerance of the original P2
parking location. This removes the separate P2 approach curve, the separate
quintic parking curve, and the large outer post-exit loop from earlier
revisions. The short P2 exit transition stays within 0.35 m of the P1
first-forward path. The copied parking curve and exit transition remain limited
to 0.45 1/m curvature (2.22 m minimum radius) and are checked against the
configured 0.58 m wheelbase and 27.5-degree autonomous steering limit.

Reapply the reviewed hill/parallel revision after regenerating the baseline
files:

```bash
python3 operations/rebuild_course_07_parallel_and_hill.py
```

The reviewed `fsm_zone` boundaries can nevertheless be transferred to the
measured course-07 route without copying preview geometry, zero speeds, or
branch directions. From the repository root run:

```bash
./operations/apply_course_07_mission_tags.sh
```

The transfer uses ordered XY boundary matching with a 1.0 m rejection limit.
The branch-dependent preview finish coordinate is exempt because only the
measured route's final point may become `FINISH`. `S_CURVE` becomes the runtime
`S_OBSTACLE` tag, and `T_PARK` becomes `PERP_PARK`.

Regenerate the files with:

```bash
python3 operations/build_course_07_parking_overlay.py \
  --course-07-route /path/to/course_07_route.csv \
  --course-07-datum /path/to/course_07_datum.yaml \
  --course-09-wgs84 /path/to/course_09_wgs84.csv \
  --output-directory ros2_ws/src/hl_ku_core/routes/course_07_parking_overlay \
  --maximum-spacing-m 0.30

python3 operations/smooth_course_07_parking_overlay.py \
  --course-07-route /path/to/course_07_route.csv \
  --course-07-datum /path/to/course_07_datum.yaml \
  --course-09-wgs84 /path/to/course_09_wgs84.csv \
  --output-directory ros2_ws/src/hl_ku_core/routes/course_07_parking_overlay \
  --minimum-radius-m 1.50 \
  --curve-spacing-m 0.15 \
  --maximum-spacing-m 0.30

python3 operations/map_course_09_fsm_to_routes.py \
  --route-directory ros2_ws/src/hl_ku_core/routes/course_07_parking_overlay \
  --annotations \
    ros2_ws/src/hl_ku_core/routes/course_07_parking_overlay/course_09_fsm_annotations.csv

python3 operations/fair_course_07_fsm_routes.py \
  --route-directory ros2_ws/src/hl_ku_core/routes/course_07_parking_overlay \
  --maximum-spacing-m 0.10 \
  --maximum-deviation-m 0.30 \
  --minimum-turning-radius-m 1.50 \
  --maximum-smoothing-length-m 8.0
```
