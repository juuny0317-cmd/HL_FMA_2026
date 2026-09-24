# LiDAR parking-route selection

Automatic parking selection is enabled for the two Course 07 TUI variants named
`전체 코스 · 주차 자동선택 · F1/F2`.  Both start with the P1/T1 geometry and
load the other P/T alternatives from `course_07_mission.csv` using the same
rigid placement as the active test route.

For campus tests, TUI `7` defaults to the standalone `T-AUTO` route and TUI `9`
defaults to `P-AUTO`.  Pressing `R` places only that parking course at the current
RTK position and heading without scaling it.

The selector is inactive in every mission except:

- `PERP_PARK`: observe the T1/T2 reverse parking areas.  When both results are
  accumulated throughout the common approach, then commit 0.75 m before
  `T1_FORWARD_TO_REVERSE`.
- `PARALLEL_PARK`: accumulate P1/P2 observations and commit 0.75 m before the
  P2 forward-to-reverse location projected on the common P1 approach.

For each area the selector retains `UNKNOWN`, `CLEAR`, or `BLOCKED`.  Map-frame
LiDAR clusters are assigned to the closer parking path, so a divider cone is not
counted against both paths.  A blocked result requires five consecutive hit
scans and is then retained through the mission.  A clear result requires
repeated scans with enough of the parking area inside the verified LiDAR field
of view.

Parking-space selection only accepts clusters within 5 m of the LiDAR and in
its configured forward field of view.  This is a selector-local filter: the raw
scan, 12 m map obstacle pipeline, and S-course obstacle avoidance range remain
unchanged.

The route table is:

| Area 1 | Area 2 | Selected route |
|---|---|---|
| clear | any | 1 |
| blocked | clear or unknown | 2 |
| unknown | clear | 2 |
| unknown | blocked or unknown | 1 |
| blocked | blocked | 1 |

The selection itself never requests a brake.  Once selected, the route remains
latched until that parking mission ends.  Authored forward/reverse transition
holds still run as part of the parking path.

RViz subscribes to `/planning/reference_path`, so the yellow path is the route
that the path tracker is currently following and changes when P1/P2 or T1/T2 is
latched.

Live status is available at:

```bash
ros2 topic echo /mission/parking_selection
```

The message contains the active mission, both area states, the latched P/T
choice, and the selected `p*_t*_f*` variant.  Route switching is accepted by the
path tracker only while `/mission/status.state_name` is the matching parking
state.  Other mission commands and paths use the existing code path unchanged.
