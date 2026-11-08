# Firmware patch: restore the full-lap turn-exit contract (2026-11-08)

`legacy-full-lap.patch` is the C-side companion of the simulator fix.

It reverts only the turn-exit / short-link contract in

- `src/route.c`, `include/route.h`
- `src/line_follow.c`, `include/line_follow.h`

to the `4477434` version and then re-applies the `7569a50` START-gate fix
(`single_line_count`) and the current `src/route_table.c` is left untouched.
Controller constants in `include/motion_config.h` are set back to
`CORNER_ARC_SPEED_CM_S=70`, `CORNER_ARC_EXIT_ALONG_TOL_CM=6`,
`CORNER_ARC_EXIT_TOL_CM=3`. Encoder polarity/scale, wheel geometry, sensor
geometry and every field measurement stay as they are.

Apply to a clean firmware checkout of `7569a50`:

```sh
git apply /path/to/firmware/legacy-full-lap.patch
```

Verify against the simulator:

```sh
cd /Users/zhendong/Documents/stm32car_simulator
FW_DIR=/path/to/patched/car_stm32/ npm run parity
```

A ready-to-review worktree already has the patch committed: `/Users/zhendong/Documents/car_stm32_full_lap`, branch `fix/full-lap-legacy`, commit `600b6e9`. `pio run` succeeds (RAM 10884 / Flash 51712).

Result on 2026-11-08: scheme 1/2/3/4/5/6/7 all PASS, scheme 1 = 15072
identical-input frames, final index 29 (full lap), mean target error
0.0029 cm/s.

**Known gap:** the firmware host suite still contains tests that assert the
post-2026-11-08 strict exit (`run_pose_node_authorization`,
`run_turn_angle_exit`, `run_short_link`, `run_pass_heading`,
`run_maneuver_loss`, `run_total_distance`, `run_pending_rewrite`). They fail
against this patch by design; migrating them to the restored contract is the
next firmware task. Do not merge the patch until that is done.
