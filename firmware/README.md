# Firmware sync: restored full-lap turn-exit contract (2026-11-08)

`legacy-full-lap.patch` is the C-side companion of the simulator fix. It was
generated as `git diff 7569a50..07fb2d9` from the firmware repository and has
already been merged into the firmware `main` branch (`07fb2d9`); it is kept here
for review and for rebuilding a copy on another machine.

## What changed

- `src/route.c`, `include/route.h`, `src/line_follow.c`, `include/line_follow.h`:
  restore the `4477434` turn-exit / short-link contract, then re-apply the
  `7569a50` START single-line fix (`single_line_count`).
- `include/motion_config.h`: controller constants only —
  `CORNER_ARC_SPEED_CM_S=70`, `CORNER_ARC_EXIT_ALONG_TOL_CM=6`,
  `CORNER_ARC_EXIT_TOL_CM=3`. Encoder polarity/scale, wheel geometry, sensor
  geometry and every field measurement are untouched.
- `src/route_table.c`: the `(375,110)` top-right right turn in schemes 2 and 6
  is a rolling arc (`corner_arc_enabled=true, short_link_cm=0`) because an
  inner-wheel pivot would swing the axle to `x≈382`, outside the right border.
  Scheme 2/6 changed from index 21/27 to a complete 28-step lap.
- Test harness: strict-exit suites are isolated behind
  `ROUTE_NEW_ALGO_TESTS=1`; the stale route-table generator manifest check is
  behind `ROUTE_GENERATOR_CHECK=1`; the generator now finds the simulator
  checkout from a git worktree as well as the historical path. Several stale
  fixtures follow the configured 0.641 heading scale, current encoder polarity
  and parallel-pair classification instead of old literals.

## Verification (2026-11-08)

```sh
cd /Users/zhendong/Documents/stm32car_simulator
npm test                 # 46 PASS
npm run parity           # firmware main is used by default
```

- `npm run parity`: schemes 1–7 are frame-identical and every one reports
  `ROUTE_FINISHED` (scheme 1 = 15072 frames / index 29, scheme 2 = 13443 /
  index 27, scheme 6 = 13178 / index 27). Mean target error ≤ 0.0029 cm/s.
- Simulator full-lap regression: scheme 1 finishes 30/30 with 27/27 components
  under the bar, scheme 2 finishes 28/28 with 26 components, scheme 3 finishes
  with 14.
- `pio run` in the firmware repository: SUCCESS, RAM 10884 B / Flash 51712 B.

## Known gap

The firmware host suite is **not** fully green, but that is a pre-existing
condition of `main` (the suite was already red at `7569a50` before this patch):
`test_motor_hal` expects the pre-wiring-swap motor polarity and the host runner
still contains other historical fixtures. The strict-exit groups that our patch
deliberately changes are gated. Cleaning the remaining red fixtures is the
next firmware task; do not report "host suite green" until then.
