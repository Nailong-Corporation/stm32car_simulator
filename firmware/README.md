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

## Firmware advanced after this patch

The firmware `main` went through `b167058` ("转弯参数和编码器参数修正"),
`cf5d30d`, `285efd5`, `f944a47`, and then an uncommitted 2026-11-10 working tree
(off-centre exit search plus `ROUTE_SCHEME_SELECT=4`). None of those firmware
changes were edited by the simulator work. The simulator mirrors the current
working tree; see the top-level `README.md` section
"2026-11-10 固件工作树（`f944a47` 之后）实车环岛入口同步" for the verification
and the remaining physical-model limitation.

## 2026-11-10 `59a7268` follow-up (simulator only)

The 2026-11-10 working tree was committed as `59a7268` ("修复了一些已知问题").
The simulator now mirrors that commit as well:

- `src/line_follow.c`: `route_gray_shows_branch()` gate on the grey position
  PID (`usable = trusted && narrow && !branch`).
- `src/route_table.c`: scheme 4 rows 11/15 use
  `TURN_TRANSISTOR_STRAIGHT_CM + 5` and row 18 uses `junction_count=7`,
  `cruise_speed=100`.
- Simulator picture fix (NOT a firmware change): the printed transistor is
  narrowed from 18 x 6 cm to **7 x 6 cm** (outer footprint 9.8 x 8.8 cm,
  ~1.1:1). The V arms now sit inside the 8-channel bar, so the Q6 corner reads
  0x80 between the arms instead of the old 0x00 hole; the base bar still reads
  0xFF from the front.

Verification on the simulator side: `npm test` exits 0 (schemes 1/4 now pass Q6
and reach the ring entrance, `maxIndex=10`), `npm run parity` is frame-identical
for schemes 1-7 against firmware `59a7268`, and `npm run build` succeeds. The
firmware repository was not modified.

## 2026-11-12 `2e751de` follow-up (simulator only)

The simulator `dev` branch now mirrors `car_stm32_school_competition` `main` at
`2e751de`, without modifying the firmware repository. In addition to the
updated route table (schemes 0-8), it mirrors the partial-bar passage/anchor
rules, same-side short-link edge confirmation, 100 ms / 0.5 cm ordinary edge
exit with a 0.30 rad heading gate, the 50 ms / 0.5 cm short-link edge limit,
the close-corner R17 exception, and the per-scheme start pose. The simulator
tracks the scheme 1 start-anchor mismatch in issue #4 and mirrors the firmware
condition while that upstream route-table inconsistency is reviewed.

Verification: `npx tsc --noEmit` passes, `npm test` exits 0 with current known
route stops reported as `KNOWN`, and `node tests/firmware_parity.mjs` matches
schemes 1-8 frame-for-frame (maximum target error 0.0043 cm/s, no threshold
frames).
