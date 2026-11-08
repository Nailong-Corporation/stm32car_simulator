/**
 * STM32 Smart Car Simulator - Parameters & Configuration
 * Exact parity with motion_config.h, control_config.h, board_config.h
 */

export const CONTROL_PERIOD_MS = 5;
export const CONTROL_PERIOD_S = 0.005;
export const CONTROL_MAX_INTERVAL_MS = 15;

export const SENSOR_FILTER_SAMPLES = 3;
export const SENSOR_MAX_REGION_WIDTH = 6;
export const SENSOR_CONTINUITY_DISTANCE = 3.0;
export const SENSOR_CONTINUITY_MARGIN = 1.0;
export const LINE_CONFIRM_SAMPLES = 3;

/* Wheel & Vehicle Dimensions */
export const LEFT_WHEEL_DIAMETER_CM = 6.5;
export const RIGHT_WHEEL_DIAMETER_CM = 6.5;
export const WHEEL_TRACK_CM = 14.0;
export const GRAY_FORWARD_OFFSET_CM = 24.0;

export const CORNER_RADIUS_CM = 24.0;
export const CORNER_HEADING_KP = 8.0;

/* Rolling corner (2026-11-08 prototype): instead of braking to the settle speed
 * and pivoting around the inner wheel, an ordinary 90 degree corner is a single
 * arc of radius CORNER_ARC_RADIUS_CM that is tangent to both segments.  The arc
 * leaves the printed wire by R*(sqrt(2)-1) at its midpoint (6.2 cm at R=15) and
 * rejoins it exactly at the exit tangent point, so the pose carries the car
 * through the corner and the grey bar recaptures the next leg on the way out.
 * The axle speed is the arc speed; the wheels are
 *   outer = v*(1 + track/(2R)),  inner = v*(1 - track/(2R)),
 * i.e. the inner wheel keeps rolling instead of being stopped. */
/* Values below are the merged defaults the prototype settled on; the sweep
 * hooks it used during tuning (ARC_R/ARC_V/...) are intentionally removed so
 * the simulator and the firmware build the same constants. */
export const CORNER_ARC_RADIUS_CM = 13.0;
/* Axle centre speed through the arc (cm/s). The outer wheel runs at
 * v*(1 + track/(2*R)) = 1.47v at R=15, track=14. */
export const CORNER_ARC_SPEED_CM_S = 70.0;
/* Differential (cm/s) per rad of heading error against the arc tangent. */
export const CORNER_ARC_HEADING_KP = 22.0;
/* The outgoing leg must leave room for the exit tangent point (R past the
 * corner) before the next corner's bar can see it: at least 2R of leg. */
export const CORNER_ARC_MIN_LEG_MARGIN_CM = 4.0;
/* Enter the arc when the axle is this close to the entry tangent point. */
export const CORNER_ARC_ENTER_TOL_CM = 0.5;
/* The wheels need CORNER_ARC_EXIT_LEAD_S to unwind after the differential is
 * released (measured: the yaw keeps integrating for ~60 ms after the target
 * differential drops to zero).  The arc therefore hands off CORNER_ARC_EXIT_LEAD_S
 * earlier: the coasting car finishes the last few degrees on the same circle. */
export const CORNER_ARC_EXIT_LEAD_S = 0.05;
/* Pose exit: heading within this residual of the corner angle AND the axle
 * within this distance of the exit tangent point. */
export const CORNER_ARC_EXIT_ALONG_TOL_CM = 6.0;
export const CORNER_ARC_EXIT_ANGLE_RAD = 0.05;
export const CORNER_ARC_EXIT_TOL_CM = 3.0;
/* Angular progress (rad) that also counts as reaching the exit tangent. */
export const CORNER_ARC_EXIT_PROGRESS_RAD = 0.06;
/* Yaw-rate damping applied while a rolling corner's differential unwinds
 * (cm/s of differential per rad/s of measured yaw rate). */
export const CORNER_ARC_EXIT_DAMPING = 2.5;
/* Slew on the differential so the inner wheel is not stepped from full speed
 * to the arc inner speed in one period (motion_config.h MOTOR_MAX_DECEL_S). */
export const CORNER_ARC_DIFF_SLEW_CM_S2 = 900.0;
export const LEFT_COUNTS_PER_WHEEL_REV = 780.0;
export const RIGHT_COUNTS_PER_WHEEL_REV = 780.0;
/* MG310 1:10 gearbox (2026-11-07): the encoder sits on the motor shaft, so a
 * wheel revolution now yields half the quadrature counts (1560 -> 780); the
 * distance scale halves with it so distance-per-count is unchanged. */
export const ENCODER_LEFT_SIGN = -1;
export const ENCODER_RIGHT_SIGN = 1;
export const ODOMETRY_DISTANCE_SCALE = 0.95;
export const ODOMETRY_HEADING_SCALE = 0.641;

export const VEHICLE_MAX_SPEED_CM_S = 500.0;
export const ODOMETRY_MAX_SPEED_CM_S = 650.0;
export const SPEED_FILTER_TAU_S = 0.030;
export const WHEEL_TARGET_MAX_CM_S = 500.0;

/* Speed PID Inner Loop */
export const SPEED_KP = 1.2;
export const SPEED_KI = 8.0;
export const SPEED_KD = 0.0;
export const SPEED_INTEGRAL_LIMIT = 10.0;
export const SPEED_FEEDFORWARD = 255.0 / VEHICLE_MAX_SPEED_CM_S;

export const MOTOR_PWM_LIMIT = 255.0;
// Simulator plant assumptions, separate from firmware tuning. Bench calibration pending.
export const MOTOR_RESPONSE_TAU_S = 0.120;
export const MOTOR_COAST_TAU_S = 0.350;
export const MOTOR_BRAKE_TAU_S = 0.080;
export const MOTOR_MAX_ACCEL_CM_S2 = 500.0;
export const MOTOR_MAX_DECEL_CM_S2 = 500.0;
/* Reverse-torque plant assumption (simulator only, no firmware counterpart):
 * driving the H-bridge against the direction of travel adds torque to the
 * rolling friction, so the achievable deceleration is higher than coasting but
 * still grip/current limited.  The model never lets a negative command spin the
 * wheel backwards: speed is floored at zero.  Bench calibration pending. */
export const MOTOR_REVERSE_BRAKE_DECEL_CM_S2 = 900.0;

/* Stall & Reverse Protection */
export const SPEED_STALL_TARGET_CM_S = 3.0;
export const SPEED_STALL_MEASURED_CM_S = 0.5;
export const SPEED_STALL_TIMEOUT_S = 0.350;
export const SPEED_REVERSE_THRESHOLD_CM_S = 0.5;
export const SPEED_REVERSE_CONFIRM_S = 0.050;
export const SPEED_REVERSE_CONFIRM_CM = 0.10;
export const SPEED_REVERSE_IMMEDIATE_CM_S = 5.0;

/* Line Position Outer Loop */
export const POSITION_KP_CM_S = 1.8;
export const POSITION_KI_CM_S = 0.0;
export const POSITION_KD_CM_S = 0.025;
export const POSITION_CURVE_SLOWDOWN = 0.0;
export const PID_INTEGRAL_LIMIT = 20.0;
export const PID_DERIVATIVE_TAU_S = 0.020;

export const STARTUP_RAMP_TIME_S = 1.0;
export const TARGET_RISE_CM_S2 = 160.0;
export const TURN_EXIT_ACCEL_CM_S2 = 400.0;
export const LINE_FOLLOW_SPEED_CM_S = 50.0;
export const HOLD_SPEED_CM_S = 40.0;
export const AMBIGUOUS_SPEED_CM_S = 80.0;
export const HEADING_KP_CM_S_PER_RAD = 12.0;
export const INTERFERENCE_TIMEOUT_MS = 400;
export const INTERFERENCE_MAX_DISTANCE_CM = 8.0;
// Printed component symbols (hollow resistors, transistor V, battery plates)
// may be crossed with a bounded straight window; a real line loss/stall keeps
// the 400 ms / 8 cm cut-off.
export const COMPONENT_CROSS_MAX_CM = 20.0;
export const COMPONENT_CROSS_MS = 1000;

// Pose-mode bounded blind follow: after a pivot the axle can leave the line
// 3-7 cm off the segment while the 24 cm look-ahead bar cannot see it, so the
// dead-reckoned pose carries the car through the gap for this long/distance
// before the plain line-loss stop applies. Speed-scaled from the 1000 ms /
// 20 cm cut-off once cruise is raised (see 2026-11-07-speed-sweep).
export const POSE_BOUNDED_LOSS_MS = 2000;
export const POSE_BOUNDED_LOSS_CM = 30.0;

/* Junction Evidence & Confirmation */
export const JUNCTION_CONFIRM_SAMPLES = 3;
export const JUNCTION_CONFIRM_DISTANCE_CM = 0.10;
export const JUNCTION_EARLY_CONFIRM_SAMPLES = 2;
export const JUNCTION_EARLY_CONFIRM_CM = 0.05;
export const JUNCTION_EDGE_CONFIRM_SAMPLES = 4;
export const JUNCTION_EDGE_CONFIRM_CM = 0.15;
export const JUNCTION_EDGE_MAX_WIDTH = 3;
export const JUNCTION_SMOOTH_POSITION_STEP = 2.0;
export const JUNCTION_CONFIRM_TIMEOUT_MS = 100;
export const JUNCTION_CENTER_MEMORY_MS = 250;
export const JUNCTION_CENTER_MEMORY_CM = 1.5;
export const JUNCTION_ACTION_TIMEOUT_MS = 3000;
export const JUNCTION_STRAIGHT_TIMEOUT_MS = 4500;
export const JUNCTION_CLEAR_TIMEOUT_MS = 1500;
export const JUNCTION_TURN_MIN_RAD = 1.35;
export const JUNCTION_EXIT_CONFIRM_MS = 50;
export const JUNCTION_EXIT_CONFIRM_CM = 0.25;
export const JUNCTION_EXIT_ANGLE_MARGIN_RATIO = 0.05;
export const JUNCTION_PASS_SPEED_CM_S = 60.0;
export const JUNCTION_TURN_MAX_RAD = 2.1;
export const JUNCTION_WRONG_WAY_RAD = 0.35;
export const JUNCTION_CENTER_LIMIT = 2.0;
export const JUNCTION_CAPTURE_MIN_RAD = 0.70;
export const JUNCTION_CLEAR_MAX_CM = 20.0;
export const STEERING_FEEDBACK_MIN_CM_S = 5.0;
export const STEERING_FEEDBACK_MIN_HALF_CM_S = 0.1;
export const JUNCTION_CAPTURE_MIN_TURN_CM_S = 0.8;
export const JUNCTION_CAPTURE_CENTER_BIAS_CM_S = 1.2;
export const JUNCTION_CAPTURE_HEADING_KP = 8.0;
export const JUNCTION_TURN_TARGET_RAD = 1.570796327;
export const JUNCTION_SETTLE_BAND_RAD = 0.60;
export const JUNCTION_PASS_ALIGNED_RAD = 0.10;
export const JUNCTION_PASS_SETTLED_YAW_RAD_S = 0.15;
export const JUNCTION_SETTLE_SPEED_CM_S = 28.0;
export const JUNCTION_SETTLE_HEADING_KP = 24.0;
export const JUNCTION_YAW_DAMPING = 2.5;
export const JUNCTION_YAW_FILTER_TAU_S = 0.030;
export const JUNCTION_EXIT_SPEED_CM_S = 20.0;
export const COMPONENT_RECOVERY_SPEED_CM_S = 16.0;
export const COMPONENT_RECOVERY_MAX_CM = 12.0;
export const COMPONENT_RECOVERY_TIMEOUT_MS = 1200;

export const BRAKING_ACCEL_FILTER_TAU_S = 0.080;
export const BRAKING_CALIBRATION_CONFIRMED = 1;
export const BRAKING_SAFE_DECEL_CM_S2 = 250.0;
export const BRAKING_DISTANCE_MARGIN_CM = 2.0;
export const BRAKING_TARGET_JERK_CM_S3 = 800.0;

/* Reverse (negative-PWM) approach braking.
 * Mirrors include/motion_config.h: the switch is a compile-time constant in the
 * firmware and a module constant here, so the default OFF state is bit-identical
 * to the coast/short-brake plant.  A negative PWM command never becomes a
 * reverse speed target: it is torque against positive motion and the plant model
 * floors the wheel speed at zero (no reverse rotation). */
export const MOTOR_REVERSE_BRAKE_ENABLED = 0;
export const BRAKING_REVERSE_HARDWARE_CONFIRMED = 0;
/* Cumulative reverse-torque time allowed per braking segment (ms). */
export const BRAKING_REVERSE_MAX_MS = 250;
/* Below this speed the reverse output is cut: it cannot stop the car, only
 * start pushing it backwards. */
export const REVERSE_BRAKE_MIN_CRUISE_CM_S = 200;
export const BRAKING_REVERSE_MIN_SPEED_CM_S = 8.0;
/* Distance/reaction reserve before the planned stop point. */
export const BRAKING_REVERSE_GUARD_CM = 3.0;
/* Only apply reverse torque when the measured speed is this far above target. */
export const REVERSE_BRAKE_MARGIN_CM_S = 3.0;
/* Documented in the firmware but not used by the current C PID branch (the
 * shared speed PID gains do the mapping). Kept for parity. */
export const REVERSE_BRAKE_KP = 4.0;
/* Magnitude limit of the negative PWM command on the 255 scale. */
export const REVERSE_BRAKE_MAX_PWM = 60.0;
export const JUNCTION_SEARCH_TIMEOUT_MS = 2000;
export const JUNCTION_SEARCH_MAX_CM = 12.0;
export const JUNCTION_ENTRY_BRAKE_TIMEOUT_MS = 2000;
export const JUNCTION_ENTRY_BRAKE_MAX_CM = 24.0;
export const JUNCTION_EXIT_ALIGNMENT_RAD = 0.15;
export const JUNCTION_APPROACH_TIMEOUT_MS = 3000;
// Axle travel between confirming a ±90° node and starting the pivot.
// The node is confirmed when the 8-channel bar first reaches it (sensor centre
// 24 cm ahead, bar half-span ~3.85 cm) and the car then brakes, so this value
// trades an early pivot (exit line just outside the bar) against overrunning a
// dead-end corner. Calibrated against the reference routes below.
export const TURN_PIVOT_APPROACH_CM = 17.0;
// After a turn has confirmed its exit and then loses the line, bias the hold
// heading toward the inside of the corner: the pivot can start a few cm early
// (the bar sees the branch with its outer channel first), leaving the exit line
// just outside the bar.
export const JUNCTION_EXIT_REWIND_RAD = 0.17;



export const JUNCTION_COUNT_CLEAR_MS = 50;
export const JUNCTION_COUNT_CLEAR_CM = 2.0;
export const JUNCTION_STRAIGHT_EXIT_MAX_WIDTH = 3;
export const JUNCTION_ADJACENT_EXIT_MS = 20;
export const JUNCTION_ADJACENT_EXIT_CM = 0.30;

/* Preset-route pose (dead-reckoning) navigation (2026-11-06).
 * The route table stores geometry lengths reduced by the 5 cm distance margin;
 * the absolute corner waypoint is recovered from the start pose and the
 * cumulative (length, angle) chain.  The 8-channel bar still confirms every
 * corner it can see; the pose check runs in parallel so a printed symbol that
 * hides the wire (Q22 transistor) or a leg shorter than the 24 cm look-ahead
 * cannot stall the run.  Clockwise positive heading in the track frame:
 * 0 = +x east, +PI/2 = +y south. */
export const ROUTE_START_X_CM = 0.0;
export const ROUTE_START_Y_CM = 110.0;
export const ROUTE_START_HEADING_RAD = -Math.PI / 2;
export const ROUTE_WAYPOINT_MARGIN_CM = 5.0;
export const ROUTE_WAYPOINT_CONFIRM_CM = 2.5;
export const ROUTE_WAYPOINT_FALLBACK_CM = 18.0;
/* The ring entry is timed on the axle pose, not the 24 cm look-ahead bar. */
export const ROUTE_WAYPOINT_RING_CONFIRM_CM = 6.0;
/* A gray junction only authorises this row's action when its bar is this close
 * to the row's known corner; farther nodes are passing junctions. */
export const ROUTE_WAYPOINT_GATE_CM = 12.0;
/* Pose-mode turn hand-off: when the axle has reached the pivot angle within
 * this distance of the row's corner, the grey exit contract is bypassed. */
export const ROUTE_WAYPOINT_EXIT_CM = 14.0;
/* Residual turn angle accepted when the pose, not the grey line, ends a pivot. */
export const ROUTE_WAYPOINT_EXIT_ANGLE_RAD = 0.12;
/* Start braking for the corner this far before the bar reaches its waypoint. */
export const ROUTE_WAYPOINT_SLOW_CM = 30.0;
/* Allowed disagreement between the grey position (weight units, ~0.55 cm each)
 * and the pose-predicted lateral offset before the reading is distrusted. */
export const JUNCTION_POSE_GRAY_SPREAD = 4.0;
/* cm/s of differential correction per cm of pose cross-track error. */
export const JUNCTION_CROSS_KP = 2.0;
/* Pose path-following on the known segment: desired heading = segment heading
 * + atan2(-cross_track, lookahead). The grey line is only a trim when it is
 * plausible, so a printed symbol/branch cannot drag the car off the segment. */
export const POSE_PURSUIT_LOOKAHEAD_CM = 30.0;
export const POSE_GRAY_TRIM = 0.25;
/* Pose-mode clearance: two cm is enough to keep the order; no line dedup. */
export const ROUTE_CLEAR_POSE_CM = 2.0;

/* Hexagon Roundabout Bounds */
export const HEXAGON_MAX_ANGLE_DEG = 720.0;
export const HEXAGON_SPEED_CM_S = 24.0;
export const HEXAGON_ENTRY_MIN_RAD = 0.35;
export const HEXAGON_ENTRY_TIMEOUT_MS = 3000;
export const HEXAGON_ENTRY_MAX_CM = 30.0;
export const HEXAGON_ACTION_TIMEOUT_MS = 30000;
export const HEXAGON_ACTION_MAX_CM = 400.0;
export const HEXAGON_EXIT_TIMEOUT_MS = 3000;
export const HEXAGON_EXIT_MAX_CM = 30.0;
export const HEXAGON_OVERANGLE_RAD = 2.0943951;
export const HEXAGON_EXIT_CONFIRM_MS = 100;
export const HEXAGON_EXIT_CONFIRM_CM = 0.5;
export const HEXAGON_ENTRY_LOST_MS = 1200;
export const HEXAGON_ENTRY_LOST_CM = 12.0;

/* Hexagon ring geometry: regular hexagon, side 35 cm, right vertex (345,165).
 * POSE-mode traversal: turn ENTRY (30 deg) at V0, then turn 60 deg at every
 * ring vertex, then EXIT (30 deg) at V0.  Turn direction comes from the sign
 * of the table's HEXAGON angle.  The axle follows a rounded-corner polygon:
 * a turn starts R*tan(theta/2) before its vertex. */
export const HEXAGON_CENTER_X_CM = 310.0;
export const HEXAGON_CENTER_Y_CM = 165.0;
export const HEXAGON_SIDE_CM = 35.0;
export const HEXAGON_TURN_RAD = Math.PI / 3.0;
export const HEXAGON_ENTRY_TURN_RAD = Math.PI / 6.0;
export const HEXAGON_TURN_TOL_RAD = 0.06;
export const HEXAGON_LEG_START_MARGIN_CM = 1.5;
export const HEXAGON_EXIT_LINE_SAMPLES = LINE_CONFIRM_SAMPLES;
/* Straight run from the bar's V0 detection (axle ~24 cm short) to the pivot. */
export const HEXAGON_ENTRY_APPROACH_MAX_CM = 35.0;
/* A ring leg is blind for most of its 35 cm; only a real off-path excursion
 * (instead of the grey loss timer) aborts it. */
export const HEXAGON_LEG_CROSS_MAX_CM = 10.0;

export const ROUTE_SHORT_SPEED_CM_S = 60.0;
export const ROUTE_LONG_SEGMENT_CM = 100.0;
export const NAVIGATION_RECOVERY_MS = 1200;
export const NAVIGATION_RECOVERY_CM = 12.0;
export const NAVIGATION_RECOVERY_SPEED_CM_S = 16.0;

export const GRAY_INVERT_ENABLED = 0;
export const LOST_LINE_STOP_ENABLED = 0;
export const FLOOR_SEAM_FILTER_ENABLED = 0;
