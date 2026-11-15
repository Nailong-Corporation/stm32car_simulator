import {
  Action,
  RouteState,
  RouteFailure,
  JunctionEvidence,
  JunctionReject,
  HexagonState,
  RouteBarHoldState,
  type HexagonNavigation,
  type RouteStep,
  type RouteStartPose,
  type GrayObservation,
  GrayKind,
} from './types';
import {
  CONTROL_MAX_INTERVAL_MS,
  LINE_CONFIRM_SAMPLES,
  GRAY_FORWARD_OFFSET_CM,
  TURN_PIVOT_APPROACH_CM,
  ROUTE_GATE_LOOKAHEAD_CM,
  SENSOR_MAX_REGION_WIDTH,
  JUNCTION_CONFIRM_SAMPLES,
  JUNCTION_CONFIRM_DISTANCE_CM,
  JUNCTION_EARLY_CONFIRM_SAMPLES,
  JUNCTION_EARLY_CONFIRM_CM,
  JUNCTION_EDGE_CONFIRM_SAMPLES,
  JUNCTION_EDGE_CONFIRM_CM,
  JUNCTION_EDGE_MAX_WIDTH,
  JUNCTION_SMOOTH_POSITION_STEP,
  JUNCTION_CONFIRM_TIMEOUT_MS,
  JUNCTION_CENTER_MEMORY_MS,
  JUNCTION_CENTER_MEMORY_CM,
  JUNCTION_ACTION_TIMEOUT_MS,
  JUNCTION_STRAIGHT_TIMEOUT_MS,
  JUNCTION_CLEAR_TIMEOUT_MS,
  JUNCTION_TURN_MIN_RAD,
  JUNCTION_EXIT_CONFIRM_MS,
  JUNCTION_EXIT_CONFIRM_CM,
  JUNCTION_EXIT_ANGLE_MARGIN_RATIO,
  JUNCTION_TURN_MAX_RAD,
  JUNCTION_WRONG_WAY_RAD,
  JUNCTION_CENTER_LIMIT,
  JUNCTION_SHORT_LINK_EXIT_POSITION_LIMIT,
  JUNCTION_SHORT_LINK_EXIT_EDGE_STABLE_MS,
  JUNCTION_SHORT_LINK_EXIT_EDGE_STABLE_CM,
  JUNCTION_EXIT_SEARCH_EDGE_HEADING_RAD,
  JUNCTION_EXIT_SEARCH_SPEED_CM_S,
  JUNCTION_EXIT_SEARCH_MAX_CM,
  JUNCTION_EXIT_SEARCH_TIMEOUT_MS,
  JUNCTION_EXIT_SEARCH_EDGE_STABLE_MS,
  JUNCTION_EXIT_SEARCH_EDGE_STABLE_CM,
  JUNCTION_CAPTURE_MIN_RAD,
  JUNCTION_YAW_FILTER_TAU_S,
  JUNCTION_CLEAR_MAX_CM,
  JUNCTION_TURN_TARGET_RAD,
  JUNCTION_SETTLE_SPEED_CM_S,
  COMPONENT_RECOVERY_MAX_CM,
  COMPONENT_RECOVERY_TIMEOUT_MS,
  BRAKING_CALIBRATION_CONFIRMED,
  BRAKING_SAFE_DECEL_CM_S2,
  JUNCTION_ENTRY_BRAKE_TIMEOUT_MS,
  JUNCTION_ENTRY_BRAKE_MAX_CM,
  JUNCTION_EXIT_ALIGNMENT_RAD,
  JUNCTION_EXIT_REWIND_RAD,
  JUNCTION_APPROACH_TIMEOUT_MS,
  JUNCTION_COUNT_CLEAR_MS,
  JUNCTION_COUNT_CLEAR_CM,
  JUNCTION_STRAIGHT_EXIT_MAX_WIDTH,
  JUNCTION_ADJACENT_EXIT_MS,
  JUNCTION_ADJACENT_EXIT_CM,
  HEXAGON_MAX_ANGLE_DEG,
  HEXAGON_SPEED_CM_S,
  HEXAGON_LEG_SPEED_CM_S,
  HEXAGON_PIVOT_LEAD_CM,
  HEXAGON_ENTRY_MIN_RAD,
  HEXAGON_ENTRY_TIMEOUT_MS,
  HEXAGON_ENTRY_MAX_CM,
  HEXAGON_ACTION_TIMEOUT_MS,
  HEXAGON_ACTION_MAX_CM,
  HEXAGON_EXIT_TIMEOUT_MS,
  HEXAGON_EXIT_MAX_CM,
  HEXAGON_OVERANGLE_RAD,
  HEXAGON_EXIT_CONFIRM_MS,
  HEXAGON_EXIT_CONFIRM_CM,
  HEXAGON_ENTRY_LOST_MS,
  HEXAGON_ENTRY_LOST_CM,
  ROUTE_START_X_CM,
  ROUTE_START_Y_CM,
  ROUTE_START_HEADING_RAD,
  ROUTE_REPEAT_COUNT,
  ROUTE_WAYPOINT_MARGIN_CM,
  ROUTE_MAX_STEPS,
  ROUTE_WAYPOINT_EXIT_CM,
  ROUTE_WAYPOINT_EXIT_ANGLE_RAD,
  HEXAGON_ENTRY_APPROACH_MAX_CM,
  HEXAGON_LEG_CROSS_MAX_CM,
  ROUTE_CLEAR_POSE_CM,
  HEXAGON_CENTER_X_CM,
  HEXAGON_CENTER_Y_CM,
  HEXAGON_SIDE_CM,
  HEXAGON_TURN_RAD,
  HEXAGON_ENTRY_TURN_RAD,
  HEXAGON_TURN_TOL_RAD,
  ROUTE_SHORT_SPEED_CM_S,
  ROUTE_LONG_SEGMENT_CM,
  NAVIGATION_RECOVERY_MS,
  NAVIGATION_RECOVERY_CM,
  WHEEL_TARGET_MAX_CM_S,
  CORNER_RADIUS_CM,
  CORNER_ARC_RADIUS_CM,
  CORNER_ARC_SPEED_CM_S,
  CORNER_ARC_ENTER_TOL_CM,
  CORNER_ARC_MIN_LEG_MARGIN_CM,
  CORNER_ARC_EXIT_ANGLE_RAD,
  CORNER_ARC_EXIT_TOL_CM,
  CORNER_ARC_EXIT_ALONG_TOL_CM,
  CORNER_ARC_EXIT_PROGRESS_RAD,
  CORNER_ARC_EXIT_LEAD_S,
  CORNER_ARC_DIFF_SLEW_CM_S2,
  WHEEL_TRACK_CM,
  INTERFERENCE_TIMEOUT_MS,
  INTERFERENCE_MAX_DISTANCE_CM,
  COMPONENT_CROSS_MAX_CM,
  COMPONENT_CROSS_MS,
  EXIT_COMPONENT_EXTEND_ENABLED,
  SHORT_LINK_TURN_EARLY_CM,
  DASHED_NODE_MIN_WHITE,
  TRANSISTOR_NODE_APPROACH_MAX_CM,
  TRANSISTOR_PIVOT_LEAD_CM,
} from './config';

const SHORT_LINK_MIN_CM = 20;
const SHORT_LINK_MAX_CM = 40;
const SHORT_LINK_MAX_LATERAL_CM = 14;
const SHORT_LINK_MAX_HEADING_RAD = 0.30;
const SHORT_LINK_TIMEOUT_MS = 6000;
function shortLinkLength(s: RouteStep): number { return s.short_link_cm ?? 0; }
function componentPattern(o: GrayObservation): boolean {
  return (o.region_count >= 2 && (o.line_mask & 0x18) === 0 &&
      (o.line_mask & 0x03) !== 0 && (o.line_mask & 0xc0) !== 0) ||
    (o.region_count === 1 && o.white_count >= 5 && o.white_count <= 7 &&
      (o.line_mask & 0x07) !== 0 && (o.line_mask & 0xe0) !== 0);
}
function ordinaryRightAngle(s: RouteStep): boolean {
  return !s.corner_arc_enabled && (s.action === Action.LEFT || s.action === Action.RIGHT ||
    (s.action === Action.CUSTOM_TURN && Math.abs(Math.abs(s.angle_deg) - 90) < 0.001));
}
/* Rolling-corner eligibility: an ordinary 90 degree corner whose own leg and
 * whose outgoing leg both carry explicit geometry (short links keep their
 * dedicated tracking, so a row feeding or leaving one stays on the pivot). */
export function cornerArcRow(s: RouteStep | undefined, next?: RouteStep): boolean {
  if (!s || !s.corner_arc_enabled) return false;
  if (shortLinkLength(s) > 0) return false;
  if (next && shortLinkLength(next) > 0) return false;
  // The exit tangent point sits R past the corner, and the 24 cm look-ahead bar
  // of the NEXT row starts watching ~21 cm early, so a leg shorter than ~2R
  // makes the next corner's trigger fire while this arc is still turning.
  if (next &&
    next.segment_length_cm + ROUTE_WAYPOINT_MARGIN_CM <
      2 * CORNER_ARC_RADIUS_CM + CORNER_ARC_MIN_LEG_MARGIN_CM) {
    return false;
  }
  if (s.action === Action.LEFT || s.action === Action.RIGHT) return true;
  return s.action === Action.CUSTOM_TURN && Math.abs(Math.abs(s.angle_deg) - 90) < 0.001;
}
function wrapRad(a: number): number {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

export function routeIsNarrowLine(o: GrayObservation): boolean {
  return (
    o.kind === GrayKind.GRAY_VALID &&
    o.region_count === 1 &&
    o.white_count > 0 &&
    o.white_count <= SENSOR_MAX_REGION_WIDTH
  );
}

export function routeIsClearLine(planner: RoutePlannerModel, o: GrayObservation): boolean {
  if (!routeIsNarrowLine(o)) return false;
  return (
    planner.currentAction() !== Action.STRAIGHT ||
    (Math.abs(o.position) <= JUNCTION_CENTER_LIMIT &&
      o.white_count <= JUNCTION_STRAIGHT_EXIT_MAX_WIDTH)
  );
}

function separatedCenterBranch(o: GrayObservation): boolean {
  return (
    o.region_count === 2 &&
    o.white_count >= 3 &&
    (o.line_mask & 0x18) !== 0 &&
    (o.line_mask & 0xc3) !== 0
  );
}

function branchSides(o: GrayObservation): number {
  const left = o.line_mask & 0x07;
  const right = (o.line_mask >> 5) & 0x07;
  return (
    (left && (left & (left - 1)) ? 1 : 0) |
    (right && (right & (right - 1)) ? 2 : 0)
  );
}

function separatedSides(o: GrayObservation): number {
  if (!separatedCenterBranch(o)) return 0;
  let center = o.line_mask & 0x18;
  for (let i = 0; i < 7; ++i) {
    center |= o.line_mask & (((center << 1) | (center >> 1)) & 0xff);
  }
  if (!(center & (center - 1))) return 0;
  const outer = o.line_mask & (~center & 0xff);
  return ((outer & 0x03) ? 1 : 0) | ((outer & 0xc0) ? 2 : 0);
}

export function routeGrayShowsBranch(o: GrayObservation): boolean {
  /* 2026-11-10 固件 59a7268：巡线层的“这一帧是不是合并进来的支路”与路由计数共用
   * 同一形状分型。单区域但带“中心两路 + 侧支”，或两区域/空心图形，都表示杆下
   * 不只是本行那一条线；此时航向保持本行线段方向，不把单区域质心交给位置 PID。 */
  const { kind, sides } = junctionShape(o);
  return kind !== JunctionEvidence.JUNCTION_EVIDENCE_NONE && sides !== 0;
}

function junctionShape(o: GrayObservation): { kind: JunctionEvidence; sides: number } {
  // Printed components are counted nodes, with the same temporal/motion proof
  // and clearance lock as a branch. Crossing control remains independent.
  const hollow = o.region_count === 2 && o.white_count >= 2 &&
    (o.line_mask & 0x18) === 0 && (o.line_mask & 0x07) !== 0 && (o.line_mask & 0xe0) !== 0;
  if ((o.region_count === 2 && o.white_count >= 3) || hollow) {
    const sides = ((o.line_mask & 0x07) ? 1 : 0) | ((o.line_mask & 0xe0) ? 2 : 0);
    return { kind: JunctionEvidence.JUNCTION_EVIDENCE_SEPARATED, sides: sides || 3 };
  }
  const separated = separatedSides(o);
  if (separated) {
    return { kind: JunctionEvidence.JUNCTION_EVIDENCE_SEPARATED, sides: separated };
  }
  // Printed symbols (2026-11-06 track) are not junctions: a multi-region
  // pattern without the central two channels is a hollow resistor / V arm /
  // plate gap. Only a center-bearing separated branch counts.
  if (o.region_count !== 1) {
    return { kind: JunctionEvidence.JUNCTION_EVIDENCE_NONE, sides: 0 };
  }
  let sides = branchSides(o);
  if (o.white_count >= 4 && (o.line_mask & 0x18) === 0x18 && !sides) sides = 3;
  if (sides && (o.line_mask & 0x18) && o.white_count >= 3) {
    return { kind: JunctionEvidence.JUNCTION_EVIDENCE_CENTER_BRANCH, sides };
  }
  // 2026-11-06: a narrow line with only an outer group (no central pair) is a
  // printed symbol V arm / resistor cap, not a branch; the old weak EDGE node
  // is retired so symbols cannot authorize or miscount junctions.
  return { kind: JunctionEvidence.JUNCTION_EVIDENCE_NONE, sides: 0 };
}

export class RoutePlannerModel {
  public steps: RouteStep[] = [];
  public count = 0;
  public index = 0;
  public state: RouteState = RouteState.ROUTE_FOLLOW;
  public failure: RouteFailure = RouteFailure.ROUTE_FAILURE_NONE;

  public hexagon: HexagonNavigation = {
    state: HexagonState.HEXAGON_OFF,
    phase_ms: 0,
    action_ms: 0,
    line_ms: 0,
    branch_ms: 0,
    center_ms: 0,
    phase_cm: 0,
    line_cm: 0,
    branch_cm: 0,
    center_cm: 0,
    hold_heading: 0,
    line_samples: 0,
    branch_samples: 0,
    center_samples: 0,
    line_seen: false,
    use_line: false,
    center_pending: false,
    exit_armed: false,
    target_heading: 0,
    turning: false,
    turn_index: 0,
    vertex_x_cm: 0,
    speed_cm_s: 0,
    vertex_y_cm: 0,
    leg_start_cm: 0,
    leg_from_x_cm: 0,
    leg_from_y_cm: 0,
    cross_cm: 0,
  };

  public navigation_warning = RouteFailure.ROUTE_FAILURE_NONE;
  public navigation_recoveries = 0;
  public next_segment_origin_cm = 0;
  public next_segment_ms = 0;
  public next_segment_origin_valid = false;
  public junctions_seen = 0;
  public passing_junction = false;
  public final_segment_active = false;
  public search_pending = false;
  public final_segment_origin_cm = 0;
  public search_origin_cm = 0;
  public search_ms = 0;
  public last_sample_ms = 0;
  public measured_left_cm_s = 0;
  public measured_right_cm_s = 0;

  public origin_cm = 0;
  public entry_cm = 0;
  public entry_heading = 0;
  public clear_cm = 0;
  public approach_center_cm = 0;
  public action_start_heading = 0;
  public turn_approaching = false;
  public turn_rewind_active = false;
  public turn_rewind_pending = false;
  public turn_rewind_ms = 0;
  public turn_approach_origin_cm = 0;
  /* Per-action pivot approach distance; a pose trigger whose bar already
   * passed the corner shortens it so the pivot still starts ~track/2 before
   * the corner instead of tens of cm late. */
  public turn_approach_cm = 0;
  public pose_trigger_late_cm = 0;
  /* 三极管正面横杠：首帧 0xFF 锁航向，按本行 turn_approach_cm 直走。 */
  public bar_hold_state: RouteBarHoldState = RouteBarHoldState.ROUTE_BAR_HOLD_NONE;
  public bar_hold_index = 0;
  public bar_hold_origin_cm = 0;
  public bar_hold_heading_rad = 0;
  public bar_hold_ms = 0;
  public bar_hold_clear_count = 0;
  public bar_hold_complete = false;
  /* Pose mode: after a pivot the car keeps exit speed until the outgoing line
   * has been re-acquired, so a printed symbol next to the corner cannot be
   * chased at cruise speed. */
  public post_turn_slow = false;
  public post_turn_samples = 0;
  public short_link_tracking = false;
  public close_corner_tracking = false;
  public short_link_active = false;
  public short_link_heading = 0;
  public short_link_forward_cm = 0;
  public short_link_lateral_cm = 0;
  public short_link_pivot_lead_cm = 0;
  public short_link_last_cm = 0;
  public short_link_last_heading = 0;
  public short_link_ms = 0;
  public segment_ms = 0;
  public phase_ms = 0;
  public approach_center_ms = 0;
  public exit_candidate_ms = 0;
  public exit_candidate_cm = 0;
  public exit_candidate_pending = false;
  /* 短连接/普通偏边出口的“同侧连续采样”：side 1 = 负位置、2 = 正位置。
   * 换侧即清零，避免把扫线时的一帧当出口（route.h short_exit_edge_*）。 */
  public short_exit_edge_side = 0;
  public short_exit_edge_samples = 0;
  public repeat_action: Action = Action.LEFT;
  public repeat_count = ROUTE_REPEAT_COUNT;
  public completed_laps = 0;
  public candidate_count = 0;
  public exit_count = 0;
  public approach_center_count = 0;
  public have_approach_center = false;
  public valid = false;
  public stop_pending = false;
  public callback_dispatched = false;
  public corner_arc_complete = false;
  /* Rolling corner state (pose mode): the arc geometry is rebuilt from the row's
   * corner waypoint and the entry heading, so the axle tracks a circle that is
   * tangent to both segments.  arc_active turns true once the axle reaches the
   * entry tangent point; before that the car rolls straight at arc speed. */
  public arc_active = false;
  /* Set when the rolling corner hands off and cleared when the next row's
   * action begins: the exit speed is kept and the yaw residue is damped while
   * the wheels unwind, instead of dropping to the pivot exit speed. */
  public arc_just_exited = false;
  public arc_roll_cm = 0;
  public arc_progress_rad = 0;
  public arc_radius_err_cm = 0;
  public arc_desired_heading_rad = 0;
  public arc_outer_cm_s = 0;
  public arc_diff_cm_s = 0;
  public arc_diff_slew_cm_s = 0;
  public arc_lateral_cm = 0;
  public arc_along_cm = 0;
  public arc_across_cm = 0;
  public arc_heading_err_rad = 0;
  public arc_yaw_rate_rad_s = 0;
  public arc_last_heading = 0;
  public arc_exit_lead_rad = 0;
  public last_arc_ms = 0;
  public turn_capturing = false;
  public lap_boundary = false;
  public capture_line_seen = false;
  public line_exit_confirmed = false;
  public capture_line_samples = 0;
  public capture_line_ms = 0;
  public exit_component_extension_used = false;
  public capture_line_cm = 0;
  public turn_exit_search = false;
  public line_exit_heading_rad = 0;
  public turn_ended_on_line = false;
  public next_junction_samples = 0;
  // Short legs: node evidence the 24 cm lookahead already crossed mid-turn.
  public turn_branch_samples = 0;
  public turn_branch_sides = 0;
  public next_junction_ms = 0;
  public next_junction_cm = 0;
  public candidate_separated = false;
  public line_confirmed_exit_used = false;
  public candidate_kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE;
  public last_confirmed_kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE;
  public candidate_reject = JunctionReject.JUNCTION_REJECT_NONE;
  public candidate_sides = 0;
  public next_junction_sides = 0;
  public previous_line_mask = 0;
  public smooth_edge_sides = 0;
  public previous_line_valid = false;
  public previous_line_position = 0;
  public candidate_restarts = 0;
  public smooth_edge_rejections = 0;
  public component_recovery_active = false;
  public component_recovery_used = false;
  public component_recovery_line_samples = 0;
  public component_recovery_ms = 0;
  public component_recovery_cm = 0;
  public component_recovery_heading = 0;

  /* Absolute corner waypoints (track frame, cm) recovered from the start pose
   * and the cumulative (geometric length, angle) chain; the table stores the
   * lengths already reduced by ROUTE_WAYPOINT_MARGIN_CM. */
  public waypoint_x_cm: number[] = [];
  public waypoint_y_cm: number[] = [];
  /* True while the caller supplies the axle pose; the controller then uses the
   * pose for turn pivots and corner hand-off. */
  public pose_mode = false;
  /* Bar-to-waypoint gap of the active row (cm); Infinity when pose is absent. */
  public pose_gap_bar_cm = Infinity;
  /* Controller-frame bearing from the axle to the active row's corner. */
  public pose_bearing_rad = 0;
  /* Signed lateral offset of the axle from the active row's segment line
   * (positive = axle right of the travel direction).  Used to veto a grey
   * reading that cannot belong to this segment (printed branch/symbol). */
  public pose_cross_cm = 0;
  /* Cross-track of the 24 cm look-ahead bar (what the grey sensor actually
   * samples); differs from the axle value by the heading error times the
   * look-ahead. */
  public pose_cross_bar_cm = 0;
  /* Controller-frame heading of the active row's segment (previous corner ->
   * this corner), used for path-following between the grey line drops. */
  public pose_seg_heading_rad = 0;
  public pose_cross_valid = false;
  /* Track-frame pose the route table is anchored on.  The official run starts
   * at P (0,110) heading north; scenario tests may re-anchor it before start. */
  public start_x_cm = ROUTE_START_X_CM;
  public start_y_cm = ROUTE_START_Y_CM;
  public start_heading_rad = ROUTE_START_HEADING_RAD;
  /* Pose navigation switch: production keeps it on; the legacy FSM regression
   * suites switch it off to pin the grey-only contract. */
  public pose_enabled = true;

  public stop_callback?: (step_index: number) => void;

  public init(steps: RouteStep[], callback?: (step_index: number) => void,
    startPose?: RouteStartPose): void {
    this.steps = steps.map((s) => ({ ...s }));
    this.count = steps.length;
    this.repeat_count = ROUTE_REPEAT_COUNT;
    this.repeat_action = Action.LEFT;
    this.stop_callback = callback;
    /* Table-selected scenario anchor (route_scenario_start_pose): the compiled
     * table may re-anchor the track frame (scheme 8 ring test starts at
     * (345,110) heading south); otherwise the production P(0,110) north. */
    this.start_x_cm = startPose ? startPose.x_cm : ROUTE_START_X_CM;
    this.start_y_cm = startPose ? startPose.y_cm : ROUTE_START_Y_CM;
    this.start_heading_rad = startPose ? startPose.heading_rad : ROUTE_START_HEADING_RAD;
    /* Explicit upper bound: over ROUTE_MAX_STEPS must fail, not silently
     * disable pose navigation (motion_config.h route_init). */
    this.valid = steps !== null && steps.length > 0 && steps.length <= ROUTE_MAX_STEPS;

    for (let i = 0; this.valid && i < this.count; ++i) {
      const s = steps[i];
      const approach = s.turn_approach_cm ?? 0;
      this.valid =
        (s.junction_count > 0 || s.segment_length_cm > 0) &&
        s.action <= Action.HEXAGON &&
        s.cruise_speed > 0 &&
        s.cruise_speed <= WHEEL_TARGET_MAX_CM_S &&
        Number.isFinite(s.turn_speed) &&
        s.turn_speed > 0 &&
        s.turn_speed <= s.cruise_speed &&
        Number.isFinite(approach) &&
        approach >= 0 &&
        Number.isFinite(s.segment_length_cm) &&
        s.segment_length_cm >= 0 &&
        (s.segment_length_cm === 0 ||
          (BRAKING_CALIBRATION_CONFIRMED &&
            Number.isFinite(BRAKING_SAFE_DECEL_CM_S2) &&
            BRAKING_SAFE_DECEL_CM_S2 > 0)) &&
        (s.action !== Action.CUSTOM_TURN ||
          (Number.isFinite(s.angle_deg) && s.angle_deg !== 0 && Math.abs(s.angle_deg) <= 360)) &&
        (s.action !== Action.HEXAGON ||
          (Number.isFinite(s.angle_deg) &&
            Math.abs(s.angle_deg) >= 60.0 &&
            Math.abs(s.angle_deg) <= HEXAGON_MAX_ANGLE_DEG &&
            !s.corner_arc_enabled)) &&
        (s.action !== Action.STOP || i === this.count - 1) &&
        (!s.corner_arc_enabled ||
          (Number.isFinite(CORNER_ARC_RADIUS_CM) &&
            CORNER_ARC_RADIUS_CM > WHEEL_TRACK_CM * 0.5)) &&
        Number.isFinite(shortLinkLength(s)) && (shortLinkLength(s) === 0 ||
          (i > 0 && shortLinkLength(s) >= SHORT_LINK_MIN_CM && shortLinkLength(s) <= SHORT_LINK_MAX_CM &&
            ordinaryRightAngle(s) &&
            (ordinaryRightAngle(steps[i - 1]) || steps[i - 1].corner_arc_enabled)));
    }

    if (this.valid && steps[this.count - 1].action !== Action.STOP) this.valid = false;
    this.state = this.valid ? RouteState.ROUTE_FOLLOW : RouteState.ROUTE_FAILED;
    this.failure = this.valid ? RouteFailure.ROUTE_FAILURE_NONE : RouteFailure.ROUTE_FAILURE_INVALID;
  }

  public start(d: number, now: number): void {
    this.short_link_tracking = this.short_link_active = this.turn_approaching = false;
    this.close_corner_tracking = false;
    this.bar_hold_state = RouteBarHoldState.ROUTE_BAR_HOLD_NONE;
    this.bar_hold_clear_count = 0;
    this.post_turn_slow = false;
    this.post_turn_samples = 0;
    this.arc_just_exited = false;
    /* Absolute corner waypoints: the anchored start pose plus the cumulative
     * (geometry - 5 cm + 5 cm, angle) chain. Computed at START so a scenario
     * test may re-anchor the start pose first. */
    this.waypoint_x_cm = [];
    this.waypoint_y_cm = [];
    let wx = this.start_x_cm;
    let wy = this.start_y_cm;
    let wh = this.start_heading_rad;
    for (let i = 0; i < this.count && i < ROUTE_MAX_STEPS; ++i) {
      const len = this.steps[i].segment_length_cm > 0
        ? this.steps[i].segment_length_cm + ROUTE_WAYPOINT_MARGIN_CM
        : 0;
      wx += len * Math.cos(wh);
      wy += len * Math.sin(wh);
      this.waypoint_x_cm.push(wx);
      this.waypoint_y_cm.push(wy);
      wh += (this.steps[i].angle_deg * Math.PI) / 180.0;
    }
    this.index = 0;
    this.completed_laps = 0;
    this.junctions_seen = 0;
    this.origin_cm = this.final_segment_origin_cm = d;
    this.segment_ms = now;
    this.passing_junction = this.lap_boundary = false;
    this.search_pending = this.stop_pending = this.callback_dispatched = false;
    this.final_segment_active = this.valid;
    this.navigation_warning = RouteFailure.ROUTE_FAILURE_NONE;
    this.navigation_recoveries = 0;
    this.failure = RouteFailure.ROUTE_FAILURE_NONE;
    this.last_confirmed_kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE;
    this.candidate_restarts = this.smooth_edge_rejections = 0;
    this.state = this.valid && Number.isFinite(d) ? RouteState.ROUTE_FOLLOW : RouteState.ROUTE_FAILED;
    if (this.state === RouteState.ROUTE_FAILED) this.failure = RouteFailure.ROUTE_FAILURE_INVALID;
    this.resetEvidence(now);
  }

  public setSpeed(left: number, right: number): void {
    this.measured_left_cm_s = left;
    this.measured_right_cm_s = right;
  }

  public fail(reason: RouteFailure): void {
    if (this.failure === RouteFailure.ROUTE_FAILURE_NONE) this.failure = reason;
    this.state = RouteState.ROUTE_FAILED;
  }

  public anotherLap(): boolean {
    return this.repeat_count === 0 || this.completed_laps < this.repeat_count - 1;
  }

  public currentAction(): Action {
    if (this.passing_junction) return Action.STRAIGHT;
    const action = this.steps[this.index].action;
    if (action === Action.CUSTOM_TURN || action === Action.HEXAGON) {
      return this.steps[this.index].angle_deg < 0 ? Action.LEFT : Action.RIGHT;
    }
    return action === Action.STOP && (this.lap_boundary || this.anotherLap())
      ? this.repeat_action
      : action;
  }

  public usesCornerArc(): boolean {
    return (
      !this.passing_junction &&
      cornerArcRow(this.steps[this.index], this.steps[this.index + 1]) &&
      Math.abs(Math.abs(this.turnTargetRad()) - JUNCTION_TURN_TARGET_RAD) < 0.0001
    );
  }

  public turnTargetRad(): number {
    if (
      this.steps[this.index].action === Action.CUSTOM_TURN ||
      this.steps[this.index].action === Action.HEXAGON
    ) {
      return this.steps[this.index].angle_deg * (JUNCTION_TURN_TARGET_RAD / 90.0);
    }
    const action = this.currentAction();
    return action === Action.LEFT
      ? -JUNCTION_TURN_TARGET_RAD
      : action === Action.RIGHT
      ? JUNCTION_TURN_TARGET_RAD
      : 0.0;
  }

  public turnThresholdRad(threshold: number): number {
    if (this.steps[this.index].action === Action.HEXAGON) {
      return Math.abs(this.turnTargetRad()) + HEXAGON_OVERANGLE_RAD;
    }
    if (this.steps[this.index].action !== Action.CUSTOM_TURN) return threshold;
    const target = Math.abs(this.turnTargetRad());
    if (threshold === JUNCTION_TURN_MIN_RAD) return target;
    if (threshold > JUNCTION_TURN_TARGET_RAD) {
      return target + threshold - JUNCTION_TURN_TARGET_RAD;
    }
    return Math.max(
      target + threshold - JUNCTION_TURN_TARGET_RAD,
      (target * threshold) / JUNCTION_TURN_TARGET_RAD
    );
  }

  public actionTimeoutMs(): number {
    if (this.steps[this.index].action === Action.HEXAGON && !this.passing_junction) {
      return HEXAGON_ACTION_TIMEOUT_MS;
    }
    if (this.currentAction() === Action.STRAIGHT) return JUNCTION_STRAIGHT_TIMEOUT_MS;
    const scale = Math.max(1.0, Math.abs(this.turnTargetRad()) / JUNCTION_TURN_TARGET_RAD);
    return Math.ceil(JUNCTION_ACTION_TIMEOUT_MS * scale);
  }

  public entrySpeedLimit(): number {
    return this.entrySpeedLimitAt(this.index, true);
  }

  private entrySpeedLimitAt(index: number, active_action: boolean): number {
    const a = this.steps[index].action;
    const turn =
      a === Action.LEFT ||
      a === Action.RIGHT ||
      a === Action.CUSTOM_TURN ||
      a === Action.HEXAGON ||
      (a === Action.STOP &&
        ((active_action && this.lap_boundary) || this.anotherLap()) &&
        this.repeat_action !== Action.STRAIGHT);
    let limit = this.steps[index].turn_speed;
    if (a === Action.HEXAGON) limit = Math.min(limit, HEXAGON_SPEED_CM_S);
    if (turn && cornerArcRow(this.steps[index], this.steps[index + 1])) {
      // A rolling corner never stops the inner wheel, so the entry gate is the
      // axle arc speed rather than the settle/pivot cap.
      return CORNER_ARC_SPEED_CM_S;
    }
    return turn ? Math.min(limit, JUNCTION_SETTLE_SPEED_CM_S) : limit;
  }

  private entrySpeedReady(): boolean {
    const limit = this.entrySpeedLimit();
    return (
      Math.abs(this.measured_left_cm_s) <= limit &&
      Math.abs(this.measured_right_cm_s) <= limit
    );
  }

  public planningIndex(): number {
    if (
      this.state === RouteState.ROUTE_CLEARING &&
      !this.passing_junction &&
      this.next_segment_origin_valid
    ) {
      const next = this.lap_boundary ? 0 : this.index + 1;
      if (next < this.count) return next;
    }
    return this.index;
  }

  public planningOrigin(): number {
    return this.state === RouteState.ROUTE_CLEARING &&
      !this.passing_junction &&
      this.next_segment_origin_valid
      ? this.next_segment_origin_cm
      : this.final_segment_origin_cm;
  }

  public planningEntrySpeedLimit(): number {
    const preview =
      this.state === RouteState.ROUTE_CLEARING &&
      !this.passing_junction &&
      this.next_segment_origin_valid;
    return this.entrySpeedLimitAt(this.planningIndex(), !preview);
  }

  public cruiseSpeed(): number {
    const s = this.steps[this.planningIndex()];
    return s.segment_length_cm > ROUTE_LONG_SEGMENT_CM
      ? s.cruise_speed
      : ROUTE_SHORT_SPEED_CM_S;
  }

  public targetReady(d: number, next: boolean): boolean {
    const s = this.steps[this.index];
    // Positive segment distance is mandatory; count cannot release it early.
    // Readiness only arms a subsequently confirmed grey node, never a pose turn.
    // 2026-11-10: the mileage is the sensor bar's (24 cm ahead of the axle), so a
    // corner node seen by the bar authorizes instead of being dropped as passing.
    if (s.segment_length_cm > 0) {
      return d + ROUTE_GATE_LOOKAHEAD_CM - this.origin_cm >= s.segment_length_cm;
    }
    return s.junction_count > 0 &&
      this.junctions_seen + (next ? 1 : 0) >= s.junction_count;
  }

  private refreshDistance(d: number, now: number): void {
    const s = this.steps[this.planningIndex()];
    if (
      s.segment_length_cm > 0 &&
      d - this.planningOrigin() >= s.segment_length_cm &&
      !this.search_pending
    ) {
      this.search_pending = true;
      this.search_ms = now;
      this.search_origin_cm = d;
    }
  }

  private resetEvidence(now: number): void {
    this.next_segment_origin_valid = false;
    this.candidate_count = this.exit_count = this.approach_center_count = 0;
    this.have_approach_center = this.exit_candidate_pending = false;
    this.turn_capturing = this.corner_arc_complete = false;
    this.arc_active = false;
    this.arc_roll_cm = this.arc_progress_rad = this.arc_radius_err_cm = 0;
    this.arc_diff_slew_cm_s = 0;
    this.arc_desired_heading_rad = this.entry_heading;
    this.arc_outer_cm_s = CORNER_ARC_SPEED_CM_S;
    this.arc_diff_cm_s = 0;
    this.arc_lateral_cm = 0;
    this.turn_rewind_active = this.turn_rewind_pending = false;
    this.line_exit_confirmed = this.turn_ended_on_line = this.capture_line_seen = false;
    this.capture_line_samples = 0;
    this.turn_exit_search = false;
    this.next_junction_samples = 0;
    this.turn_branch_samples = 0;
    this.turn_branch_sides = 0;
    this.candidate_separated = false;
    this.line_confirmed_exit_used = false;
    this.candidate_kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE;
    this.candidate_sides = 0;
    this.next_junction_sides = 0;
    this.candidate_reject = JunctionReject.JUNCTION_REJECT_NONE;
    this.previous_line_valid = false;
    this.smooth_edge_sides = 0;
    this.component_recovery_active = this.component_recovery_used = false;
    this.component_recovery_line_samples = 0;
    this.last_sample_ms = now;
    this.hexagon = {
      state: HexagonState.HEXAGON_OFF,
      phase_ms: 0,
      action_ms: 0,
      line_ms: 0,
      branch_ms: 0,
      center_ms: 0,
      phase_cm: 0,
      line_cm: 0,
      branch_cm: 0,
      center_cm: 0,
      hold_heading: 0,
      line_samples: 0,
      branch_samples: 0,
      center_samples: 0,
      line_seen: false,
      use_line: false,
      center_pending: false,
      exit_armed: false,
      target_heading: 0,
      turning: false,
      turn_index: 0,
      vertex_x_cm: 0,
      speed_cm_s: 0,
      vertex_y_cm: 0,
      leg_start_cm: 0,
      leg_from_x_cm: 0,
      leg_from_y_cm: 0,
      cross_cm: 0,
    };
  }

  private rowTurnApproachCm(): number {
    const approach = this.steps[this.index].turn_approach_cm ?? 0;
    return approach > 0 ? approach : TURN_PIVOT_APPROACH_CM;
  }

  /** C: close_corner_approach_cm(). The row's pivot is placed on the geometric
   * corner instead of a fixed straight distance, so a resistor right in front
   * of the corner cannot start the pivot ~10 cm early. Table value, segment
   * gate, counting, angle and speed are untouched. */
  private closeCornerApproachCm(d: number): number {
    const s = this.steps[this.index];
    if (!(s.segment_length_cm > 0)) return 0;
    const corner = this.origin_cm + s.segment_length_cm + ROUTE_WAYPOINT_MARGIN_CM;
    let pivot = corner - WHEEL_TRACK_CM * 0.5;
    const transistor = (s.segment_length_cm === 45 || s.segment_length_cm === 245) &&
      (s.turn_approach_cm ?? 0) >= 12;
    if (transistor && s.segment_length_cm >= 200) {
      pivot = Math.min(pivot, d + TRANSISTOR_NODE_APPROACH_MAX_CM - TRANSISTOR_PIVOT_LEAD_CM);
    }
    return Math.max(0, pivot - d);
  }

  /** Mirror of route_bar_hold_active(): the latched transverse bar keeps the
   * confirmed entry heading for the row's configured straight distance. */
  public dashedHold(): boolean {
    if (this.index >= this.count) return false;
    if (this.steps[this.index].dashed_row) return true;
    return this.state === RouteState.ROUTE_CLEARING && !this.passing_junction &&
      this.index + 1 < this.count && !!this.steps[this.index + 1].dashed_row;
  }

  public barHoldActive(d: number): boolean {
    if (this.index >= this.count ||
      this.bar_hold_state === RouteBarHoldState.ROUTE_BAR_HOLD_NONE ||
      this.bar_hold_index !== this.index ||
      this.bar_hold_complete) return false;
    const phase =
      this.state === RouteState.ROUTE_FOLLOW ||
      this.state === RouteState.ROUTE_CANDIDATE ||
      this.state === RouteState.ROUTE_BRAKING ||
      (this.state === RouteState.ROUTE_CLEARING && this.passing_junction) ||
      (this.state === RouteState.ROUTE_EXECUTING && this.turn_approaching);
    return phase && d >= this.bar_hold_origin_cm &&
      d - this.bar_hold_origin_cm < (this.steps[this.index].turn_approach_cm ?? 0);
  }

  /** Candidate cancellation / side change only unbinds the bar; it never
   * cancels the steering hold (C: unbind_bar_hold). */
  private unbindBarHold(): void {
    if (this.bar_hold_state === RouteBarHoldState.ROUTE_BAR_HOLD_PENDING ||
      this.bar_hold_state === RouteBarHoldState.ROUTE_BAR_HOLD_SIDE) {
      this.bar_hold_state = RouteBarHoldState.ROUTE_BAR_HOLD_PASS;
    }
  }

  /** A bar is this row's corner only when the pivot start (first bar frame +
   * the row's straight distance) puts the 24 cm look-ahead bar on the segment
   * end. Mid-row printed-resistor end walls stay "passing" nodes. */
  private barMarksCorner(): boolean {
    if (this.bar_hold_state !== RouteBarHoldState.ROUTE_BAR_HOLD_PENDING ||
      this.bar_hold_index !== this.index) return false;
    const s = this.steps[this.index];
    const approach = s.turn_approach_cm ?? 0;
    if (s.segment_length_cm <= 0 || approach <= 0) return false;
    const pivot = this.bar_hold_origin_cm + approach;
    if (pivot + GRAY_FORWARD_OFFSET_CM <
      this.origin_cm + s.segment_length_cm + ROUTE_WAYPOINT_MARGIN_CM) return false;
    return this.targetReady(pivot, false);
  }

  private updateBarHold(o: GrayObservation, d: number, h: number, now: number): void {
    const s = this.steps[this.index];
    const phase =
      this.state === RouteState.ROUTE_FOLLOW ||
      this.state === RouteState.ROUTE_CANDIDATE ||
      (this.state === RouteState.ROUTE_CLEARING && this.passing_junction) ||
      (this.state === RouteState.ROUTE_EXECUTING && this.turn_approaching) ||
      this.state === RouteState.ROUTE_BRAKING;
    const eligible = (s.turn_approach_cm ?? 0) > 0 &&
      !s.corner_arc_enabled &&
      (s.short_link_cm ?? 0) === 0 &&
      (s.action === Action.LEFT || s.action === Action.RIGHT || s.action === Action.CUSTOM_TURN);
    if (!eligible || !phase || this.bar_hold_index !== this.index) {
      this.bar_hold_state = RouteBarHoldState.ROUTE_BAR_HOLD_NONE;
      this.bar_hold_clear_count = 0;
    }
    if (this.bar_hold_state !== RouteBarHoldState.ROUTE_BAR_HOLD_NONE) {
      if (!this.bar_hold_complete) {
        if (now - this.bar_hold_ms >= JUNCTION_APPROACH_TIMEOUT_MS) {
          this.fail(RouteFailure.ROUTE_FAILURE_EXIT_RECAPTURE);
          return;
        }
        if (d - this.bar_hold_origin_cm >= (s.turn_approach_cm ?? 0)) {
          this.bar_hold_complete = true;
        }
      }
      if (this.bar_hold_complete) {
        const center =
          this.state === RouteState.ROUTE_FOLLOW &&
          routeIsNarrowLine(o) &&
          o.white_count <= JUNCTION_STRAIGHT_EXIT_MAX_WIDTH &&
          Math.abs(o.position) <= JUNCTION_CENTER_LIMIT &&
          now - this.last_sample_ms <= CONTROL_MAX_INTERVAL_MS;
        if (!center) this.bar_hold_clear_count = 0;
        else if (this.bar_hold_clear_count < LINE_CONFIRM_SAMPLES) this.bar_hold_clear_count++;
        if (this.bar_hold_clear_count >= LINE_CONFIRM_SAMPLES &&
          this.bar_hold_state !== RouteBarHoldState.ROUTE_BAR_HOLD_PENDING) {
          this.bar_hold_state = RouteBarHoldState.ROUTE_BAR_HOLD_NONE;
        }
      }
      /* A full transverse bar may follow the partial edge of the SAME live
       * candidate. Give it its own anchor; a cancelled or confirmed passage
       * has already become PASS and cannot lend an old symbol to a corner. */
      if (this.bar_hold_state === RouteBarHoldState.ROUTE_BAR_HOLD_SIDE &&
        !this.bar_hold_complete &&
        this.state === RouteState.ROUTE_CANDIDATE &&
        o.line_mask === 0xff &&
        now - this.last_sample_ms <= CONTROL_MAX_INTERVAL_MS) {
        this.bar_hold_state = RouteBarHoldState.ROUTE_BAR_HOLD_NONE;
      } else {
        return;
      }
    }
    /* Steering freezes on the first filtered bar; node confirmation stays
     * independent. An early bar can never be reused after the distance gate. */
    if (!eligible || !phase ||
      this.state === RouteState.ROUTE_BRAKING ||
      this.state === RouteState.ROUTE_EXECUTING) return;
    const fullBar = o.line_mask === 0xff;
    /* A side-on transistor first covers only part of the bar (field: 0x1f,
     * then 0x0f). Hold before its V arm becomes an ordinary narrow line.
     * This partial observation can only grant passage, never a corner anchor. */
    const passBranch = !this.targetReady(d, true) &&
      o.white_count >= 4 && routeGrayShowsBranch(o);
    if (!fullBar && !passBranch) return;
    this.bar_hold_index = this.index;
    this.bar_hold_origin_cm = d;
    this.bar_hold_heading_rad = h;
    this.bar_hold_ms = now;
    this.bar_hold_clear_count = 0;
    this.bar_hold_complete = false;
    this.bar_hold_state = this.state === RouteState.ROUTE_CLEARING
      ? RouteBarHoldState.ROUTE_BAR_HOLD_PASS
      : fullBar
        ? RouteBarHoldState.ROUTE_BAR_HOLD_PENDING
        : RouteBarHoldState.ROUTE_BAR_HOLD_SIDE;
  }

  private beginAction(d: number, h: number, now: number): void {
    const explicitApproach = (this.steps[this.index].turn_approach_cm ?? 0) > 0;
    let approachOrigin = explicitApproach && this.state === RouteState.ROUTE_BRAKING
      ? this.entry_cm : d;
    let approachHeading = explicitApproach && this.state === RouteState.ROUTE_BRAKING
      ? this.entry_heading : h;
    if (explicitApproach &&
      this.bar_hold_state === RouteBarHoldState.ROUTE_BAR_HOLD_TARGET &&
      this.bar_hold_index === this.index &&
      !this.short_link_active) {
      approachOrigin = this.bar_hold_origin_cm;
      approachHeading = this.bar_hold_heading_rad;
    }
    this.state = RouteState.ROUTE_EXECUTING;
    this.phase_ms = now;
    this.entry_cm = d;
    if (!this.short_link_active) this.entry_heading = approachHeading;
    this.action_start_heading = this.entry_heading;
    this.turn_approach_origin_cm = approachOrigin;
    this.turn_approach_cm = Math.max(0.0, this.rowTurnApproachCm() -
      (!explicitApproach && this.pose_mode ? this.pose_trigger_late_cm : 0.0));
    if (explicitApproach && !this.short_link_active && this.steps[this.index].close_corner) {
      approachOrigin = d;
      this.turn_approach_origin_cm = approachOrigin;
      this.turn_approach_cm = this.closeCornerApproachCm(d);
    }
    const current = this.currentAction();
    this.turn_approaching = !this.passing_junction &&
      !this.usesCornerArc() &&
      this.steps[this.index].action !== Action.HEXAGON &&
      current !== Action.STRAIGHT &&
      current !== Action.STOP &&
      (explicitApproach ||
        Math.abs(Math.abs(this.turnTargetRad()) - JUNCTION_TURN_TARGET_RAD) < 1e-4);
    this.resetEvidence(now);
    this.last_arc_ms = now;
    this.arc_just_exited = false;
    if (this.steps[this.index].action === Action.HEXAGON && !this.passing_junction) {
      this.hexagon = {
        state: HexagonState.HEXAGON_ENTERING,
        phase_ms: now,
        action_ms: now,
        line_ms: now,
        branch_ms: 0,
        center_ms: 0,
        phase_cm: d,
        line_cm: d,
        branch_cm: d,
        center_cm: 0,
        hold_heading: h,
        line_samples: 0,
        branch_samples: 0,
        center_samples: 0,
        line_seen: false,
        use_line: false,
        center_pending: false,
        exit_armed: false,
        target_heading: 0,
        turning: false,
        turn_index: 0,
        vertex_x_cm: 0,
        speed_cm_s: 0,
        vertex_y_cm: 0,
        leg_start_cm: 0,
        leg_from_x_cm: 0,
        leg_from_y_cm: 0,
        cross_cm: 0,
      };
    }
  }

  private acceptTarget(d: number, h: number, now: number): void {
    if (this.steps[this.index].action === Action.STOP) {
      const repeat = this.anotherLap();
      this.completed_laps++;
      if (!repeat) {
        this.state = RouteState.ROUTE_FINISHED;
        this.stop_pending = true;
        return;
      }
      this.lap_boundary = true;
    }
    this.beginAction(d, h, now);
  }

  private anchorNextSegment(d: number, now: number): void {
    this.next_segment_origin_cm = d;
    this.next_segment_ms = now;
    if (this.close_corner_tracking && this.index + 1 < this.count && this.steps[this.index + 1].close_corner) {
      this.next_segment_origin_cm -= this.short_link_forward_cm;
      this.close_corner_tracking = false;
    }
    this.next_segment_origin_valid = true;
    this.search_pending = false;
  }

  private startShortLinkTracking(d: number, h: number, now: number): void {
    this.short_link_active = false;
    this.short_link_tracking = !this.passing_junction && this.index + 1 < this.count &&
      shortLinkLength(this.steps[this.index + 1]) > 0 && ordinaryRightAngle(this.steps[this.index]);
    this.close_corner_tracking = this.pose_mode && !this.passing_junction && this.index + 1 < this.count &&
      !!this.steps[this.index + 1].close_corner && ordinaryRightAngle(this.steps[this.index]);
    if (!this.short_link_tracking && !this.close_corner_tracking) return;
    this.short_link_heading = this.entry_heading + this.turnTargetRad();
    this.short_link_forward_cm = this.short_link_lateral_cm = 0;
    this.short_link_last_cm = d; this.short_link_last_heading = h; this.short_link_ms = now;
  }

  private enterClear(d: number, now: number): void {
    if (this.short_link_tracking && !this.passing_junction && this.line_exit_confirmed &&
      this.index + 1 < this.count && shortLinkLength(this.steps[this.index + 1]) > 0) {
      /* Pose completion also sets line_exit_confirmed without grey evidence.
       * An explicit link must first pass the existing bounded real-exit search;
       * both grey exit paths supply LINE_CONFIRM_SAMPLES before entering here. */
      if (this.exit_count < LINE_CONFIRM_SAMPLES) {
        this.beginTurnExitSearch(d, now);
        return;
      }
      if (Math.abs(this.line_exit_heading_rad - this.short_link_heading) > SHORT_LINK_MAX_HEADING_RAD) {
        this.short_link_tracking = false;
      } else {
        // Previous lateral pivot motion includes its actual coast footprint.
        const early = this.index + 1 < this.count &&
          this.steps[this.index + 1].segment_length_cm === 145 ? SHORT_LINK_TURN_EARLY_CM : 0;
        this.short_link_pivot_lead_cm = Math.max(WHEEL_TRACK_CM * 0.5 - early, Math.abs(this.short_link_lateral_cm));
        if (this.short_link_pivot_lead_cm > SHORT_LINK_MAX_LATERAL_CM ||
          shortLinkLength(this.steps[this.index + 1]) - this.short_link_pivot_lead_cm - this.short_link_forward_cm < 1) {
          this.short_link_tracking = false; this.fail(RouteFailure.ROUTE_FAILURE_EXIT_RECAPTURE); return;
        }
        ++this.index; this.junctions_seen = 0;
        this.origin_cm = this.final_segment_origin_cm = d; this.segment_ms = now;
        this.beginAction(d, this.short_link_heading, now);
        this.short_link_active = true;
        const remaining = shortLinkLength(this.steps[this.index]) - this.short_link_pivot_lead_cm - this.short_link_forward_cm;
        this.turn_approach_origin_cm = d - (this.rowTurnApproachCm() - remaining);
        return;
      }
    }
    this.state = RouteState.ROUTE_CLEARING;
    this.post_turn_slow = this.pose_mode;
    this.phase_ms = now;
    this.clear_cm = d;
    this.exit_count = 0;
    this.exit_candidate_pending = false;
    this.turn_capturing = false;
    this.turn_ended_on_line = true;
    if (!this.passing_junction && this.currentAction() !== Action.STRAIGHT) {
      this.anchorNextSegment(d, now);
    }
    if (this.currentAction() === Action.STRAIGHT) {
      this.capture_line_ms = now;
      this.capture_line_cm = d;
    }
  }

  /** The estimated corner may drift when a real node authorizes a later turn.
   * Full angle ends the pivot, but never grants a short link without a real exit. */
  private beginTurnExitSearch(d: number, now: number): void {
    this.turn_exit_search = true;
    this.capture_line_ms = now;
    this.capture_line_cm = d;
    this.exit_component_extension_used = false;
    this.capture_line_samples = this.exit_count = 0;
    this.exit_candidate_pending = false;
    this.short_exit_edge_side = this.short_exit_edge_samples = 0;
    this.turn_capturing = this.capture_line_seen = this.line_exit_confirmed = false;
    this.turn_ended_on_line = false;
    this.turn_rewind_active = this.turn_rewind_pending = false;
    this.component_recovery_active = false;
  }

  private updateTurnExitSearch(
    o: GrayObservation,
    d: number,
    h: number,
    now: number,
    adjacent: boolean
  ): void {
    /* R25 wide component may cover the exit wire; extend the deadline once, without
     * moving the original time/distance baseline. */
    if (EXIT_COMPONENT_EXTEND_ENABLED && componentPattern(o) && !this.exit_component_extension_used &&
      this.index + 1 < this.count && this.steps[this.index + 1].segment_length_cm === 145) {
      this.exit_component_extension_used = true;
    }
    const limitMs = JUNCTION_EXIT_SEARCH_TIMEOUT_MS +
      (this.exit_component_extension_used ? COMPONENT_CROSS_MS : 0);
    const limitCm = JUNCTION_EXIT_SEARCH_MAX_CM +
      (this.exit_component_extension_used ? COMPONENT_CROSS_MAX_CM : 0);
    if (now - this.capture_line_ms >= limitMs ||
      Math.abs(d - this.capture_line_cm) >= limitCm) {
      this.fail(RouteFailure.ROUTE_FAILURE_EXIT_RECAPTURE);
      return;
    }
    if (this.short_link_tracking && this.index + 1 < this.count) {
      const lead = Math.max(WHEEL_TRACK_CM * 0.5, Math.abs(this.short_link_lateral_cm));
      if (lead > SHORT_LINK_MAX_LATERAL_CM ||
        shortLinkLength(this.steps[this.index + 1]) - lead - this.short_link_forward_cm < 1.0) {
        this.fail(RouteFailure.ROUTE_FAILURE_EXIT_RECAPTURE);
        return;
      }
    }
    /* On a short link a real narrow exit at position +3 may be all the room
     * there is; qualify by the outgoing heading and let enterClear charge the
     * search travel to the link. */
    const shortExit = this.short_link_tracking && this.index + 1 < this.count &&
      shortLinkLength(this.steps[this.index + 1]) > 0;
    const positionLimit = shortExit
      ? JUNCTION_SHORT_LINK_EXIT_POSITION_LIMIT
      : JUNCTION_CENTER_LIMIT;
    const narrowExit = adjacent && routeIsNarrowLine(o) &&
      o.white_count <= JUNCTION_STRAIGHT_EXIT_MAX_WIDTH &&
      (!shortExit || Math.abs(h - this.short_link_heading) <= SHORT_LINK_MAX_HEADING_RAD);
    const centered = narrowExit && Math.abs(o.position) <= positionLimit;
    if (!centered) this.exit_count = 0;
    else if (this.exit_count < LINE_CONFIRM_SAMPLES) this.exit_count++;
    this.capture_line_samples = this.exit_count;
    let confirmed = this.exit_count >= LINE_CONFIRM_SAMPLES;
    if (EXIT_COMPONENT_EXTEND_ENABLED && adjacent && componentPattern(o) &&
      this.index + 1 < this.count && this.steps[this.index + 1].segment_length_cm === 145 &&
      Math.abs(h - (this.entry_heading + this.turnTargetRad())) <= SHORT_LINK_MAX_HEADING_RAD) {
      confirmed = true;
      this.capture_line_samples = this.exit_count = LINE_CONFIRM_SAMPLES;
    }
    if (shortExit) {
      /* Q13 leaves only a brief, same-side edge window before the 12 cm
       * deadline. Keep the fast +/-3 exit; farther edges need both time and
       * travel, with every sample qualified by the outgoing heading above. */
      if (narrowExit && Number.isFinite(o.position) && Math.abs(o.position) > positionLimit) {
        const side = o.position < 0 ? 1 : 2;
        if (!this.exit_candidate_pending || this.short_exit_edge_side !== side) {
          this.exit_candidate_pending = true;
          this.exit_candidate_ms = now;
          this.exit_candidate_cm = d;
          this.short_exit_edge_side = side;
          this.short_exit_edge_samples = 0;
        }
        if (this.short_exit_edge_samples < LINE_CONFIRM_SAMPLES) this.short_exit_edge_samples++;
        if (this.short_exit_edge_samples >= LINE_CONFIRM_SAMPLES &&
          now - this.exit_candidate_ms >= JUNCTION_SHORT_LINK_EXIT_EDGE_STABLE_MS &&
          d - this.exit_candidate_cm >= JUNCTION_SHORT_LINK_EXIT_EDGE_STABLE_CM) {
          confirmed = true;
          /* enter_clear also requires a real multi-frame exit proof. */
          this.capture_line_samples = this.exit_count = LINE_CONFIRM_SAMPLES;
        }
      } else {
        this.exit_candidate_pending = false;
        this.short_exit_edge_side = this.short_exit_edge_samples = 0;
      }
    }
    if (!shortExit) {
      /* The ordinary exit can first appear near the end of the 12 cm search
       * (field: +5 -> +4 for 165 ms / 2.59 cm). Require a stable same-side
       * line aligned with the outgoing axis, then let clearance PID recenter.
       * 2026-11-10 实车（索引 9 → 环岛）：搜索只保持出口航向、冻结灰度位置 PID，
       * 枢轴留下的横向残差会让真正的出口线一直停在 8 路杆最外侧（0x03,
       * position=-6）。持续可见且与出口轴对齐的窄单区域线就是出口证据，交给
       * 离口位置 PID 回中；宽线/多区域/空心图形和单帧笔画仍不满足稳定窗口。 */
      const edge = narrowExit && Number.isFinite(o.position) && !centered &&
        Math.abs(h - (this.entry_heading + this.turnTargetRad())) <=
        JUNCTION_EXIT_SEARCH_EDGE_HEADING_RAD;
      if (edge && !confirmed) {
        const side = o.position < 0 ? 1 : 2;
        if (!this.exit_candidate_pending || this.short_exit_edge_side !== side) {
          this.exit_candidate_pending = true;
          this.exit_candidate_ms = now;
          this.exit_candidate_cm = d;
          this.short_exit_edge_side = side;
          this.short_exit_edge_samples = 0;
        }
        if (this.short_exit_edge_samples < LINE_CONFIRM_SAMPLES) this.short_exit_edge_samples++;
        if (this.short_exit_edge_samples >= LINE_CONFIRM_SAMPLES &&
          now - this.exit_candidate_ms >= JUNCTION_EXIT_SEARCH_EDGE_STABLE_MS &&
          d - this.exit_candidate_cm >= JUNCTION_EXIT_SEARCH_EDGE_STABLE_CM) {
          confirmed = true;
          this.capture_line_samples = this.exit_count = LINE_CONFIRM_SAMPLES;
        }
      } else {
        this.exit_candidate_pending = false;
        this.short_exit_edge_side = this.short_exit_edge_samples = 0;
      }
    }
    if (!confirmed) return;
    this.exit_candidate_pending = false;
    this.turn_exit_search = false;
    this.capture_line_seen = true;
    this.line_exit_confirmed = true;
    this.line_exit_heading_rad = h;
    this.enterClear(d, now);
  }

  private completeClear(d: number, now: number): boolean {
    const passed = this.passing_junction;
    if (passed) {
      this.passing_junction = false;
    } else {
      this.index = this.lap_boundary ? 0 : this.index + 1;
      this.lap_boundary = false;
      if (this.index >= this.count) {
        this.fail(RouteFailure.ROUTE_FAILURE_INVALID);
        return false;
      }
      this.junctions_seen = 0;
    }

    if (!passed) {
      this.origin_cm = this.next_segment_origin_valid
        ? this.next_segment_origin_cm
        : this.clear_cm;
      this.segment_ms = this.next_segment_origin_valid
        ? this.next_segment_ms
        : this.phase_ms;
      this.final_segment_origin_cm = this.origin_cm;
      this.search_pending =
        this.steps[this.index].segment_length_cm > 0 &&
        d - this.origin_cm >= this.steps[this.index].segment_length_cm;
      if (this.search_pending) {
        this.search_ms = now;
        this.search_origin_cm = d;
      }
    }

    this.final_segment_active = true;
    this.state = RouteState.ROUTE_FOLLOW;
    this.resetEvidence(now);
    return true;
  }

  private componentRecovery(
    o: GrayObservation,
    d: number,
    h: number,
    now: number,
    adjacent: boolean,
    eligible: boolean
  ): boolean {
    const graphic =
      o.white_count >= 2 &&
      (o.region_count > 1 || o.white_count > JUNCTION_STRAIGHT_EXIT_MAX_WIDTH);
    if (
      !this.component_recovery_active &&
      !this.component_recovery_used &&
      eligible &&
      graphic &&
      adjacent
    ) {
      this.component_recovery_active = this.component_recovery_used = true;
      this.component_recovery_ms = now;
      this.component_recovery_cm = d;
      this.component_recovery_heading =
        this.currentAction() === Action.STRAIGHT ? this.entry_heading : h;
      this.component_recovery_line_samples = 0;
      this.exit_count = 0;
      this.exit_candidate_pending = false;
      this.next_junction_samples = this.approach_center_count = 0;
    }
    if (!this.component_recovery_active) return false;
    this.turn_capturing = false;

    if (
      now - this.component_recovery_ms >= COMPONENT_RECOVERY_TIMEOUT_MS ||
      Math.abs(d - this.component_recovery_cm) >= COMPONENT_RECOVERY_MAX_CM
    ) {
      if (this.steps[this.index].action === Action.HEXAGON) {
        this.fail(RouteFailure.ROUTE_FAILURE_EXIT_RECAPTURE);
        return true;
      }
      this.component_recovery_active = false;
      return false;
    }

    const line =
      routeIsNarrowLine(o) &&
      o.white_count <= JUNCTION_STRAIGHT_EXIT_MAX_WIDTH &&
      Math.abs(o.position) <= JUNCTION_CENTER_LIMIT;
    if (!line || !adjacent) this.component_recovery_line_samples = 0;
    if (line && adjacent && this.component_recovery_line_samples < LINE_CONFIRM_SAMPLES) {
      this.component_recovery_line_samples++;
    }
    if (this.component_recovery_line_samples < LINE_CONFIRM_SAMPLES) return true;

    this.component_recovery_active = false;
    this.capture_line_ms = now;
    this.capture_line_cm = d;
    this.exit_component_extension_used = false;
    this.capture_line_samples = LINE_CONFIRM_SAMPLES;
    return false;
  }

  /** Pose-driven regular-hexagon traversal (2026-11-06).
   *
   * The ring has one external connection (right vertex V0), so a loop entered
   * from one side has to leave on the other side.  The table's HEXAGON angle
   * sign selects the direction (+ = clockwise) and the traversal uses the known
   * 35 cm edges / 60 deg vertices with odometry + IMU: waiting for the 24 cm
   * look-ahead bar to find the next vertex cannot work on 35 cm edges, and the
   * old tangent/branch exit assumption does not hold for this lollipop.
   * The 8-channel line still centres the car along each edge and confirms the
   * outgoing line after the exit turn. */
  private hexagonUpdate(
    o: GrayObservation,
    d: number,
    h: number,
    now: number,
    adjacent: boolean,
    axle_x_cm: number,
    axle_y_cm: number
  ): void {
    const x = this.hexagon;
    const dir = this.turnTargetRad() >= 0 ? 1 : -1;
    const h0 = this.entry_heading;
    const turnAngles = [
      HEXAGON_ENTRY_TURN_RAD, HEXAGON_TURN_RAD, HEXAGON_TURN_RAD,
      HEXAGON_TURN_RAD, HEXAGON_TURN_RAD, HEXAGON_TURN_RAD, HEXAGON_ENTRY_TURN_RAD,
    ];
    const cumulative: number[] = [];
    for (let i = 0; i < turnAngles.length; ++i) {
      cumulative.push((cumulative[i - 1] ?? 0) + turnAngles[i]);
    }
    const vertex = (i: number): { x: number; y: number } => {
      const k = dir > 0 ? (6 - (i % 6)) % 6 : i % 6;
      const ang = (-k * Math.PI) / 3;
      return {
        x: HEXAGON_CENTER_X_CM + HEXAGON_SIDE_CM * Math.cos(ang),
        y: HEXAGON_CENTER_Y_CM + HEXAGON_SIDE_CM * Math.sin(ang),
      };
    };
    const wrap = (a: number): number => Math.atan2(Math.sin(a), Math.cos(a));

    if (
      now - x.action_ms >= HEXAGON_ACTION_TIMEOUT_MS ||
      Math.abs(d - this.entry_cm) >= HEXAGON_ACTION_MAX_CM
    ) {
      this.fail(RouteFailure.ROUTE_FAILURE_ACTION_TIMEOUT);
      return;
    }
    // Direction / over-rotation guards: the polygon FSM bypasses the generic
    // EXECUTING checks, so it keeps them explicitly.
    const targetRad = Math.abs(this.turnTargetRad());
    const netTurn = (h - h0) * dir;
    if (netTurn < -JUNCTION_WRONG_WAY_RAD) {
      this.fail(RouteFailure.ROUTE_FAILURE_WRONG_WAY);
      return;
    }
    if (netTurn > targetRad + HEXAGON_OVERANGLE_RAD) {
      this.fail(RouteFailure.ROUTE_FAILURE_TURN_LIMIT);
      return;
    }

    const narrow = routeIsNarrowLine(o);
    const centered = narrow && Math.abs(o.position) <= JUNCTION_CENTER_LIMIT;
    // Only a completely blank bar is treated as true line loss inside the ring;
    // the printed R14 symbol and the vertex joins keep some channel lit.
    if (o.line_mask !== 0 && adjacent) {
      x.line_ms = now;
      x.line_cm = d;
    }

    // Ring speed plan (2026-11-09): pivots stay at the settle speed; straight
    // legs run at HEXAGON_LEG_SPEED_CM_S and brake toward the next vertex with
    // a_safe so each pivot still starts on its tangent point.
    {
      const ringStep = this.steps[this.index];
      const pivot = Math.min(HEXAGON_SPEED_CM_S, JUNCTION_SETTLE_SPEED_CM_S, ringStep.turn_speed);
      let ringSpeed = pivot;
      if (x.state === HexagonState.HEXAGON_FOLLOWING && !x.turning) {
        const leg = Math.min(HEXAGON_LEG_SPEED_CM_S, this.cruiseSpeed());
        const gap = Math.hypot(x.vertex_x_cm - axle_x_cm, x.vertex_y_cm - axle_y_cm) - HEXAGON_PIVOT_LEAD_CM;
        ringSpeed = Math.min(leg, Math.sqrt(pivot * pivot + 2 * BRAKING_SAFE_DECEL_CM_S2 * Math.max(0, gap)));
      }
      x.speed_cm_s = ringSpeed;
    }

    if (x.state === HexagonState.HEXAGON_ENTERING) {
      // The action starts when the 24 cm look-ahead bar reaches V0, so first
      // drive straight (V0 is ~24 cm ahead of the axle) and only pivot when the
      // axle is one turn-lead short of the vertex.
      const rawGap = Math.hypot(axle_x_cm - vertex(0).x, axle_y_cm - vertex(0).y);
      const entryLead = WHEEL_TRACK_CM * 0.5 * Math.tan(0.5 * turnAngles[0]);
      if (rawGap > entryLead + 0.6) {
        x.turning = false;
        x.use_line = narrow && centered;
        x.target_heading = h0;
        if (now - x.action_ms >= HEXAGON_ACTION_TIMEOUT_MS ||
          Math.abs(d - this.entry_cm) >= HEXAGON_ENTRY_APPROACH_MAX_CM) {
          this.fail(RouteFailure.ROUTE_FAILURE_ENTRY_SPEED);
        }
        return;
      }
      x.turning = true;
      x.turn_index = 0;
      x.target_heading = h0 + dir * turnAngles[0];
      x.vertex_x_cm = vertex(1).x;
      x.vertex_y_cm = vertex(1).y;
      x.leg_from_x_cm = vertex(0).x;
      x.leg_from_y_cm = vertex(0).y;
      x.state = HexagonState.HEXAGON_FOLLOWING;
      return;
    }

    if (x.state === HexagonState.HEXAGON_FOLLOWING) {
      if (x.turning) {
        x.use_line = false;
        if (Math.abs(wrap(x.target_heading - h)) <= HEXAGON_TURN_TOL_RAD) {
          x.turning = false;
          x.leg_start_cm = d;
          x.line_seen = true;
          x.line_ms = now;
          x.line_cm = d;
          if (x.turn_index >= turnAngles.length - 1) {
            // Pose-mode ring exit: the axle is at V0 and already turned onto the
            // external line, so hand over directly instead of waiting for a
            // centred grey line whose bar may still be inside the ring.
            x.state = HexagonState.HEXAGON_DONE;
            x.use_line = false;
            this.line_exit_confirmed = true;
            this.line_exit_heading_rad = h;
            this.enterClear(d, now);
            return;
          } else {
            const from = vertex(x.turn_index);
            const v = vertex(x.turn_index + 1);
            x.leg_from_x_cm = from.x;
            x.leg_from_y_cm = from.y;
            x.vertex_x_cm = v.x;
            x.vertex_y_cm = v.y;
          }
        }
        return;
      }
      const nextAngle = Math.abs(turnAngles[x.turn_index + 1]);
      const lead = WHEEL_TRACK_CM * 0.5 * Math.tan(0.5 * nextAngle);
      const gap = Math.hypot(x.vertex_x_cm - axle_x_cm, x.vertex_y_cm - axle_y_cm);
      if (gap <= lead + (Number.isFinite(axle_x_cm) ? 0.6 : 0)) {
        ++x.turn_index;
        x.turning = true;
        x.target_heading = h0 + dir * cumulative[x.turn_index];
        x.use_line = false;
        return;
      }
      x.use_line = centered || (narrow && Math.abs(o.position) <= JUNCTION_CENTER_LIMIT + 1);
      // Blind stretch: the 24 cm bar leaves a 35 cm edge long before the axle
      // reaches the vertex, so bound the leg by the pose cross-track instead of
      // the 400 ms grey-loss timer.
      const ldx = x.vertex_x_cm - x.leg_from_x_cm;
      const ldy = x.vertex_y_cm - x.leg_from_y_cm;
      const llen = Math.hypot(ldx, ldy);
      if (llen > 1e-3) {
        x.cross_cm = (-(axle_x_cm - x.leg_from_x_cm) * ldy +
          (axle_y_cm - x.leg_from_y_cm) * ldx) / llen;
      } else {
        x.cross_cm = 0;
      }
      if (Number.isFinite(axle_x_cm) && Math.abs(x.cross_cm) > HEXAGON_LEG_CROSS_MAX_CM) {
        this.fail(RouteFailure.ROUTE_FAILURE_EXIT_LOST);
        return;
      }
      return;
    }

    if (x.state === HexagonState.HEXAGON_EXITING) {
      x.use_line = centered;
      if (
        now - x.phase_ms >= HEXAGON_EXIT_TIMEOUT_MS ||
        Math.abs(d - x.phase_cm) >= HEXAGON_EXIT_MAX_CM
      ) {
        this.fail(RouteFailure.ROUTE_FAILURE_EXIT_LOST);
        return;
      }
      if (!centered) {
        x.center_samples = 0;
        x.center_pending = false;
      } else {
        if (!x.center_pending) {
          x.center_pending = true;
          x.center_ms = now;
          x.center_cm = d;
        }
        if (x.center_samples < LINE_CONFIRM_SAMPLES) ++x.center_samples;
      }
      if (
        x.center_samples >= LINE_CONFIRM_SAMPLES &&
        now - x.center_ms >= HEXAGON_EXIT_CONFIRM_MS &&
        d - x.center_cm >= HEXAGON_EXIT_CONFIRM_CM
      ) {
        x.state = HexagonState.HEXAGON_DONE;
        this.line_exit_confirmed = true;
        this.line_exit_heading_rad = h;
        this.enterClear(d, now);
      }
      return;
    }
  }

  /* Rolling corner geometry.  The corner waypoint K and the incoming segment
   * direction u give the arc that is tangent to both legs:
   *   entry tangent  E = K - R*u
   *   exit tangent   X = K + R*o      (o = u rotated by +-90 deg, the turn side)
   *   centre         C = K - R*u + R*o
   * The axle tracks that circle; the heading target is the circle tangent, so
   * the loop is a real path-following loop (the target depends on the pose)
   * and cannot wind up like a pure heading bang-bang pivot. */
  private cornerArcUpdate(h: number, px: number, py: number, dt: number): void {
    const dir = this.turnTargetRad() < 0 ? -1 : 1;
    const R = CORNER_ARC_RADIUS_CM;
    const track = WHEEL_TRACK_CM;
    const Kx = this.waypoint_x_cm[this.index];
    const Ky = this.waypoint_y_cm[this.index];
    const Px = this.index > 0 ? this.waypoint_x_cm[this.index - 1] : this.start_x_cm;
    const Py = this.index > 0 ? this.waypoint_y_cm[this.index - 1] : this.start_y_cm;
    let ux = Kx - Px;
    let uy = Ky - Py;
    const len = Math.hypot(ux, uy);
    if (len > 1e-3) {
      ux /= len;
      uy /= len;
    } else {
      const w = this.entry_heading - Math.PI / 2;
      ux = Math.cos(w);
      uy = Math.sin(w);
    }
    // Turn-side normal: rotating u by dir*90 degrees.
    const ox = dir > 0 ? -uy : uy;
    const oy = dir > 0 ? ux : -ux;
    const Cx = Kx - R * ux + R * ox;
    const Cy = Ky - R * uy + R * oy;
    const Ex = Kx - R * ux;
    const Ey = Ky - R * uy;
    const Xx = Kx + R * ox;
    const Xy = Ky + R * oy;
    // Roll-up distance to the entry tangent point along the incoming leg, and
    // the lateral offset from that leg (a badly-off entry must not arm the arc
    // geometry blindly).
    const proj = (Kx - px) * ux + (Ky - py) * uy;
    this.arc_roll_cm = proj - R;
    this.arc_lateral_cm = (px - Ex) * ox + (py - Ey) * oy;
    if (!this.arc_active &&
      proj <= R + CORNER_ARC_ENTER_TOL_CM &&
      Math.abs(this.arc_lateral_cm) <= CORNER_ARC_EXIT_TOL_CM * 4.0) {
      this.arc_active = true;
      this.arc_last_heading = h;
      this.arc_yaw_rate_rad_s = 0;
    }
    const rx = px - Cx;
    const ry = py - Cy;
    const alpha = Math.atan2(ry, rx);
    const alpha_in = Math.atan2(Ey - Cy, Ex - Cx);
    this.arc_radius_err_cm = Math.hypot(rx, ry) - R;
    const dphi = wrapRad(alpha - alpha_in) * dir;
    this.arc_progress_rad = Math.max(0, Math.min(Math.PI / 2, dphi));
    const segHeading = Math.atan2(uy, ux) + Math.PI / 2;
    this.arc_desired_heading_rad = this.arc_active
      ? alpha + dir * (Math.PI / 2) + Math.PI / 2
      : segHeading;
    const v = CORNER_ARC_SPEED_CM_S;
    this.arc_outer_cm_s = this.arc_active ? v * (1 + track / (2 * R)) : v;
    const diffTarget = this.arc_active ? (v * track) / (2 * R) : 0;
    const step = CORNER_ARC_DIFF_SLEW_CM_S2 * dt;
    if (diffTarget > this.arc_diff_slew_cm_s) {
      this.arc_diff_slew_cm_s = Math.min(diffTarget, this.arc_diff_slew_cm_s + step);
    } else {
      this.arc_diff_slew_cm_s = Math.max(diffTarget, this.arc_diff_slew_cm_s - step);
    }
    this.arc_diff_cm_s = this.arc_diff_slew_cm_s;
    const target = Math.abs(this.turnTargetRad());
    // Exit contract in the outgoing leg's frame (o = along, u = across, since u
    // and o are perpendicular for a right angle): the axle must have reached the
    // exit tangent point, sit on the outgoing wire, and point along the outgoing
    // leg.  The heading test is absolute (not "turned target radians since the
    // action began") because the entry heading itself carries a couple of
    // degrees from the previous corner.
    const along = (px - Kx) * ox + (py - Ky) * oy;
    const across = (px - Kx) * ux + (py - Ky) * uy;
    this.arc_along_cm = along;
    this.arc_across_cm = across;
    const outHeading = Math.atan2(oy, ox) + Math.PI / 2;
    const headingErr = wrapRad(h - outHeading);
    this.arc_heading_err_rad = headingErr;
    // Measured yaw rate: the arc releases its differential at the hand-off and
    // the car keeps rotating while the wheels unwind, so the hand-off must lead
    // the target heading by that pending rotation.
    if (dt > 0) {
      const raw = wrapRad(h - this.arc_last_heading) / dt;
      const a = dt / (JUNCTION_YAW_FILTER_TAU_S + dt);
      this.arc_yaw_rate_rad_s += a * (raw - this.arc_yaw_rate_rad_s);
    }
    this.arc_last_heading = h;
    this.arc_exit_lead_rad = this.arc_yaw_rate_rad_s * CORNER_ARC_EXIT_LEAD_S;
    const predicted = wrapRad(headingErr + this.arc_exit_lead_rad);
    if (this.arc_active &&
      Math.abs(predicted) <= CORNER_ARC_EXIT_ANGLE_RAD &&
      along >= R - CORNER_ARC_EXIT_ALONG_TOL_CM &&
      Math.abs(across) <= CORNER_ARC_EXIT_TOL_CM) {
      this.corner_arc_complete = true;
    }
    void target;
  }

  public recoverNavigation(d: number, h: number, now: number): boolean {
    if (this.state !== RouteState.ROUTE_FAILED) return false;
    switch (this.failure) {
      case RouteFailure.ROUTE_FAILURE_CONFIRM_TIMEOUT:
      case RouteFailure.ROUTE_FAILURE_ACTION_TIMEOUT:
      case RouteFailure.ROUTE_FAILURE_TURN_LIMIT:
      case RouteFailure.ROUTE_FAILURE_WRONG_WAY:
      case RouteFailure.ROUTE_FAILURE_SEARCH:
      case RouteFailure.ROUTE_FAILURE_WINDOW:
      case RouteFailure.ROUTE_FAILURE_ENTRY_SPEED:
        break;
      default:
        return false;
    }
    this.short_link_tracking = this.short_link_active = this.turn_approaching = false;
    this.close_corner_tracking = false;
    this.bar_hold_state = RouteBarHoldState.ROUTE_BAR_HOLD_NONE;
    this.bar_hold_clear_count = 0;
    this.navigation_warning = this.failure;
    this.navigation_recoveries++;
    this.failure = RouteFailure.ROUTE_FAILURE_NONE;
    this.resetEvidence(now);
    this.state = RouteState.ROUTE_RECOVERING;
    this.phase_ms = now;
    this.clear_cm = d;
    this.entry_heading = h;
    this.capture_line_ms = now;
    this.capture_line_cm = d;
    return true;
  }

  public update(
    o: GrayObservation,
    d: number,
    h: number,
    now: number,
    axle_x_cm = NaN,
    axle_y_cm = NaN
  ): void {
    if (
      this.state === RouteState.ROUTE_FINISHED ||
      this.state === RouteState.ROUTE_FAILED ||
      this.state === RouteState.ROUTE_DISABLED
    ) {
      return;
    }

    if (
      !this.valid ||
      this.index >= this.count ||
      !Number.isFinite(d) ||
      !Number.isFinite(h) ||
      !Number.isFinite(this.measured_left_cm_s) ||
      !Number.isFinite(this.measured_right_cm_s)
    ) {
      this.fail(RouteFailure.ROUTE_FAILURE_INVALID);
      return;
    }

    // Pose mode needs both the caller's axle pose and the waypoint chain that
    // init() derives from the table; hand-built test planners stay on the grey
    // contract.
    this.pose_mode = Number.isFinite(axle_x_cm) && Number.isFinite(axle_y_cm) &&
      this.pose_enabled && this.count > 0 && this.waypoint_x_cm.length === this.count;
    if (this.pose_mode && this.waypoint_x_cm.length === this.count) {
      /* h is the mileage heading relative to START (START aligns 0), so the
       * world heading is start_heading + h.  The old code hard-coded h - PI/2
       * (the production P-north anchor); a scenario that starts southward put
       * the 24 cm bar behind the car and flipped every segment heading by 180°. */
      const wh = h + this.start_heading_rad;
      const sx0 = axle_x_cm + GRAY_FORWARD_OFFSET_CM * Math.cos(wh);
      const sy0 = axle_y_cm + GRAY_FORWARD_OFFSET_CM * Math.sin(wh);
      this.pose_gap_bar_cm = Math.hypot(
        sx0 - this.waypoint_x_cm[this.index],
        sy0 - this.waypoint_y_cm[this.index]
      );
      this.pose_bearing_rad =
        Math.atan2(this.waypoint_y_cm[this.index] - axle_y_cm,
          this.waypoint_x_cm[this.index] - axle_x_cm) - this.start_heading_rad;
      const px = this.index > 0 ? this.waypoint_x_cm[this.index - 1] : this.start_x_cm;
      const py = this.index > 0 ? this.waypoint_y_cm[this.index - 1] : this.start_y_cm;
      const dx = this.waypoint_x_cm[this.index] - px;
      const dy = this.waypoint_y_cm[this.index] - py;
      const len = Math.hypot(dx, dy);
      if (len > 1e-3) {
        // right-of-travel unit vector = (-dy, dx)/len (x east, y south)
        this.pose_cross_cm =
          (-(axle_x_cm - px) * dy + (axle_y_cm - py) * dx) / len;
        // The bar sits 24 cm ahead; a heading error shifts its lateral offset.
        const segH = Math.atan2(dy, dx);
        this.pose_cross_bar_cm =
          this.pose_cross_cm + GRAY_FORWARD_OFFSET_CM * Math.sin(wh - segH);
        // World direction -> mileage-relative heading (0 = start heading).
        this.pose_seg_heading_rad = segH - this.start_heading_rad;
        this.pose_cross_valid = true;
      } else {
        // Degenerate (zero-length) row: there is no segment line, so hold the
        // row's entry heading instead of bearing back to the corner behind us.
        this.pose_cross_valid = false;
        this.pose_cross_cm = 0;
        this.pose_bearing_rad = this.entry_heading;
        this.pose_seg_heading_rad = this.entry_heading;
      }
    } else {
      this.pose_gap_bar_cm = Infinity;
    }
    const s = this.steps[this.index];
    this.updateBarHold(o, d, h, now);
    if ((this.state as RouteState) === RouteState.ROUTE_FAILED) return;
    if (this.short_link_tracking || this.close_corner_tracking) {
      const ds = d - this.short_link_last_cm, dh = h - this.short_link_last_heading;
      const half = 0.5 * dh;
      const chord = Math.abs(half) < 0.0001 ? 1 : Math.sin(half) / half;
      const angle = 0.5 * (h + this.short_link_last_heading) - this.short_link_heading;
      this.short_link_forward_cm += ds * chord * Math.cos(angle);
      this.short_link_lateral_cm += ds * chord * Math.sin(angle);
      this.short_link_last_cm = d; this.short_link_last_heading = h;
      if (this.short_link_active) {
        if (now - this.short_link_ms >= SHORT_LINK_TIMEOUT_MS || now - this.last_sample_ms > CONTROL_MAX_INTERVAL_MS ||
          Math.abs(this.short_link_lateral_cm) > SHORT_LINK_MAX_LATERAL_CM ||
          Math.abs(h - this.short_link_heading) > SHORT_LINK_MAX_HEADING_RAD ||
          ds < -0.01 || this.short_link_forward_cm > shortLinkLength(s) + 2) {
          this.short_link_tracking = this.short_link_active = false;
          this.fail(RouteFailure.ROUTE_FAILURE_EXIT_RECAPTURE); return;
        }
        const remaining = shortLinkLength(s) - this.short_link_pivot_lead_cm - this.short_link_forward_cm;
        this.turn_approach_origin_cm = d - (this.rowTurnApproachCm() - remaining);
      }
    }
    if (this.state === RouteState.ROUTE_RECOVERING) {
      if (
        now - this.capture_line_ms >= NAVIGATION_RECOVERY_MS ||
        Math.abs(d - this.capture_line_cm) >= NAVIGATION_RECOVERY_CM
      ) {
        this.fail(RouteFailure.ROUTE_FAILURE_EXIT_LOST);
        return;
      }
      const narrow = routeIsNarrowLine(o);
      const continuous = now - this.last_sample_ms <= CONTROL_MAX_INTERVAL_MS;
      this.last_sample_ms = now;

      if (!narrow || !continuous) {
        this.exit_count = 0;
        this.exit_candidate_pending = false;
      }
      if (narrow && !this.exit_candidate_pending) {
        this.exit_candidate_pending = true;
        this.exit_candidate_ms = now;
        this.exit_candidate_cm = d;
      }
      if (narrow && this.exit_count < LINE_CONFIRM_SAMPLES) this.exit_count++;
      if (narrow && this.exit_count >= LINE_CONFIRM_SAMPLES) {
        this.capture_line_ms = now;
        this.capture_line_cm = d;
      }
      if (
        this.exit_count >= LINE_CONFIRM_SAMPLES &&
        now - this.exit_candidate_ms >= JUNCTION_COUNT_CLEAR_MS &&
        d - this.exit_candidate_cm >= JUNCTION_COUNT_CLEAR_CM
      ) {
        this.passing_junction = false;
        this.state = RouteState.ROUTE_FOLLOW;
        this.resetEvidence(now);
        return;
      }
      return;
    }

    const adjacent = now - this.last_sample_ms <= CONTROL_MAX_INTERVAL_MS;
    this.last_sample_ms = now;

    if (
      this.state === RouteState.ROUTE_FOLLOW ||
      this.state === RouteState.ROUTE_CANDIDATE ||
      this.state === RouteState.ROUTE_CLEARING
    ) {
      this.refreshDistance(d, now);
    }

    if (this.state === RouteState.ROUTE_FOLLOW || this.state === RouteState.ROUTE_CANDIDATE) {
      const narrow = routeIsNarrowLine(o);
      const center = narrow && Math.abs(o.position) <= JUNCTION_CENTER_LIMIT;
      if (this.post_turn_slow && narrow && center && adjacent) {
        if (++this.post_turn_samples >= LINE_CONFIRM_SAMPLES) this.post_turn_slow = false;
      } else if (this.post_turn_slow && (!narrow || !center)) {
        this.post_turn_samples = 0;
      }

      if (
        adjacent &&
        narrow &&
        o.white_count <= JUNCTION_EDGE_MAX_WIDTH &&
        !center &&
        this.previous_line_valid &&
        (this.previous_line_mask & o.line_mask) &&
        Math.abs(o.position - this.previous_line_position) <= JUNCTION_SMOOTH_POSITION_STEP &&
        Math.abs(o.position) > Math.abs(this.previous_line_position)
      ) {
        this.smooth_edge_sides |= o.position < 0 ? 1 : 2;
      }

      this.previous_line_valid =
        adjacent && narrow && o.white_count <= JUNCTION_EDGE_MAX_WIDTH;
      this.previous_line_mask = o.line_mask;
      this.previous_line_position = o.position;

      if (center) {
        if (!adjacent) this.approach_center_count = 0;
        if (this.approach_center_count < LINE_CONFIRM_SAMPLES) this.approach_center_count++;
        if (this.approach_center_count >= LINE_CONFIRM_SAMPLES) {
          this.have_approach_center = true;
          this.approach_center_ms = now;
          this.approach_center_cm = d;
          this.smooth_edge_sides = 0;
        }
      } else {
        this.approach_center_count = 0;
      }

      const recent_center =
        this.have_approach_center &&
        now - this.approach_center_ms < JUNCTION_CENTER_MEMORY_MS &&
        d >= this.approach_center_cm &&
        d - this.approach_center_cm <= JUNCTION_CENTER_MEMORY_CM;

      let history = (this.state === RouteState.ROUTE_CANDIDATE && d >= this.entry_cm) || recent_center;
      let { kind, sides } = junctionShape(o);
      if (s.dashed_row) {
        if (o.white_count < DASHED_NODE_MIN_WHITE) { kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE; sides = 0; }
        else if (kind !== JunctionEvidence.JUNCTION_EVIDENCE_NONE) history = true;
      }
      // A gradually shifting single line can still include a center channel
      // (e.g. 0x38 -> 0x30 -> 0x70). Keep its same-side suppression when
      // classified CENTER_BRANCH; an abrupt or separated branch stays eligible.
      const smooth =
        (kind === JunctionEvidence.JUNCTION_EVIDENCE_EDGE ||
          (kind === JunctionEvidence.JUNCTION_EVIDENCE_CENTER_BRANCH &&
            o.white_count <= JUNCTION_EDGE_MAX_WIDTH)) &&
        (sides & this.smooth_edge_sides) !== 0;
      const shape = history && kind !== JunctionEvidence.JUNCTION_EVIDENCE_NONE && !smooth;

      if (!shape || !adjacent) {
        this.candidate_reject = !adjacent
          ? JunctionReject.JUNCTION_REJECT_SAMPLE_GAP
          : smooth
          ? JunctionReject.JUNCTION_REJECT_SMOOTH_EDGE
          : !history
          ? JunctionReject.JUNCTION_REJECT_HISTORY
          : JunctionReject.JUNCTION_REJECT_SHAPE;

        if (smooth) this.smooth_edge_rejections++;
        this.unbindBarHold();
        this.candidate_count = 0;
        this.candidate_separated = false;
        this.candidate_kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE;
        this.candidate_sides = 0;
        this.state = RouteState.ROUTE_FOLLOW;
        return;
      }

      this.candidate_reject = JunctionReject.JUNCTION_REJECT_NONE;

      if (this.state === RouteState.ROUTE_CANDIDATE && !(this.candidate_sides & sides)) {
        this.unbindBarHold();
        this.candidate_restarts++;
        this.candidate_reject = JunctionReject.JUNCTION_REJECT_SIDE_CHANGE;
        this.candidate_count = 0;
        this.candidate_separated = false;
        this.state = RouteState.ROUTE_FOLLOW;
        this.candidate_kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE;
        this.candidate_sides = 0;
        if (!recent_center) return;
      }

      const same_sides =
        this.state === RouteState.ROUTE_CANDIDATE && this.candidate_sides === sides;

      if (this.state === RouteState.ROUTE_FOLLOW) {
        this.state = RouteState.ROUTE_CANDIDATE;
        this.phase_ms = now;
        this.entry_cm = d;
        this.entry_heading = h;
        this.candidate_count = 0;
        this.candidate_kind = kind;
        this.candidate_sides = sides;
      } else {
        this.candidate_sides &= sides;
        if (kind !== JunctionEvidence.JUNCTION_EVIDENCE_CENTER_BRANCH) {
          this.candidate_kind = kind;
        }
      }

      if (now - this.phase_ms >= JUNCTION_CONFIRM_TIMEOUT_MS) {
        this.fail(RouteFailure.ROUTE_FAILURE_CONFIRM_TIMEOUT);
        return;
      }

      const weak =
        this.candidate_kind !== JunctionEvidence.JUNCTION_EVIDENCE_CENTER_BRANCH &&
        !this.candidate_separated;
      const early =
        same_sides &&
        this.candidate_kind === JunctionEvidence.JUNCTION_EVIDENCE_CENTER_BRANCH &&
        this.targetReady(d, true) &&
        ((s.action === Action.LEFT && sides === 1) || (s.action === Action.RIGHT && sides === 2));

      const samples = early
        ? JUNCTION_EARLY_CONFIRM_SAMPLES
        : weak
        ? JUNCTION_EDGE_CONFIRM_SAMPLES
        : JUNCTION_CONFIRM_SAMPLES;

      const distance = early
        ? JUNCTION_EARLY_CONFIRM_CM
        : weak
        ? JUNCTION_EDGE_CONFIRM_CM
        : JUNCTION_CONFIRM_DISTANCE_CM;

      if (this.candidate_count < samples) this.candidate_count++;
      if (this.candidate_count < samples || d - this.entry_cm < distance) return;

      this.last_confirmed_kind = this.candidate_kind;
      this.junctions_seen++;
      this.passing_junction = !this.targetReady(d, false) && !this.barMarksCorner();
      if (this.passing_junction) {
        this.unbindBarHold();
      } else if (this.bar_hold_state === RouteBarHoldState.ROUTE_BAR_HOLD_PENDING &&
        this.bar_hold_index === this.index) {
        this.bar_hold_state = RouteBarHoldState.ROUTE_BAR_HOLD_TARGET;
      } else if (this.bar_hold_state !== RouteBarHoldState.ROUTE_BAR_HOLD_TARGET) {
        this.bar_hold_state = RouteBarHoldState.ROUTE_BAR_HOLD_NONE;
      }
      this.search_pending = false;

      if (this.passing_junction) {
        this.beginAction(d, h, now);
        this.enterClear(d, now);
        return;
      }

      if (this.currentAction() !== Action.STRAIGHT && !this.entrySpeedReady()) {
        this.state = RouteState.ROUTE_BRAKING;
        this.phase_ms = now;
        this.entry_cm = d;
        this.entry_heading = h;
        return;
      }

      this.acceptTarget(d, h, now);
      return;
    }

    if (this.state === RouteState.ROUTE_BRAKING) {
      if (
        now - this.phase_ms >= JUNCTION_ENTRY_BRAKE_TIMEOUT_MS ||
        Math.abs(d - this.entry_cm) > JUNCTION_ENTRY_BRAKE_MAX_CM ||
        Math.abs(h - this.entry_heading) > JUNCTION_WRONG_WAY_RAD
      ) {
        this.fail(RouteFailure.ROUTE_FAILURE_ENTRY_SPEED);
        return;
      }
      // Pose mode: at the bottom-right corner the bar is already off the board,
      // so a blank reading cannot abort the entry; the pose trigger is the node.
      if (!this.short_link_active && !this.pose_mode && o.line_mask === 0 &&
        now - this.phase_ms >= INTERFERENCE_TIMEOUT_MS) {
        this.fail(RouteFailure.ROUTE_FAILURE_ENTRY_SPEED); return;
      }
      if (this.entrySpeedReady() && (o.line_mask !== 0 || this.short_link_active || this.pose_mode)) {
        if (this.short_link_active) {
          this.state = RouteState.ROUTE_EXECUTING; this.phase_ms = now; this.entry_cm = d;
          this.turn_approaching = false; this.resetEvidence(now);
          this.startShortLinkTracking(d, h, now); return;
        }
        this.acceptTarget(d, h, now);
      }
      return;
    }

    if (s.action === Action.HEXAGON && !this.passing_junction && this.state === RouteState.ROUTE_EXECUTING) {
      this.hexagonUpdate(o, d, h, now, adjacent, axle_x_cm, axle_y_cm);
      return;
    }

    if (this.state === RouteState.ROUTE_EXECUTING && this.turn_approaching) {
      if (now - this.phase_ms >= JUNCTION_APPROACH_TIMEOUT_MS) {
        this.fail(RouteFailure.ROUTE_FAILURE_ACTION_TIMEOUT); return;
      }
      // Explicit row distances count from grey confirmation, including braking.
      // Zero-valued rows retain the legacy front-bar/axle approach.
      if (d - this.turn_approach_origin_cm < this.turn_approach_cm) return;
      if (this.short_link_active && !this.entrySpeedReady()) {
        this.state = RouteState.ROUTE_BRAKING; this.phase_ms = now; this.entry_cm = d; return;
      }
      this.turn_approaching = false;
      this.bar_hold_state = RouteBarHoldState.ROUTE_BAR_HOLD_NONE;
      // Approach wheel inertia is part of this action's accumulated yaw.
      this.phase_ms = now;
      this.entry_cm = d;
      this.resetEvidence(now);
      this.startShortLinkTracking(d, h, now);
    }
    const action = this.currentAction();
    // Angle convention: clockwise positive, so the direction comes from the
    // configured angle (a negative CUSTOM_TURN is a left turn).
    const sign = this.turnTargetRad() < 0 ? -1 : 1;
    const turn = (h - this.entry_heading) * sign;
    const target = Math.abs(this.turnTargetRad());

    // Pose exit (2026-11-06): a short leg's outgoing line can run out under the
    // 24 cm look-ahead bar a few cm after the pivot, so the grey-only contract
    // would time out.  When the axle has reached the pivot angle at the known
    // corner, hand over to the next row immediately.
    /* Firmware working tree (2026-11-10): the pose hand-off is a one-shot
     * EXECUTING transition. Re-entering from CLEARING refreshed its deadline
     * and moved the next segment origin every frame while the axle stayed
     * inside the corner exit radius. */
    const pose_available = this.state === RouteState.ROUTE_EXECUTING &&
      this.pose_mode &&
      action !== Action.STRAIGHT &&
      !this.passing_junction &&
      !this.turn_exit_search &&
      Number.isFinite(axle_x_cm) &&
      Number.isFinite(axle_y_cm) &&
      this.waypoint_x_cm.length === this.count;
    const arc_row = pose_available && this.usesCornerArc();
    if (arc_row && this.state === RouteState.ROUTE_EXECUTING) {
      const arc_dt = Math.max(0.001, Math.min(0.020, (now - this.last_arc_ms) / 1000));
      this.cornerArcUpdate(h, axle_x_cm, axle_y_cm, arc_dt);
      this.last_arc_ms = now;
    }
    if (pose_available) {
      if (arc_row) {
        // Rolling corner: leave on the arc's own tangent, not on the legacy
        // "within 14 cm of the corner" hand-off, which fires ~7 degrees early.
        // Only in EXECUTING: once CLEARING starts the exit flag stays set, and
        // re-entering CLEARING every frame would freeze its clear timer.
        if (this.state === RouteState.ROUTE_EXECUTING && this.corner_arc_complete) {
          this.line_exit_confirmed = true;
          this.line_exit_heading_rad = h;
          this.arc_just_exited = true;
          this.enterClear(d, now);
          return;
        }
      } else {
        const gap = Math.hypot(axle_x_cm - this.waypoint_x_cm[this.index], axle_y_cm - this.waypoint_y_cm[this.index]);
        /* 2026-11-09 现场：这个“轴距角点 14 cm + 差 0.12 rad 就交棒”的提前退出只对
         * 普通直角成立；非 90° 自定义角没有离口位置 PID 补偿，枢轴在角点前 5 cm 起转
         * 后车轴一直在 14 cm 窗口内，旧条件会在 target-0.12 rad（设 62° 只转 55°）
         * 就交出，表现为“转弯达不到设定角度”。 */
        const poseOrdinaryExit = this.steps[this.index].action !== Action.CUSTOM_TURN ||
          Math.abs(Math.abs(this.turnTargetRad()) - JUNCTION_TURN_TARGET_RAD) < 1e-4;
        /* 直角沿用 0.12 rad 余量；非 90° 自定义角必须真正转到设定角才交棒（否则少 ~7°），
         * 但也不能等灰度出口线（枢轴期间 mask=0 会一直拧到 EXIT_LOST）。 */
        const poseExitTolerance = poseOrdinaryExit ? ROUTE_WAYPOINT_EXIT_ANGLE_RAD : 0.02;
        if (gap <= ROUTE_WAYPOINT_EXIT_CM && turn >= target - poseExitTolerance) {
          this.line_exit_confirmed = true;
          this.line_exit_heading_rad = h;
          this.enterClear(d, now);
          return;
        }
      }
    }

    if (
      action !== Action.STRAIGHT &&
      (turn < -JUNCTION_WRONG_WAY_RAD ||
        turn > this.turnThresholdRad(JUNCTION_TURN_MAX_RAD))
    ) {
      this.fail(
        turn < 0
          ? RouteFailure.ROUTE_FAILURE_WRONG_WAY
          : RouteFailure.ROUTE_FAILURE_TURN_LIMIT
      );
      return;
    }

    if (this.state === RouteState.ROUTE_EXECUTING) {
      if (this.turn_exit_search) {
        this.updateTurnExitSearch(o, d, h, now, adjacent);
        return;
      }
      if (now - this.phase_ms >= this.actionTimeoutMs()) {
        this.fail(RouteFailure.ROUTE_FAILURE_ACTION_TIMEOUT);
        return;
      }
      if (this.pose_mode && !this.passing_junction &&
        ordinaryRightAngle(s) && turn >= target) {
        this.beginTurnExitSearch(d, now);
        return;
      }

      if (action === Action.STRAIGHT) {
        const exit = routeIsClearLine(this, o);
        if (!exit || !adjacent) this.exit_count = 0;
        if (exit && this.exit_count < LINE_CONFIRM_SAMPLES) this.exit_count++;
        if (this.exit_count >= LINE_CONFIRM_SAMPLES) {
          this.line_exit_confirmed = true;
          this.line_exit_heading_rad = h;
          this.enterClear(d, now);
          if (!this.passing_junction) this.anchorNextSegment(d, now);
        }
        return;
      }

      const narrow = routeIsNarrowLine(o);
      if (!this.turn_rewind_active && ordinaryRightAngle(s) && !this.short_link_active) {
        if (!narrow && turn >= target - 0.03 && adjacent) {
          if (!this.turn_rewind_pending) { this.turn_rewind_pending = true; this.turn_rewind_ms = now; }
          if (now - this.turn_rewind_ms >= 150) this.turn_rewind_active = true;
        } else this.turn_rewind_pending = false;
      }
      if (
        this.componentRecovery(
          o,
          d,
          h,
          now,
          adjacent,
          !narrow &&
            this.capture_line_seen &&
            turn >= target * (1.0 - JUNCTION_EXIT_ANGLE_MARGIN_RATIO)
        )
      ) {
        return;
      }

      // 30-40 cm short legs: the 24 cm lookahead crosses the next node mid-turn.
      // Record centre-bearing branch evidence seen after 45 degrees so it can
      // seed the next adjacent handoff without re-counting this corner.
      if (!this.passing_junction && turn >= 0.8) {
        const tshape = junctionShape(o);
        if (tshape.kind !== JunctionEvidence.JUNCTION_EVIDENCE_NONE && tshape.sides !== 0) {
          if (this.turn_branch_samples === 0) this.turn_branch_sides = tshape.sides;
          else this.turn_branch_sides &= tshape.sides;
          if (this.turn_branch_samples < 255) this.turn_branch_samples++;
        }
      }
      this.turn_capturing =
        narrow && turn >= this.turnThresholdRad(JUNCTION_CAPTURE_MIN_RAD);
      if (this.turn_capturing) {
        if (!adjacent) this.capture_line_samples = 0;
        if (this.capture_line_samples < LINE_CONFIRM_SAMPLES) this.capture_line_samples++;
        if (!this.capture_line_seen || this.capture_line_samples >= LINE_CONFIRM_SAMPLES) {
          this.capture_line_ms = now;
          this.capture_line_cm = d;
        }
        this.capture_line_seen = true;
      } else {
        this.capture_line_samples = 0;
      }

      if (turn >= target) this.corner_arc_complete = true;

      const ordinary_exit =
        s.action !== Action.CUSTOM_TURN ||
        Math.abs(Math.abs(this.turnTargetRad()) - JUNCTION_TURN_TARGET_RAD) < 0.0001;
      const angle_full = turn >= target;
      const alignment = this.turn_rewind_active ? Math.max(JUNCTION_EXIT_ALIGNMENT_RAD, JUNCTION_EXIT_REWIND_RAD + 0.03) : JUNCTION_EXIT_ALIGNMENT_RAD;
      const line_ready =
        ordinary_exit && turn >= target - alignment &&
        this.turn_capturing &&
        o.white_count <= JUNCTION_STRAIGHT_EXIT_MAX_WIDTH;
      // Firmware route.c (600b6e9 C-side patch): the exit needs a narrow line
      // *and* a centroid inside JUNCTION_CENTER_LIMIT; reaching the full angle
      // does not relax the centre window.  The simulator used to accept any
      // narrow line once turn >= target, which handed a legacy (grey-only) turn
      // over one frame early, so parity:legacy diverged from the C controller.
      const exit =
        narrow &&
        Math.abs(o.position) <= JUNCTION_CENTER_LIMIT &&
        (line_ready || angle_full || turn >= target - JUNCTION_EXIT_ANGLE_MARGIN_RATIO * target);

      if (!exit || !adjacent) {
        this.exit_count = 0;
        this.exit_candidate_pending = false;
      }
      if (exit && !this.exit_candidate_pending) {
        this.exit_candidate_pending = true;
        this.exit_candidate_ms = now;
        this.exit_candidate_cm = d;
      }
      if (exit && this.exit_count < LINE_CONFIRM_SAMPLES) this.exit_count++;

      // The line exit stays available in pose mode: on some corners the grey
      // bar confirms the outgoing line before the axle finishes the last few
      // degrees, which keeps the next short leg on its wire.
      if (
        this.exit_count >= LINE_CONFIRM_SAMPLES &&
        (line_ready ||
          angle_full ||
          (now - this.exit_candidate_ms >= 2 * JUNCTION_EXIT_CONFIRM_MS &&
            d - this.exit_candidate_cm >= 2.0 * JUNCTION_EXIT_CONFIRM_CM))
      ) {
        this.line_confirmed_exit_used = line_ready && !angle_full;
        this.line_exit_confirmed = true;
        this.line_exit_heading_rad = h;
        this.enterClear(d, now);
      }
      return;
    }

    if (
      now - this.phase_ms >= JUNCTION_CLEAR_TIMEOUT_MS ||
      d - this.clear_cm > JUNCTION_CLEAR_MAX_CM
    ) {
      this.fail(RouteFailure.ROUTE_FAILURE_ACTION_TIMEOUT);
      return;
    }

    /* A passed symbol can be followed by the target corner at the 2 cm
     * boundary. Carry only a real, continuous center corridor across the pose
     * clearance reset, then let the ordinary candidate consume this frame. */
    if (this.pose_mode && this.passing_junction) {
      const shape = junctionShape(o);
      const branch = shape.kind !== JunctionEvidence.JUNCTION_EVIDENCE_NONE;
      const center = routeIsNarrowLine(o) && !branch &&
        o.white_count <= JUNCTION_STRAIGHT_EXIT_MAX_WIDTH &&
        Math.abs(o.position) <= JUNCTION_CENTER_LIMIT;
      if (adjacent && center) {
        if (!this.exit_candidate_pending) {
          this.exit_candidate_pending = true;
          this.exit_candidate_ms = now;
          this.exit_candidate_cm = d;
          this.exit_count = 0;
        }
        if (this.exit_count < LINE_CONFIRM_SAMPLES) this.exit_count++;
        if (this.exit_count >= LINE_CONFIRM_SAMPLES &&
          now - this.exit_candidate_ms >= JUNCTION_ADJACENT_EXIT_MS &&
          d - this.exit_candidate_cm >= JUNCTION_ADJACENT_EXIT_CM) {
          this.have_approach_center = true;
          this.approach_center_ms = now;
          this.approach_center_cm = d;
        }
      } else {
        this.exit_count = 0;
        this.exit_candidate_pending = false;
        if (!adjacent || !branch) this.have_approach_center = false;
      }
      const history = adjacent && this.have_approach_center &&
        now - this.approach_center_ms < JUNCTION_CENTER_MEMORY_MS &&
        d >= this.approach_center_cm &&
        d - this.approach_center_cm <= JUNCTION_CENTER_MEMORY_CM;
      if (!history) this.unbindBarHold();
      /* A new bar after the proven corridor owns its own first sample, even
       * if it starts just before clearance completes. Never promote the
       * resistor's PASS hold, or restart this new bar on subsequent frames. */
      if (history && branch && o.line_mask === 0xff && (s.turn_approach_cm ?? 0) > 0 &&
        !s.corner_arc_enabled && (s.short_link_cm ?? 0) === 0 &&
        (s.action === Action.LEFT || s.action === Action.RIGHT ||
          s.action === Action.CUSTOM_TURN) &&
        this.bar_hold_state !== RouteBarHoldState.ROUTE_BAR_HOLD_PENDING) {
        this.bar_hold_state = RouteBarHoldState.ROUTE_BAR_HOLD_PENDING;
        this.bar_hold_index = this.index;
        this.bar_hold_origin_cm = d;
        this.bar_hold_heading_rad = h;
        this.bar_hold_ms = now;
        this.bar_hold_clear_count = 0;
        this.bar_hold_complete = false;
      }
      /* 特例（用户明确允许）：R17 紧接 (275,0) 的 95 cm 行。电阻的 2 cm 离口屏蔽
       * 正好盖住紧随的真实角点横杠：等屏蔽结束时角点笔画已经消失，只能记到
       * 下一个路口才转（现场索引 14 的“过了”）。该行在已证实的中心走廊之后
       * 只要求 0.3 cm 就交回正常候选流程；角点仍须自己完成连续帧/位移确认，
       * 段距门禁、计数和解口去重其它行均不变。 */
      const closeRow = this.steps[this.index].close_corner === true;
      const clearDistance = closeRow ? JUNCTION_ADJACENT_EXIT_CM : ROUTE_CLEAR_POSE_CM;
      if (d - this.clear_cm < clearDistance || (branch && !history)) return;
      const centerMs = this.approach_center_ms;
      const centerCm = this.approach_center_cm;
      if (history && this.bar_hold_state === RouteBarHoldState.ROUTE_BAR_HOLD_PASS &&
        (branch || this.targetReady(d, false))) {
        this.bar_hold_state = RouteBarHoldState.ROUTE_BAR_HOLD_NONE;
      }
      if (!this.completeClear(d, now) || !history) return;
      this.have_approach_center = true;
      this.approach_center_count = LINE_CONFIRM_SAMPLES;
      this.approach_center_ms = centerMs;
      this.approach_center_cm = centerCm;
      /* Same frame, now in FOLLOW: the ordinary candidate consumes it
       * (C re-enters route_update_pose after the clearance reset). */
      this.update(o, d, h, now, axle_x_cm, axle_y_cm);
      return;
    }

    // Pose mode: the corner is known, so the clearance dedup cannot gain any
    // information; a couple of cm keeps the same order without letting the
    // outgoing line loss stop a short leg.
    if (this.pose_mode && Number.isFinite(axle_x_cm) && d - this.clear_cm >= ROUTE_CLEAR_POSE_CM) {
      this.completeClear(d, now);
      return;
    }

    const straight = this.currentAction() === Action.STRAIGHT;

    if (!straight && this.line_exit_confirmed && !this.passing_junction) {
      const corridor =
        this.exit_candidate_pending &&
        this.approach_center_count >= LINE_CONFIRM_SAMPLES &&
        now - this.exit_candidate_ms >= JUNCTION_ADJACENT_EXIT_MS &&
        d - this.exit_candidate_cm >= JUNCTION_ADJACENT_EXIT_CM;
      // Adjacent junctions may also appear as a one-sided T (centre + branch),
      // e.g. the 30 cm leg (245,140)->(245,170). Reuse the normal shape classifier
      // but still require the corridor, consecutive frames, side agreement and
      // forward displacement before transferring.
      const adjacentShape = junctionShape(o);
      const sides = adjacentShape.sides;
      const branch = adjacentShape.kind !== JunctionEvidence.JUNCTION_EVIDENCE_NONE && sides !== 0;

      if (this.next_junction_samples > 0 && !(this.next_junction_sides & sides)) {
        this.next_junction_samples = 0;
        this.candidate_reject = JunctionReject.JUNCTION_REJECT_SIDE_CHANGE;
        this.candidate_restarts++;
      }

      // Table lengths are axle distances. With the 24 cm sensor lookahead a short leg's

      // next node is already under the sensor when the turn exit is confirmed, so a

      // sensor-side distance that has reached the next length may start the handoff.

      const sensorReady = ((): boolean => {

        if (this.passing_junction || !this.next_segment_origin_valid) return false;

        const next = this.steps[this.index + 1];

        if (!next || !(next.segment_length_cm > 0)) return false;

        return d + GRAY_FORWARD_OFFSET_CM - this.next_segment_origin_cm >= next.segment_length_cm;

      })();


      if (
        sensorReady &&
        this.next_junction_samples === 0 &&
        this.turn_branch_samples >= LINE_CONFIRM_SAMPLES &&
        (this.turn_branch_sides & sides) !== 0
      ) {
        this.next_junction_ms = now;
        this.next_junction_cm = d;
        this.next_junction_sides = this.turn_branch_sides;
        this.next_junction_samples = JUNCTION_CONFIRM_SAMPLES;
      }
      if (adjacent && branch && (corridor || this.next_junction_samples > 0 || sensorReady)) {
        if (this.next_junction_samples === 0) {
          this.next_junction_ms = now;
          this.next_junction_cm = d;
          this.next_junction_sides = sides;
        }
        this.next_junction_sides &= sides;

        if (now - this.next_junction_ms >= JUNCTION_CONFIRM_TIMEOUT_MS) {
          this.fail(RouteFailure.ROUTE_FAILURE_CONFIRM_TIMEOUT);
          return;
        }

        if (this.next_junction_samples < JUNCTION_CONFIRM_SAMPLES) this.next_junction_samples++;
        if (
          this.next_junction_samples >= JUNCTION_CONFIRM_SAMPLES &&
          d - this.next_junction_cm >= JUNCTION_CONFIRM_DISTANCE_CM
        ) {
          const candidate_ms = this.next_junction_ms;
          const candidate_cm = this.next_junction_cm;
          const confirmed_sides = this.next_junction_sides;
          if (!this.completeClear(d, now)) return;
          this.state = RouteState.ROUTE_CANDIDATE;
          this.phase_ms = candidate_ms;
          this.entry_cm = candidate_cm;
          this.candidate_count = JUNCTION_CONFIRM_SAMPLES;
          this.candidate_separated = adjacentShape.kind === JunctionEvidence.JUNCTION_EVIDENCE_SEPARATED;
          this.candidate_kind = adjacentShape.kind;
          this.candidate_sides = confirmed_sides;
          this.update(o, d, h, now, axle_x_cm, axle_y_cm);
          return;
        }
      } else {
        this.next_junction_samples = 0;
      }

      if (adjacent && routeIsNarrowLine(o) && Math.abs(o.position) <= JUNCTION_CENTER_LIMIT) {
        if (this.approach_center_count < LINE_CONFIRM_SAMPLES) this.approach_center_count++;
      } else {
        this.approach_center_count = 0;
      }
    }

    if (
      this.next_junction_samples === 0 &&
      this.componentRecovery(
        o,
        d,
        h,
        now,
        adjacent,
        straight || (this.line_exit_confirmed && !routeIsNarrowLine(o))
      )
    ) {
      return;
    }

    const clear = routeIsClearLine(this, o);
    if (!clear || !adjacent) {
      this.exit_count = 0;
      this.exit_candidate_pending = false;
    }
    if (clear && !this.exit_candidate_pending) {
      this.exit_candidate_pending = true;
      this.exit_candidate_ms = now;
      this.exit_candidate_cm = d;
    }
    if (clear && this.exit_count < LINE_CONFIRM_SAMPLES) this.exit_count++;

    if (straight && clear && this.exit_count >= LINE_CONFIRM_SAMPLES) {
      this.capture_line_ms = now;
      this.capture_line_cm = d;
      if (!this.passing_junction && !this.next_segment_origin_valid) {
        this.anchorNextSegment(d, now);
        this.refreshDistance(d, now);
      }
    }

    if (
      this.exit_count < LINE_CONFIRM_SAMPLES ||
      now - this.exit_candidate_ms < JUNCTION_COUNT_CLEAR_MS ||
      d - this.exit_candidate_cm < JUNCTION_COUNT_CLEAR_CM
    ) {
      return;
    }

    const centered_exit =
      Math.abs(o.position) <= JUNCTION_CENTER_LIMIT &&
      o.white_count <= JUNCTION_STRAIGHT_EXIT_MAX_WIDTH &&
      (straight || this.approach_center_count >= LINE_CONFIRM_SAMPLES);

    if (this.completeClear(d, now) && centered_exit) {
      this.have_approach_center = true;
      this.approach_center_count = LINE_CONFIRM_SAMPLES;
      this.approach_center_ms = now;
      this.approach_center_cm = d;
    }
  }

  public dispatchStopCallback(): void {
    if (!this.stop_pending || this.callback_dispatched) return;
    this.stop_pending = false;
    this.callback_dispatched = true;
    if (this.stop_callback) this.stop_callback(this.index);
  }
}
