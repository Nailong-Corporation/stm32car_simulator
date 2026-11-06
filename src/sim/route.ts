import {
  Action,
  RouteState,
  RouteFailure,
  JunctionEvidence,
  JunctionReject,
  HexagonState,
  type HexagonNavigation,
  type RouteStep,
  type GrayObservation,
  GrayKind,
} from './types';
import {
  CONTROL_MAX_INTERVAL_MS,
  LINE_CONFIRM_SAMPLES,
  GRAY_FORWARD_OFFSET_CM,
  TURN_PIVOT_APPROACH_CM,
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
  JUNCTION_CAPTURE_MIN_RAD,
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
  ROUTE_WAYPOINT_MARGIN_CM,
  ROUTE_WAYPOINT_CONFIRM_CM,
  ROUTE_WAYPOINT_FALLBACK_CM,
  ROUTE_WAYPOINT_RING_CONFIRM_CM,
  ROUTE_WAYPOINT_GATE_CM,
  ROUTE_WAYPOINT_EXIT_CM,
  ROUTE_WAYPOINT_EXIT_ANGLE_RAD,
  ROUTE_WAYPOINT_SLOW_CM,
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
  WHEEL_TRACK_CM,
  INTERFERENCE_TIMEOUT_MS,
  INTERFERENCE_MAX_DISTANCE_CM,
} from './config';

const SHORT_LINK_MIN_CM = 20;
const SHORT_LINK_MAX_CM = 40;
const SHORT_LINK_MAX_LATERAL_CM = 14;
const SHORT_LINK_MAX_HEADING_RAD = 0.30;
const SHORT_LINK_TIMEOUT_MS = 6000;
function shortLinkLength(s: RouteStep): number { return s.short_link_cm ?? 0; }
function ordinaryRightAngle(s: RouteStep): boolean {
  return !s.corner_arc_enabled && (s.action === Action.LEFT || s.action === Action.RIGHT ||
    (s.action === Action.CUSTOM_TURN && Math.abs(Math.abs(s.angle_deg) - 90) < 0.001));
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
  /* Pose mode: after a pivot the car keeps exit speed until the outgoing line
   * has been re-acquired, so a printed symbol next to the corner cannot be
   * chased at cruise speed. */
  public post_turn_slow = false;
  public post_turn_samples = 0;
  public short_link_tracking = false;
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
  public repeat_action: Action = Action.LEFT;
  public repeat_count = 0;
  public completed_laps = 0;
  public candidate_count = 0;
  public exit_count = 0;
  public approach_center_count = 0;
  public have_approach_center = false;
  public valid = false;
  public stop_pending = false;
  public callback_dispatched = false;
  public corner_arc_complete = false;
  public turn_capturing = false;
  public lap_boundary = false;
  public capture_line_seen = false;
  public line_exit_confirmed = false;
  public capture_line_samples = 0;
  public capture_line_ms = 0;
  public capture_line_cm = 0;
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

  public init(steps: RouteStep[], callback?: (step_index: number) => void): void {
    this.steps = steps.map((s) => ({ ...s }));
    this.count = steps.length;
    this.repeat_count = 0;
    this.repeat_action = Action.LEFT;
    this.stop_callback = callback;
    this.valid = steps !== null && steps.length > 0;

    for (let i = 0; this.valid && i < this.count; ++i) {
      const s = steps[i];
      this.valid =
        (s.junction_count > 0 || s.segment_length_cm > 0) &&
        s.action <= Action.HEXAGON &&
        s.cruise_speed > 0 &&
        s.cruise_speed <= WHEEL_TARGET_MAX_CM_S &&
        Number.isFinite(s.turn_speed) &&
        s.turn_speed > 0 &&
        s.turn_speed <= s.cruise_speed &&
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
          (Number.isFinite(CORNER_RADIUS_CM) && CORNER_RADIUS_CM > WHEEL_TRACK_CM * 0.5)) &&
        Number.isFinite(shortLinkLength(s)) && (shortLinkLength(s) === 0 ||
          (i > 0 && shortLinkLength(s) >= SHORT_LINK_MIN_CM && shortLinkLength(s) <= SHORT_LINK_MAX_CM &&
            ordinaryRightAngle(s) && ordinaryRightAngle(steps[i - 1])));
    }

    if (this.valid && steps[this.count - 1].action !== Action.STOP) this.valid = false;
    this.state = this.valid ? RouteState.ROUTE_FOLLOW : RouteState.ROUTE_FAILED;
    this.failure = this.valid ? RouteFailure.ROUTE_FAILURE_NONE : RouteFailure.ROUTE_FAILURE_INVALID;
  }

  public start(d: number, now: number): void {
    this.short_link_tracking = this.short_link_active = this.turn_approaching = false;
    this.post_turn_slow = false;
    this.post_turn_samples = 0;
    /* Absolute corner waypoints: the anchored start pose plus the cumulative
     * (geometry - 5 cm + 5 cm, angle) chain. Computed at START so a scenario
     * test may re-anchor the start pose first. */
    this.waypoint_x_cm = [];
    this.waypoint_y_cm = [];
    let wx = this.start_x_cm;
    let wy = this.start_y_cm;
    let wh = this.start_heading_rad;
    for (let i = 0; i < this.count && i < 32; ++i) {
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
      this.steps[this.index].action !== Action.HEXAGON &&
      this.steps[this.index].corner_arc_enabled &&
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
    const countReady = s.junction_count > 0 &&
      this.junctions_seen + (next ? 1 : 0) >= s.junction_count;
    const distanceReady = s.segment_length_cm > 0 &&
      d - this.origin_cm >= s.segment_length_cm;
    return countReady || distanceReady;
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
    this.turn_rewind_active = this.turn_rewind_pending = false;
    this.line_exit_confirmed = this.turn_ended_on_line = this.capture_line_seen = false;
    this.capture_line_samples = 0;
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
      vertex_y_cm: 0,
      leg_start_cm: 0,
      leg_from_x_cm: 0,
      leg_from_y_cm: 0,
      cross_cm: 0,
    };
  }

  private beginAction(d: number, h: number, now: number): void {
    this.state = RouteState.ROUTE_EXECUTING;
    this.phase_ms = now;
    this.entry_cm = d;
    if (!this.short_link_active) this.entry_heading = h;
    this.action_start_heading = this.entry_heading;
    this.turn_approach_origin_cm = d;
    this.turn_approach_cm = Math.max(0.0,
      TURN_PIVOT_APPROACH_CM - (this.pose_mode ? this.pose_trigger_late_cm : 0.0));
    this.turn_approaching = !this.passing_junction &&
      !this.usesCornerArc() &&
      Math.abs(Math.abs(this.turnTargetRad()) - JUNCTION_TURN_TARGET_RAD) < 1e-4 &&
      this.steps[this.index].action !== Action.HEXAGON;
    this.resetEvidence(now);
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
    this.next_segment_origin_valid = true;
    this.search_pending = false;
  }

  private startShortLinkTracking(d: number, h: number, now: number): void {
    this.short_link_active = false;
    this.short_link_tracking = !this.passing_junction && this.index + 1 < this.count &&
      shortLinkLength(this.steps[this.index + 1]) > 0 && ordinaryRightAngle(this.steps[this.index]);
    if (!this.short_link_tracking) return;
    this.short_link_heading = this.entry_heading + this.turnTargetRad();
    this.short_link_forward_cm = this.short_link_lateral_cm = 0;
    this.short_link_last_cm = d; this.short_link_last_heading = h; this.short_link_ms = now;
  }

  private enterClear(d: number, now: number): void {
    if (this.short_link_tracking && !this.passing_junction && this.line_exit_confirmed &&
      this.index + 1 < this.count && shortLinkLength(this.steps[this.index + 1]) > 0) {
      if (Math.abs(this.line_exit_heading_rad - this.short_link_heading) > SHORT_LINK_MAX_HEADING_RAD) {
        this.short_link_tracking = false;
      } else {
        // Previous lateral pivot motion includes its actual coast footprint.
        this.short_link_pivot_lead_cm = Math.max(WHEEL_TRACK_CM * 0.5, Math.abs(this.short_link_lateral_cm));
        if (this.short_link_pivot_lead_cm > SHORT_LINK_MAX_LATERAL_CM ||
          shortLinkLength(this.steps[this.index + 1]) - this.short_link_pivot_lead_cm - this.short_link_forward_cm < 1) {
          this.short_link_tracking = false; this.fail(RouteFailure.ROUTE_FAILURE_EXIT_RECAPTURE); return;
        }
        ++this.index; this.junctions_seen = 0;
        this.origin_cm = this.final_segment_origin_cm = d; this.segment_ms = now;
        this.beginAction(d, this.short_link_heading, now);
        this.short_link_active = true;
        const remaining = shortLinkLength(this.steps[this.index]) - this.short_link_pivot_lead_cm - this.short_link_forward_cm;
        this.turn_approach_origin_cm = d - (TURN_PIVOT_APPROACH_CM - remaining);
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
      const wh0 = h - Math.PI / 2;
      const sx0 = axle_x_cm + GRAY_FORWARD_OFFSET_CM * Math.cos(wh0);
      const sy0 = axle_y_cm + GRAY_FORWARD_OFFSET_CM * Math.sin(wh0);
      this.pose_gap_bar_cm = Math.hypot(
        sx0 - this.waypoint_x_cm[this.index],
        sy0 - this.waypoint_y_cm[this.index]
      );
      this.pose_bearing_rad =
        Math.atan2(this.waypoint_y_cm[this.index] - axle_y_cm,
          this.waypoint_x_cm[this.index] - axle_x_cm) + Math.PI / 2;
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
        const worldH = h - Math.PI / 2;
        this.pose_cross_bar_cm =
          this.pose_cross_cm + GRAY_FORWARD_OFFSET_CM * Math.sin(worldH - segH);
        this.pose_seg_heading_rad = segH + Math.PI / 2;
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
    if (this.short_link_tracking) {
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
        this.turn_approach_origin_cm = d - (TURN_PIVOT_APPROACH_CM - remaining);
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
      // Parallel pose trigger (2026-11-06): the table corner is known in track
      // coordinates, so encoders+IMU can confirm it even when a printed symbol
      // hides the wire or the leg is shorter than the 24 cm look-ahead bar.
      if (
        this.state === RouteState.ROUTE_FOLLOW &&
        Number.isFinite(axle_x_cm) &&
        Number.isFinite(axle_y_cm) &&
        this.waypoint_x_cm.length === this.count
      ) {
        const st = this.steps[this.index];
        const wh = h - Math.PI / 2;
        const sx = axle_x_cm + GRAY_FORWARD_OFFSET_CM * Math.cos(wh);
        const sy = axle_y_cm + GRAY_FORWARD_OFFSET_CM * Math.sin(wh);
        const wx = this.waypoint_x_cm[this.index];
        const wy = this.waypoint_y_cm[this.index];
        const axleGap = Math.hypot(axle_x_cm - wx, axle_y_cm - wy);
        const barGap = Math.hypot(sx - wx, sy - wy);
        // A missed window must not stall the row: if the axle has already
        // passed the corner while still within a corridor width of it, this is
        // that corner (only reachable when the previous exit ran long).
        const proj = (wx - axle_x_cm) * Math.cos(wh) + (wy - axle_y_cm) * Math.sin(wh);
        const lateral = Math.abs(-(wx - axle_x_cm) * Math.sin(wh) + (wy - axle_y_cm) * Math.cos(wh));
        // The trigger is timed for the bar, so a bar already past the corner
        // (short leg started late) must still hand the row over instead of
        // driving by until the next corner.
        const barProj = proj - GRAY_FORWARD_OFFSET_CM;
        const passed = barProj <= ROUTE_WAYPOINT_CONFIRM_CM && lateral <= ROUTE_WAYPOINT_GATE_CM;
        const reached = barGap <= ROUTE_WAYPOINT_CONFIRM_CM || passed;
        const fallback = st.segment_length_cm > 0 &&
          d - this.origin_cm >= st.segment_length_cm + ROUTE_WAYPOINT_FALLBACK_CM;
        this.pose_trigger_late_cm = barProj < 0 ? -barProj : 0;
        if (st.action !== Action.STRAIGHT && (reached || fallback)) {
          this.candidate_count = 0;
          this.search_pending = false;
          this.passing_junction = false;
          if (this.junctions_seen < 0xffff) ++this.junctions_seen;
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
      }
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

      const history = (this.state === RouteState.ROUTE_CANDIDATE && d >= this.entry_cm) || recent_center;
      const { kind, sides } = junctionShape(o);
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
        this.candidate_count = 0;
        this.candidate_separated = false;
        this.candidate_kind = JunctionEvidence.JUNCTION_EVIDENCE_NONE;
        this.candidate_sides = 0;
        this.state = RouteState.ROUTE_FOLLOW;
        return;
      }

      this.candidate_reject = JunctionReject.JUNCTION_REJECT_NONE;

      if (this.state === RouteState.ROUTE_CANDIDATE && !(this.candidate_sides & sides)) {
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
      // Pose gate: a real junction far from this table row's known corner is a
      // passing node (printed elements and swept nodes included), not the
      // target. Keeps the 8-channel proof but stops an over-counted node from
      // starting the turn tens of cm early.
      let poseMatches = true;
      if (Number.isFinite(axle_x_cm) && Number.isFinite(axle_y_cm) &&
        this.waypoint_x_cm.length === this.count) {
        const wh = h - Math.PI / 2;
        const sx = axle_x_cm + GRAY_FORWARD_OFFSET_CM * Math.cos(wh);
        const sy = axle_y_cm + GRAY_FORWARD_OFFSET_CM * Math.sin(wh);
        // The ring vertex is a lollipop: the bar sees the split bands before
        // the axle is anywhere near V0, so a HEXAGON row only accepts a grey
        // node within the ring confirmation radius.
        const gate = this.steps[this.index].action === Action.HEXAGON
          ? ROUTE_WAYPOINT_RING_CONFIRM_CM
          : ROUTE_WAYPOINT_GATE_CM;
        const __gap = Math.hypot(sx - this.waypoint_x_cm[this.index], sy - this.waypoint_y_cm[this.index]);
        poseMatches = __gap <= gate;
      }
      this.passing_junction = !this.targetReady(d, false) || !poseMatches;
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
      // The front bar sees a junction L ahead of the axle. A pivot of radius
      // track/2 must start L-track/2 later, not at the detection point.
      if (d - this.turn_approach_origin_cm < this.turn_approach_cm) return;
      if (this.short_link_active && !this.entrySpeedReady()) {
        this.state = RouteState.ROUTE_BRAKING; this.phase_ms = now; this.entry_cm = d; return;
      }
      this.turn_approaching = false;
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
    if (
      action !== Action.STRAIGHT &&
      !this.passing_junction &&
      Number.isFinite(axle_x_cm) &&
      Number.isFinite(axle_y_cm) &&
      this.waypoint_x_cm.length === this.count
    ) {
      const wob = h - Math.PI / 2;
      const gap = Math.hypot(axle_x_cm - this.waypoint_x_cm[this.index], axle_y_cm - this.waypoint_y_cm[this.index]);
      if (gap <= ROUTE_WAYPOINT_EXIT_CM && turn >= target - ROUTE_WAYPOINT_EXIT_ANGLE_RAD) {
        this.line_exit_confirmed = true;
        this.line_exit_heading_rad = h;
        this.enterClear(d, now);
        return;
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
      if (now - this.phase_ms >= this.actionTimeoutMs()) {
        this.fail(RouteFailure.ROUTE_FAILURE_ACTION_TIMEOUT);
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
      // A completed pivot may leave the axle 1-2 cm off the exit line (coast,
      // printed symbol edge).  Once the full angle is reached, a narrow
      // single-region line is the exit even if the centroid is one channel
      // beyond the strict centre window; blind wide/edge reads still fail.
      const exit =
        narrow &&
        (Math.abs(o.position) <= JUNCTION_CENTER_LIMIT || angle_full) &&
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

    // Pose mode: the corner is known, so the clearance dedup cannot gain any
    // information; a couple of cm keeps the same order without letting the
    // outgoing line loss stop a short leg.
    if (Number.isFinite(axle_x_cm) && d - this.clear_cm >= ROUTE_CLEAR_POSE_CM) {
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
