import {
  LineState,
  LineStopReason,
  Action,
  RouteState,
  RouteFailure,
  HexagonState,
  GrayKind,
  type RouteStep,
  type GrayObservation,
  type EncoderCounts,
  type MotorCommand,
  HeadingSource,
} from './types';
import {
  WHEEL_TARGET_MAX_CM_S,
  STARTUP_RAMP_TIME_S,
  TRACKING_SPEED_LEAD_CM_S,
  TARGET_RISE_CM_S2,
  TURN_EXIT_ACCEL_CM_S2,
  LINE_FOLLOW_SPEED_CM_S,
  HOLD_SPEED_CM_S,
  AMBIGUOUS_SPEED_CM_S,
  HEADING_KP_CM_S_PER_RAD,
  POSITION_KP_CM_S,
  POSITION_KI_CM_S,
  POSITION_KD_CM_S,
  PID_INTEGRAL_LIMIT,
  PID_DERIVATIVE_TAU_S,
  POSITION_CURVE_SLOWDOWN,
  LINE_CONFIRM_SAMPLES,
  INTERFERENCE_TIMEOUT_MS,
  INTERFERENCE_MAX_DISTANCE_CM,
  COMPONENT_CROSS_MAX_CM,
  COMPONENT_CROSS_MS,
  POSE_BOUNDED_LOSS_MS,
  POSE_BOUNDED_LOSS_CM,
  BRAKING_ACCEL_FILTER_TAU_S,
  BRAKING_SAFE_DECEL_CM_S2,
  BRAKING_DISTANCE_MARGIN_CM,
  BRAKING_TARGET_JERK_CM_S3,
  BRAKING_REVERSE_MAX_MS,
  BRAKING_REVERSE_MIN_SPEED_CM_S,
  BRAKING_REVERSE_GUARD_CM,
  BRAKING_REVERSE_HARDWARE_CONFIRMED,
  MOTOR_REVERSE_BRAKE_ENABLED,
  REVERSE_BRAKE_MARGIN_CM_S,
  REVERSE_BRAKE_MIN_CRUISE_CM_S,
  JUNCTION_PASS_SPEED_CM_S,
  JUNCTION_SETTLE_BAND_RAD,
  JUNCTION_PASS_ALIGNED_RAD,
  JUNCTION_PASS_SETTLED_YAW_RAD_S,
  JUNCTION_TURN_TARGET_RAD,
  JUNCTION_SETTLE_SPEED_CM_S,
  JUNCTION_SETTLE_HEADING_KP,
  JUNCTION_CAPTURE_HEADING_KP,
  JUNCTION_CAPTURE_CENTER_BIAS_CM_S,
  JUNCTION_CAPTURE_MIN_TURN_CM_S,
  JUNCTION_YAW_DAMPING,
  JUNCTION_YAW_FILTER_TAU_S,
  JUNCTION_EXIT_SPEED_CM_S,
  COMPONENT_RECOVERY_SPEED_CM_S,
  HEXAGON_SPEED_CM_S,
  NAVIGATION_RECOVERY_SPEED_CM_S,
  CORNER_RADIUS_CM,
  CORNER_HEADING_KP,
  CORNER_ARC_RADIUS_CM,
  CORNER_ARC_SPEED_CM_S,
  CORNER_ARC_HEADING_KP,
  CORNER_ARC_DIFF_SLEW_CM_S2,
  CORNER_ARC_EXIT_DAMPING,
  STRAIGHT_YAW_DAMPING,
  GRAY_FORWARD_OFFSET_CM,
  WHEEL_TRACK_CM,
  JUNCTION_EXIT_REWIND_RAD,
  STEERING_FEEDBACK_MIN_CM_S,
  STEERING_FEEDBACK_MIN_HALF_CM_S,
  ROUTE_START_Y_CM,
  ROUTE_WAYPOINT_SLOW_CM,
  JUNCTION_POSE_GRAY_SPREAD,
  JUNCTION_CENTER_LIMIT,
  JUNCTION_CROSS_KP,
  POSE_PURSUIT_LOOKAHEAD_CM,
  ROUTE_WAYPOINT_GATE_CM,
  HEXAGON_TURN_TOL_RAD,
} from './config';
import { OdometryModel, defaultOdometryCalibration } from './odometry';
import { RoutePlannerModel, routeIsNarrowLine, routeIsClearLine } from './route';
import { SpeedControlModel } from './speed_control';
import { Pid } from './pid';
import { BrakingSlewModel, brakingPlan, brakingDecide, brakingSlew } from './braking_planner';

/** True while the grayscale pattern matches a printed component symbol
 * (hollow resistor interior / symmetric wide graphic band). */
export function grayIsComponentPattern(o: GrayObservation): boolean {
  if (
    o.region_count >= 2 &&
    (o.line_mask & 0x18) === 0 &&
    (o.line_mask & 0x03) !== 0 &&
    (o.line_mask & 0xc0) !== 0
  ) {
    return true;
  }
  return (
    o.region_count === 1 &&
    o.white_count >= 5 &&
    o.white_count <= 7 &&
    (o.line_mask & 0x07) !== 0 &&
    (o.line_mask & 0xe0) !== 0
  );
}

export class LineFollowModel {
  public state: LineState = LineState.LINE_IDLE;
  public stop_reason: LineStopReason = LineStopReason.LINE_STOP_NONE;
  public heading_source: HeadingSource = HeadingSource.HEADING_IMU;
  /* 2026-11-08 START 时对齐 IMU 相对航向的零点：位姿导航假设「START 瞬间车头朝向
   * 赛道起点航向、里程位姿为零」。上电后被搬动/旋转过的车体会让相对航向不为 0，
   * 旧换算 world = h − π/2 会把本行 waypoint 算到车后，位姿触发在 d≈0 就接受动作
   * （现场表现为一按 START 就左转，而不是先直行）。 */
  public imu_heading_offset_rad = 0;

  public odometry: OdometryModel;
  public route: RoutePlannerModel;
  public speed: SpeedControlModel;
  public braking_slew: BrakingSlewModel;
  public pid: Pid;
  public command: MotorCommand = {
    left_pwm: 0,
    right_pwm: 0,
    left_forward: true,
    right_forward: true,
    standby: false,
  };

  public braking_origin_cm = 0;
  public braking_active = false;
  public braking_reverse_active = false;
  /* Reverse-brake diagnostics, mirroring line_follow.c: cumulative reverse-torque
   * time per braking segment, the planner's required deceleration and the
   * filtered observed deceleration used to decide that reverse torque is needed. */
  public braking_reverse_ms = 0;
  public braking_required_cm_s2 = 0;
  public braking_accel_cm_s2 = 0;
  public braking_speed_primed = false;
  public braking_previous_speed_cm_s = 0;
  public have_reliable = false;
  public abnormal = false;
  public valid_count = 0;
  public reliable_position = 0;
  public base = 0;
  public correction = 0;
  public left_target_cm_s = 0;
  public right_target_cm_s = 0;
  public hold_heading_rad = 0;
  public abnormal_origin_cm = 0;
  /** Latched bounded window for crossing a printed component symbol. */
  public component_cross_latched = false;
  public component_cross_ms = 0;
  public component_cross_cm = 0;
  public turn_yaw_rate_rad_s = 0;
  public exit_heading_pending = false;
  public exit_heading_rad = 0;
  public exit_heading_origin_cm = 0;
  public exit_accelerating = false;
  public startup_elapsed_s = 0;
  public last_reliable_ms = 0;
  public sample_ms = 0;
  public observation: GrayObservation = {
    kind: GrayKind.GRAY_NONE,
    raw_mask: 0,
    line_mask: 0,
    selected_mask: 0,
    white_count: 0,
    region_count: 0,
    position: 0,
    prefilter_mask: 0,
    seam_mask: 0,
  };

  constructor(steps: RouteStep[] = []) {
    this.odometry = new OdometryModel(defaultOdometryCalibration());
    this.route = new RoutePlannerModel();
    this.speed = new SpeedControlModel();
    this.braking_slew = new BrakingSlewModel();
    this.pid = new Pid({
      kp: POSITION_KP_CM_S,
      ki: POSITION_KI_CM_S,
      kd: POSITION_KD_CM_S,
      integral_limit: PID_INTEGRAL_LIMIT,
      derivative_tau: PID_DERIVATIVE_TAU_S,
    });
    this.init(steps);
  }

  public init(steps: RouteStep[], callback?: (step_index: number) => void): void {
    this.state = LineState.LINE_IDLE;
    this.stop_reason = LineStopReason.LINE_STOP_NONE;
    this.heading_source = HeadingSource.HEADING_IMU;
    this.imu_heading_offset_rad = 0;
    this.odometry.init(defaultOdometryCalibration());
    this.speed.reset();
    this.braking_slew.reset();
    this.pid.reset();
    this.route.init(steps, callback);

    this.braking_active = false;
    this.braking_reverse_active = false;
    this.braking_reverse_ms = 0;
    this.braking_required_cm_s2 = 0;
    this.braking_accel_cm_s2 = 0;
    this.braking_speed_primed = false;
    this.braking_previous_speed_cm_s = 0;
    this.have_reliable = false;
    this.abnormal = false;
    this.valid_count = 0;
    this.reliable_position = 0;
    this.base = 0;
    this.correction = 0;
    this.left_target_cm_s = 0;
    this.right_target_cm_s = 0;
    this.hold_heading_rad = 0;
    this.abnormal_origin_cm = 0;
    this.component_cross_latched = false;
    this.component_cross_ms = 0;
    this.component_cross_cm = 0;
    this.turn_yaw_rate_rad_s = 0;
    this.exit_heading_pending = false;
    this.exit_heading_rad = 0;
    this.exit_heading_origin_cm = 0;
    this.exit_accelerating = false;
    this.startup_elapsed_s = 0;
    this.last_reliable_ms = 0;
    this.sample_ms = 0;
    this.command = {
      left_pwm: 0,
      right_pwm: 0,
      left_forward: true,
      right_forward: true,
      standby: false,
    };
  }

  public stop(reason: LineStopReason): void {
    this.state = LineState.LINE_STOPPED;
    this.stop_reason = reason;
    this.left_target_cm_s = 0;
    this.right_target_cm_s = 0;
    this.base = 0;
    this.correction = 0;
    this.pid.reset();
    this.speed.reset();
    this.braking_slew.reset();
    this.command = {
      left_pwm: 0,
      right_pwm: 0,
      left_forward: true,
      right_forward: true,
      standby: false,
    };
  }

  private targetRise(): number {
    return this.exit_accelerating && this.route.state !== RouteState.ROUTE_CLEARING
      ? TURN_EXIT_ACCEL_CM_S2 : TARGET_RISE_CM_S2;
  }

  private nextOuter(outer: number, dt: number): number {
    const previous = Math.max(this.left_target_cm_s, this.right_target_cm_s);
    const fraction = Math.min(1, (this.startup_elapsed_s + dt) / STARTUP_RAMP_TIME_S);
    return Math.min(WHEEL_TARGET_MAX_CM_S, outer * fraction, previous + this.targetRise() * dt);
  }

  private targets(base: number, correction: number, dt: number): void {
    const fraction = Math.min(1, (this.startup_elapsed_s + dt) / STARTUP_RAMP_TIME_S);
    this.base = Math.min(base * fraction, this.base + this.targetRise() * dt);
    if (this.base >= base) this.exit_accelerating = false;
    this.startup_elapsed_s = Math.min(STARTUP_RAMP_TIME_S, this.startup_elapsed_s + dt);
    if (!Number.isFinite(correction)) correction = 0;
    this.left_target_cm_s = Math.max(0, Math.min(WHEEL_TARGET_MAX_CM_S, this.base + correction));
    this.right_target_cm_s = Math.max(0, Math.min(WHEEL_TARGET_MAX_CM_S, this.base - correction));
    this.correction = 0.5 * (this.left_target_cm_s - this.right_target_cm_s);
  }

  private outerTargets(outer: number, correction: number, dt: number): void {
    const requested = outer;
    outer = this.nextOuter(outer, dt);
    if (this.state === LineState.LINE_TRACKING) {
      const measured = Math.max(0, this.odometry.left_speed_cm_s, this.odometry.right_speed_cm_s);
      outer = Math.min(outer, measured + TRACKING_SPEED_LEAD_CM_S);
    }
    if (outer >= requested) this.exit_accelerating = false;
    correction = Math.max(-outer * 0.5, Math.min(outer * 0.5, correction));
    const mean = outer - Math.abs(correction);
    this.left_target_cm_s = mean + correction;
    this.right_target_cm_s = mean - correction;
    // Tracking preserves the grey PID/yaw half-difference despite common wheel
    // lag. Keep the legacy manoeuvre/hold compensation, matching the firmware.
    if (this.state !== LineState.LINE_TRACKING &&
        Math.abs(correction) >= STEERING_FEEDBACK_MIN_HALF_CM_S && outer > 0) {
      const measured = correction > 0 ? this.odometry.left_speed_cm_s : this.odometry.right_speed_cm_s;
      let inner = correction > 0 ? this.right_target_cm_s : this.left_target_cm_s;
      if (measured >= STEERING_FEEDBACK_MIN_CM_S && measured < outer) {
        inner = Math.min(inner, measured * inner / outer);
        if (correction > 0) this.right_target_cm_s = inner;
        else this.left_target_cm_s = inner;
      }
    }
    this.base = 0.5 * (this.left_target_cm_s + this.right_target_cm_s);
    this.correction = 0.5 * (this.left_target_cm_s - this.right_target_cm_s);
    this.startup_elapsed_s = Math.min(STARTUP_RAMP_TIME_S, this.startup_elapsed_s + dt);
  }

  private turnTargets(outer: number, correction: number, dt: number): void {
    const requested = outer;
    outer = this.nextOuter(outer, dt);
    if (outer >= requested) this.exit_accelerating = false;
    correction = Math.max(-outer * 0.5, Math.min(outer * 0.5, correction));
    const mean = outer - Math.abs(correction);
    this.left_target_cm_s = mean + correction;
    this.right_target_cm_s = mean - correction;
    this.base = 0.5 * (this.left_target_cm_s + this.right_target_cm_s);
    this.correction = 0.5 * (this.left_target_cm_s - this.right_target_cm_s);
    this.startup_elapsed_s = Math.min(STARTUP_RAMP_TIME_S, this.startup_elapsed_s + dt);
  }

  private straightTargets(base: number, correction: number, dt: number): void {
    this.targets(base, 0, dt);
    this.left_target_cm_s = Math.max(0, Math.min(WHEEL_TARGET_MAX_CM_S, this.base + correction));
    this.right_target_cm_s = Math.max(0, Math.min(WHEEL_TARGET_MAX_CM_S, this.base - correction));
    this.correction = 0.5 * (this.left_target_cm_s - this.right_target_cm_s);
    this.base = 0.5 * (this.left_target_cm_s + this.right_target_cm_s);
  }

  /** Borrow the recent exit axis only during an adjacent non-target passage. */
  private passageHeading(fallback: number): number {
    return this.exit_heading_pending &&
      Math.abs(this.exit_heading_rad - this.odometry.heading_rad) <= JUNCTION_SETTLE_BAND_RAD
      ? this.exit_heading_rad : fallback;
  }

  /** A rolling corner has just released its differential, or the route is still
   * clearing the exit: those frames keep the stronger corner-exit damping
   * (line_follow.c `arc_just_exited || state==ROUTE_CLEARING`). It is spelled as
   * a helper because the FOLLOW straight branch has already narrowed the route
   * state, where the CLEARING arm is unreachable in this controller. */
  private inCornerExit(): boolean {
    return this.route.arc_just_exited || this.route.state === RouteState.ROUTE_CLEARING;
  }

  /** Which grey reading may steer, now that the straight branch trusts grey.
   * Mirrors line_follow.c gray_plausible() exactly:
   *  - a centred narrow line is unambiguous, always trusted (after a pivot the
   *    pose can be several cm off);
   *  - a pose cross-track beyond ROUTE_WAYPOINT_GATE_CM means the pose itself is
   *    the suspect side, so the visible narrow line is trusted (field 2026-11-08:
   *    pose_cross≈-16.5 cm, mask=0x04 = real line, old code vetoed it and the car
   *    drove blind along its heading);
   *  - the pose says "still on the segment" and the narrow reading disagrees: that
   *    is most likely the 24 cm look-ahead bar already sitting on another real
   *    wire (hexagon NW edge) or a printed stroke, so the pose hold keeps
   *    authority. Wide/multi-region patterns are never trusted here. */
  private grayPlausible(obs: GrayObservation): boolean {
    if (!this.route.pose_mode || !this.route.pose_cross_valid) return true;
    if (!routeIsNarrowLine(obs)) return false;
    if (Math.abs(obs.position) <= JUNCTION_CENTER_LIMIT) return true;
    if (Math.abs(this.route.pose_cross_cm) > ROUTE_WAYPOINT_GATE_CM) return true;
    const expected = -this.route.pose_cross_cm / 0.55;
    return Math.abs(obs.position - expected) <= JUNCTION_POSE_GRAY_SPREAD;
  }

  /** Ordinary maneuvers use route phase bounds; HEXAGON keeps its own loss rules. */
  private routeOwnsGrayLoss(): boolean {
    const r = this.route;
    return r.state === RouteState.ROUTE_BRAKING ||
      r.state === RouteState.ROUTE_RECOVERING ||
      ((r.state === RouteState.ROUTE_EXECUTING || r.state === RouteState.ROUTE_CLEARING) &&
        (r.steps[r.index]?.action !== Action.HEXAGON ||
         (r.state === RouteState.ROUTE_EXECUTING && r.turn_approaching)));
  }

  public update(
    observation: GrayObservation,
    counts: EncoderCounts,
    now_ms: number,
    dt: number,
    start_request: boolean,
    imu_valid = true,
    imu_heading_rad = 0
  ): void {
    this.sample_ms = now_ms;
    this.observation = observation;

    const previous_heading = this.odometry.heading_rad;
    const previous_route = this.route.state;

    /* START 是位姿原点：IMU 相对航向统一减去 START 时刻的读数（见
     * imu_heading_offset_rad）。灰度/编码器兜底时沿用编码器航向。 */
    const use_imu = this.heading_source === HeadingSource.HEADING_IMU && imu_valid;
    if (!this.odometry.updateWithHeading(counts, dt, use_imu,
      imu_heading_rad - this.imu_heading_offset_rad)) {
      this.stop(LineStopReason.LINE_STOP_ENCODER);
      return;
    }

    if (dt > 0) {
      const yaw_rate = (this.odometry.heading_rad - previous_heading) / dt;
      const alpha = dt / (JUNCTION_YAW_FILTER_TAU_S + dt);
      this.turn_yaw_rate_rad_s += alpha * (yaw_rate - this.turn_yaw_rate_rad_s);
    }

    const obs = observation;
    const narrow = routeIsNarrowLine(obs);

    if (narrow) {
      if (this.valid_count < LINE_CONFIRM_SAMPLES) this.valid_count++;
    } else {
      this.valid_count = 0;
    }
    const stable = this.valid_count >= LINE_CONFIRM_SAMPLES;

    if (this.state === LineState.LINE_IDLE || this.state === LineState.LINE_STOPPED) {
      if (start_request) {
        if (!stable) {
          return;
        }
        this.state = LineState.LINE_TRACKING;
        this.stop_reason = LineStopReason.LINE_STOP_NONE;
        this.startup_elapsed_s = 0;
        this.exit_heading_pending = false;
        this.have_reliable = true;
        this.last_reliable_ms = now_ms;
        this.reliable_position = obs.position;
        this.braking_slew.reset();
        this.braking_active = false;
        this.braking_speed_primed = false;
        this.braking_reverse_ms = 0;
        this.braking_reverse_active = false;
        this.braking_required_cm_s2 = 0;
        /* START 是位姿原点。旧的位姿导航假设「START 时相对航向 = 0、里程位姿 = 0」，
         * 但上电后把车搬上赛道时会转动车体，IMU 相对航向在 START 已不是 0；此时
         * world = h − π/2 的换算把本行 waypoint 算到车后（bar_proj<0 → passed），
         * 位姿触发在 d≈0 立刻接受动作 —— 现场表现就是一按 START 就左转、不先直行。
         * 这里在每次 START 重新对齐：航向零点取当前 IMU 读数，里程位姿清零。 */
        this.imu_heading_offset_rad =
          this.heading_source === HeadingSource.HEADING_IMU && imu_valid &&
          Number.isFinite(imu_heading_rad)
            ? imu_heading_rad
            : 0.0;
        this.odometry.heading_rad = 0.0;
        this.odometry.x_cm = this.odometry.y_cm = 0.0;
        this.odometry.sensor_x_cm = GRAY_FORWARD_OFFSET_CM;
        this.odometry.sensor_y_cm = 0.0;
        this.route.start(this.odometry.distance_cm, now_ms);
      } else {
        return;
      }
    }


    // Route mode always bounds ordinary lost/ambiguous-line holding, even with
    // LOST_LINE_STOP_ENABLED=0. Check before a late good sample can renew it.
    // Printed component symbols (hollow resistors, transistor V, battery
    // plates) latch a bounded straight-crossing window; a plain line loss keeps
    // the 400 ms / 8 cm cut-off.
    if (!this.abnormal) this.component_cross_latched = false;
    else if (!this.component_cross_latched && grayIsComponentPattern(obs)) {
      this.component_cross_latched = true;
      this.component_cross_ms = now_ms;
      this.component_cross_cm = this.odometry.distance_cm;
    }
    const component_crossing = this.abnormal && this.component_cross_latched &&
      now_ms - this.component_cross_ms < COMPONENT_CROSS_MS &&
      Math.abs(this.odometry.distance_cm - this.component_cross_cm) < COMPONENT_CROSS_MAX_CM;
    // Pose mode: a printed component hides the wire for up to ~10 cm along the
    // segment, and the pose says the car is still on that segment, so reuse the
    // bounded component-crossing window instead of the plain 400 ms / 8 cm cut.
    const poseBoundedLoss = this.route.pose_mode &&
      (this.route.state === RouteState.ROUTE_FOLLOW ||
        this.route.state === RouteState.ROUTE_CANDIDATE);
    const lost_ms = poseBoundedLoss ? POSE_BOUNDED_LOSS_MS : INTERFERENCE_TIMEOUT_MS;
    const lost_cm = poseBoundedLoss ? POSE_BOUNDED_LOSS_CM : INTERFERENCE_MAX_DISTANCE_CM;
    const gray_loss_expired = this.abnormal && !component_crossing &&
      (now_ms - this.last_reliable_ms >= lost_ms ||
        Math.abs(this.odometry.distance_cm - this.abnormal_origin_cm) >= lost_cm);
    const route_owned_gray_loss = this.routeOwnsGrayLoss();
    const turn_target = this.route.valid && this.route.index < this.route.count ? this.route.turnTargetRad() : 0;
    const completing_turn = previous_route === RouteState.ROUTE_CLEARING &&
      !this.route.passing_junction && this.route.currentAction() !== Action.STRAIGHT;
    const previous_exit_axis = this.route.entry_heading + turn_target;
    const right_angle_handoff = previous_route === RouteState.ROUTE_EXECUTING &&
      !this.route.passing_junction && this.route.steps[this.route.index].action !== Action.HEXAGON &&
      Math.abs(Math.abs(turn_target) - JUNCTION_TURN_TARGET_RAD) < 0.0001;
    this.route.setSpeed(this.odometry.left_speed_cm_s, this.odometry.right_speed_cm_s);
    const wasRecovering = previous_route === RouteState.ROUTE_RECOVERING;
    /* Odometry frame -> track frame. The odometry origin is the START point in
     * the car's initial heading (x forward, y right); the planner's anchored
     * start pose supplies the world placement, so a scenario test may start
     * anywhere and face any direction. */
    const startH = this.route.start_heading_rad;
    const cosH = Math.cos(startH), sinH = Math.sin(startH);
    const axleX = this.route.start_x_cm +
      this.odometry.x_cm * cosH - this.odometry.y_cm * sinH;
    const axleY = this.route.start_y_cm +
      this.odometry.x_cm * sinH + this.odometry.y_cm * cosH;
    this.route.update(
      obs,
      this.odometry.distance_cm,
      this.odometry.heading_rad,
      now_ms,
      axleX,
      axleY
    );

    // A real node confirmed on this frame may transfer ownership to its phase.
    if (gray_loss_expired && !route_owned_gray_loss && !this.routeOwnsGrayLoss()) {
      this.stop(LineStopReason.LINE_STOP_LOST);
      return;
    }
    if (this.route.state === RouteState.ROUTE_EXECUTING && !this.route.passing_junction &&
      previous_route !== RouteState.ROUTE_EXECUTING) this.exit_heading_pending = false;
    if (right_angle_handoff && this.route.state === RouteState.ROUTE_CLEARING) {
      this.exit_heading_rad = previous_exit_axis;
      this.exit_heading_origin_cm = this.odometry.distance_cm;
      this.exit_heading_pending = true;
    }
    if (this.exit_heading_pending &&
      Math.abs(this.odometry.distance_cm - this.exit_heading_origin_cm) > GRAY_FORWARD_OFFSET_CM) {
      this.exit_heading_pending = false;
    }
    if (this.exit_heading_pending && this.route.state === RouteState.ROUTE_FOLLOW &&
      this.route.approach_center_count >= LINE_CONFIRM_SAMPLES &&
      (Math.abs(this.odometry.heading_rad - this.exit_heading_rad) <= JUNCTION_PASS_ALIGNED_RAD ||
        Math.abs(this.turn_yaw_rate_rad_s) <= JUNCTION_PASS_SETTLED_YAW_RAD_S)) {
      this.exit_heading_pending = false;
    }
    if (this.route.state === RouteState.ROUTE_CLEARING && previous_route === RouteState.ROUTE_EXECUTING) {
      this.abnormal = false;
      this.exit_accelerating = false;
      if (this.route.line_exit_confirmed) this.have_reliable = true;
      this.last_reliable_ms = now_ms;
    }
    if (completing_turn && this.route.state === RouteState.ROUTE_FOLLOW) this.exit_accelerating = true;
    if (this.route.state === RouteState.ROUTE_FINISHED) {
      this.route.dispatchStopCallback();
      this.stop(LineStopReason.LINE_STOP_ROUTE_COMPLETE);
      return;
    }

    if (this.route.state === RouteState.ROUTE_FAILED) {
      if (!this.route.recoverNavigation(this.odometry.distance_cm, this.odometry.heading_rad, now_ms)) {
        this.stop(LineStopReason.LINE_STOP_ROUTE);
        return;
      }
    }

    // recoverNavigation may enter RECOVERING on this very frame. Clear the
    // borrowed axis after that transition, as in the C controller.
    if (this.route.state === RouteState.ROUTE_RECOVERING && !wasRecovering) {
      this.pid.reset();
      this.abnormal = false;
      this.exit_heading_pending = false;
    }

    const planning_step = this.route.steps[this.route.planningIndex()];
    const step = this.route.steps[this.route.index];
    const cruise = this.route.cruiseSpeed();
    const planning_origin = this.route.planningOrigin();
    const planning_entry_speed = this.route.planningEntrySpeedLimit();
    const entry_speed_limit = this.route.entrySpeedLimit();
    let base_speed = cruise;
    let navigation_wheel_cap = WHEEL_TARGET_MAX_CM_S;

    const passing_without_exit =
      (this.route.passing_junction &&
        this.route.state === RouteState.ROUTE_CLEARING &&
        (this.route.component_recovery_active ||
          !stable ||
          !routeIsClearLine(this.route, obs) ||
          this.route.exit_count < LINE_CONFIRM_SAMPLES)) ||
      (this.route.state === RouteState.ROUTE_CANDIDATE &&
        !this.route.targetReady(this.odometry.distance_cm, true) &&
        (!narrow || Math.abs(obs.position) > 2.0));

    if (passing_without_exit) {
      base_speed = Math.min(base_speed, COMPONENT_RECOVERY_SPEED_CM_S);
      navigation_wheel_cap = Math.min(navigation_wheel_cap, COMPONENT_RECOVERY_SPEED_CM_S);
    }

    if (
      planning_step.segment_length_cm > 0 &&
      this.route.final_segment_active &&
      (this.route.state === RouteState.ROUTE_FOLLOW ||
        this.route.state === RouteState.ROUTE_CANDIDATE ||
        this.route.state === RouteState.ROUTE_CLEARING)
    ) {
      if (!this.braking_active || this.braking_origin_cm !== planning_origin) {
        this.braking_slew.speed = Math.max(this.left_target_cm_s, this.right_target_cm_s);
        this.braking_slew.acceleration = 0;
        this.braking_origin_cm = planning_origin;
        this.braking_active = true;
        this.braking_speed_primed = false;
        this.braking_reverse_ms = 0;
        this.braking_reverse_active = false;
      }
      const exitAdvance = this.route.planningIndex() > 0 ? WHEEL_TRACK_CM : 0;
      const target_dist = Math.max(0, planning_step.segment_length_cm - GRAY_FORWARD_OFFSET_CM - exitAdvance);
      const plan = brakingPlan(
        base_speed,
        Math.min(base_speed, planning_entry_speed),
        target_dist,
        this.odometry.distance_cm - planning_origin,
        Math.max(Math.abs(this.odometry.left_speed_cm_s), Math.abs(this.odometry.right_speed_cm_s)),
        BRAKING_SAFE_DECEL_CM_S2,
        BRAKING_DISTANCE_MARGIN_CM,
        true
      );
      if (!plan.valid) {
        this.stop(LineStopReason.LINE_STOP_ROUTE);
        return;
      }
      navigation_wheel_cap = Math.min(navigation_wheel_cap, plan.allowed_speed);
      this.braking_active = true;
      /* Reverse-brake arming, mirroring line_follow.c: the planner says the
       * passive envelope (BRAKING_SAFE_DECEL_CM_S2) cannot meet the stop point,
       * the line is a stable narrow centre, and both wheels are fast enough that
       * reverse torque cannot push the car backwards.  It is an arming
       * threshold, never a physical stopping boundary; the pump is capped by
       * BRAKING_REVERSE_MAX_MS and cut below BRAKING_REVERSE_MIN_SPEED_CM_S.
       * Gated on the two switches so the default (OFF) build executes nothing
       * new here: the replay output stays bit-identical. */
      if (BRAKING_REVERSE_HARDWARE_CONFIRMED && MOTOR_REVERSE_BRAKE_ENABLED) {
        const braking_speed = Math.max(
          Math.abs(this.odometry.left_speed_cm_s),
          Math.abs(this.odometry.right_speed_cm_s)
        );
        const raw_accel = this.braking_speed_primed
          ? (braking_speed - this.braking_previous_speed_cm_s) / dt
          : 0;
        this.braking_previous_speed_cm_s = braking_speed;
        if (!this.braking_speed_primed) this.braking_accel_cm_s2 = raw_accel;
        else {
          this.braking_accel_cm_s2 +=
            dt / (BRAKING_ACCEL_FILTER_TAU_S + dt) * (raw_accel - this.braking_accel_cm_s2);
        }
        this.braking_speed_primed = true;
        const decision = brakingDecide(
          braking_speed,
          planning_entry_speed,
          plan.remaining_cm,
          -this.braking_accel_cm_s2,
          BRAKING_SAFE_DECEL_CM_S2,
          BRAKING_REVERSE_GUARD_CM
        );
        this.braking_required_cm_s2 = decision.required_decel;
        this.braking_reverse_active = false;
        if (decision.distance_exhausted && braking_speed > planning_entry_speed + REVERSE_BRAKE_MARGIN_CM_S) {
          this.route.navigation_warning = RouteFailure.ROUTE_FAILURE_BRAKE_DISTANCE;
        }
        if (
          decision.reverse_needed &&
          !decision.distance_exhausted &&
          narrow &&
          stable &&
          this.route.state === RouteState.ROUTE_FOLLOW &&
          this.route.steps[this.route.index].cruise_speed >= REVERSE_BRAKE_MIN_CRUISE_CM_S &&
          Math.min(this.odometry.left_speed_cm_s, this.odometry.right_speed_cm_s) >
            BRAKING_REVERSE_MIN_SPEED_CM_S &&
          this.braking_reverse_ms < BRAKING_REVERSE_MAX_MS
        ) {
          const frame_ms = Math.round(dt * 1000);
          if (frame_ms <= BRAKING_REVERSE_MAX_MS - this.braking_reverse_ms) {
            this.braking_reverse_active = true;
          }
        }
      }
      base_speed = brakingSlew(
        this.braking_slew,
        plan.allowed_speed,
        dt,
        TARGET_RISE_CM_S2,
        BRAKING_TARGET_JERK_CM_S3
      );
    } else {
      this.braking_active = false;
      this.braking_reverse_active = false;
    }

    if (this.route.search_pending && planning_step.segment_length_cm > 0) {
      base_speed = Math.min(base_speed, planning_entry_speed);
    }

    // A rolling corner exits on the outgoing wire, so it keeps its arc speed
    // through the clearance and the post-turn phase instead of braking to the
    // pivot exit speed: the asymmetric spin-down of a 60 -> 20 cm/s drop adds
    // several tenths of a radian of yaw on top of the arc's own unwind.
    const rolled_corner = this.route.usesCornerArc() || this.route.arc_just_exited;
    if (this.route.state === RouteState.ROUTE_CLEARING && this.route.currentAction() !== Action.STRAIGHT) {
      base_speed = Math.min(
        base_speed,
        rolled_corner
          ? CORNER_ARC_SPEED_CM_S
          : Math.min(step.turn_speed, JUNCTION_EXIT_SPEED_CM_S)
      );
    }
    // Pose mode keeps the exit speed until the outgoing line is re-centred; the
    // printed symbol beside a corner must not be chased at cruise speed.
    if (this.route.post_turn_slow && !rolled_corner) {
      base_speed = Math.min(base_speed, Math.min(step.turn_speed, JUNCTION_EXIT_SPEED_CM_S));
    }
    // Pose-mode corner approach: pre-slow so the brake phase does not eat the
    // pivot approach distance (the old CANDIDATE deceleration did this). The
    // window grows with speed: v^2 / (2*a_safe) plus the 30 cm base.
    if (this.route.pose_mode) {
      const brake = (this.odometry.left_speed_cm_s * this.odometry.left_speed_cm_s) /
        (2 * BRAKING_SAFE_DECEL_CM_S2);
      if (this.route.pose_gap_bar_cm <= Math.max(ROUTE_WAYPOINT_SLOW_CM, brake + 8.0)) {
        base_speed = Math.min(base_speed, this.route.entrySpeedLimit());
      }
    }

    if (this.route.state === RouteState.ROUTE_CANDIDATE && this.route.targetReady(this.odometry.distance_cm, true)) {
      base_speed = Math.min(base_speed, entry_speed_limit);
    }

    if (this.route.state === RouteState.ROUTE_BRAKING) {
      this.abnormal = false;
      this.pid.reset();
      this.targets(0, 0, dt);
      this.state = LineState.LINE_JUNCTION;
      this.speed.update(0, 0, this.odometry.left_speed_cm_s, this.odometry.right_speed_cm_s, dt);
      this.command = { left_pwm: 0, right_pwm: 0, left_forward: true, right_forward: true, standby: false };
      return;
    }

    if (
      this.route.pose_mode &&
      this.route.pose_cross_valid &&
      this.route.state === RouteState.ROUTE_FOLLOW &&
      step.action !== Action.HEXAGON
    ) {
      // 2026-11-08 field fix: on a straight the lateral correction must be owned
      // by the grey position PID. The old branch treated the pose segment as the
      // control target with the 8-channel reading only trimming it by
      // POSE_GRAY_TRIM=25%, so position=3 produced ~0.16 cm/s of differential
      // (no correction at all): the car followed its IMU heading, drifted off a
      // visible wire and finally lost it into GAP_HOLD (field route_index=0,
      // pose_cross≈-16.5 cm, mask=0x04).
      // Now: grey visible (single narrow line) -> ordinary position PID steering;
      // pose/heading only hold the segment when grey is unusable (loss, wide or
      // multi-region pattern, printed symbol). The gyro only damps yaw jitter; it
      // never generates the straight steering command.
      const trusted = this.grayPlausible(obs);
      const usable = trusted && narrow;
      if (usable) {
        if (stable) {
          this.have_reliable = true;
          this.last_reliable_ms = now_ms;
          this.reliable_position = obs.position;
        }
        // Coming back from an unusable stretch: rebuild the PID history instead
        // of resuming with a stale derivative/integral.
        if (this.abnormal) {
          this.pid.reset();
          this.speed.reset();
        }
        this.abnormal = false;
      } else if (!this.abnormal &&
        Math.abs(this.route.pose_cross_cm) > ROUTE_WAYPOINT_GATE_CM) {
        // The pose itself says the car is off the segment: from here the grey
        // loss is real, so arm the bounded loss window instead of trusting the
        // path follower.
        this.abnormal = true;
        this.abnormal_origin_cm = this.odometry.distance_cm;
      }
      if (usable) {
        const base = base_speed * (1.0 - (POSITION_CURVE_SLOWDOWN * Math.abs(obs.position)) / 7.0);
        const next = this.nextOuter(base, dt);
        const pid = this.pid.update(obs.position, dt, 0.5 * next);
        if (!pid.success) {
          this.stop(LineStopReason.LINE_STOP_FAULT);
          return;
        }
        // Yaw-rate damping only presses the measured yaw rate back to zero; the
        // lateral error is always decided by the grey PID.
        const correction = pid.output -
          (this.inCornerExit()
            ? CORNER_ARC_EXIT_DAMPING * this.turn_yaw_rate_rad_s
            : STRAIGHT_YAW_DAMPING * this.turn_yaw_rate_rad_s);
        this.state = LineState.LINE_TRACKING;
        this.outerTargets(base, correction, dt);
      } else {
        // Grey unusable: hold the pose segment direction and pull the car back
        // onto that segment with the cross-track term (inside the bounded window
        // while a printed symbol hides the wire or the narrow line is lost).
        const cross = this.route.pose_cross_cm;
        const desired = this.route.pose_seg_heading_rad +
          Math.atan2(-cross, POSE_PURSUIT_LOOKAHEAD_CM);
        let corr = HEADING_KP_CM_S_PER_RAD *
          Math.atan2(Math.sin(desired - this.odometry.heading_rad),
            Math.cos(desired - this.odometry.heading_rad));
        // Right after a rolling corner the released differential keeps yawing the
        // car for ~0.1-0.3 s; damping the measured yaw rate stops the drift much
        // sooner than the cross-track term alone.
        if (this.inCornerExit()) {
          corr -= CORNER_ARC_EXIT_DAMPING * this.turn_yaw_rate_rad_s;
        }
        const cap = 0.5 * Math.max(base_speed, 1);
        corr = Math.max(-cap, Math.min(cap, corr));
        this.state = LineState.LINE_GAP_HOLD;
        this.pid.reset();
        this.outerTargets(base_speed, corr, dt);
      }
    } else if (this.route.state === RouteState.ROUTE_CANDIDATE && !this.route.targetReady(this.odometry.distance_cm, true)) {
      this.state = LineState.LINE_JUNCTION;
      this.abnormal = false;
      this.pid.reset();
      this.outerTargets(
        base_speed,
        HEADING_KP_CM_S_PER_RAD * (this.passageHeading(this.route.entry_heading) - this.odometry.heading_rad),
        dt
      );
    } else if (this.route.state === RouteState.ROUTE_RECOVERING) {
      this.state = LineState.LINE_JUNCTION;
      this.abnormal = false;
      if (narrow && stable) {
        const res = this.pid.update(obs.position, dt, NAVIGATION_RECOVERY_SPEED_CM_S * 0.5);
        this.outerTargets(NAVIGATION_RECOVERY_SPEED_CM_S, res.output, dt);
      } else {
        this.pid.reset();
        this.outerTargets(
          NAVIGATION_RECOVERY_SPEED_CM_S,
          HEADING_KP_CM_S_PER_RAD * (this.route.entry_heading - this.odometry.heading_rad) -
            JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s,
          dt
        );
      }
    } else if (this.route.component_recovery_active) {
      this.state = LineState.LINE_JUNCTION;
      this.abnormal = false;
      this.exit_accelerating = false;
      this.pid.reset();
      this.outerTargets(
        this.route.passing_junction ? base_speed : Math.min(step.turn_speed, COMPONENT_RECOVERY_SPEED_CM_S),
        HEADING_KP_CM_S_PER_RAD * ((this.route.passing_junction ?
          this.passageHeading(this.route.component_recovery_heading) :
          this.route.component_recovery_heading) - this.odometry.heading_rad) -
          JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s,
        dt
      );
    } else if (this.route.state === RouteState.ROUTE_EXECUTING && step.action === Action.HEXAGON) {
      const x = this.route.hexagon;
      this.state = LineState.LINE_JUNCTION;
      this.abnormal = false;
      this.exit_accelerating = false;
      if (previous_route !== RouteState.ROUTE_EXECUTING) this.pid.reset();
      const outer = Math.min(step.turn_speed, Math.min(HEXAGON_SPEED_CM_S, JUNCTION_SETTLE_SPEED_CM_S));
      let correction = 0;
      if (x.turning) {
        // Pose-mode ring: pivot to the next 30/60 degree vertex heading.  The
        // saturated proportional term reproduces the track/2 pivot radius the
        // rounded-corner polygon assumes; damping kills the residual rate.
        this.pid.reset();
        const err = x.target_heading - this.odometry.heading_rad;
        correction =
          JUNCTION_SETTLE_HEADING_KP * err - JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s;
        correction = Math.max(-0.5 * outer, Math.min(0.5 * outer, correction));
        this.outerTargets(outer, correction, dt);
      } else if (x.use_line) {
        const res = this.pid.update(obs.position, dt, 0.5 * outer);
        correction = res.output - JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s;
        this.outerTargets(outer, correction, dt);
      } else {
        this.pid.reset();
        correction =
          HEADING_KP_CM_S_PER_RAD * (x.target_heading - this.odometry.heading_rad) -
          JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s;
        // Inside-ring legs use the leg cross-track; the entry approach still
        // uses the incoming segment's cross-track so a drifted approach lands
        // on V0 instead of beside it.
        if (this.route.pose_mode) {
          correction -= JUNCTION_CROSS_KP * (x.cross_cm !== 0
            ? x.cross_cm : this.route.pose_cross_cm);
        }
        correction = Math.max(-0.5 * outer, Math.min(0.5 * outer, correction));
        this.outerTargets(outer, correction, dt);
      }
    } else if (this.route.state === RouteState.ROUTE_EXECUTING) {
      this.exit_accelerating = false;
      if (previous_route !== RouteState.ROUTE_EXECUTING) {
        this.pid.reset();
        this.speed.reset();
      }
      this.abnormal = false;
      this.state = LineState.LINE_JUNCTION;
      const direction = this.route.currentAction() === Action.LEFT ? -1.0 : 1.0;
      const target = this.route.turnTargetRad();
      const turn = direction * (this.odometry.heading_rad - this.route.entry_heading);
      const outer =
        this.route.currentAction() === Action.STRAIGHT
          ? step.turn_speed
          : Math.min(step.turn_speed, JUNCTION_SETTLE_SPEED_CM_S);
      const remaining = Math.abs(target) - turn;
      const band = Math.min(JUNCTION_SETTLE_BAND_RAD, 0.4 * Math.abs(target));
      const settling = this.route.currentAction() !== Action.STRAIGHT && band > 0 && remaining <= band;
      let base = 0.5 * outer;
      let correction = direction * base;

      if (this.route.turn_approaching) {
        this.pid.reset();
        this.straightTargets(outer,
          HEADING_KP_CM_S_PER_RAD * (this.route.action_start_heading - this.odometry.heading_rad), dt);
      } else if (this.route.pose_mode && this.route.usesCornerArc()) {
        // Rolling corner: hold the axle at the arc speed and set the wheel
        // differential to v*track/(2R). The feed-forward differential makes the
        // radius; the heading error against the tangent (route supplies the
        // desired heading from the circle, so this is real path following)
        // steers back onto the circle. The inner wheel keeps rolling.
        this.pid.reset();
        const axle = CORNER_ARC_SPEED_CM_S;
        const err = Math.atan2(
          Math.sin(this.route.arc_desired_heading_rad - this.odometry.heading_rad),
          Math.cos(this.route.arc_desired_heading_rad - this.odometry.heading_rad)
        );
        let differential = direction * this.route.arc_diff_cm_s + CORNER_ARC_HEADING_KP * err;
        differential = Math.max(-0.9 * axle, Math.min(0.9 * axle, differential));
        this.left_target_cm_s = Math.max(0, Math.min(WHEEL_TARGET_MAX_CM_S, axle + differential));
        this.right_target_cm_s = Math.max(0, Math.min(WHEEL_TARGET_MAX_CM_S, axle - differential));
        this.base = 0.5 * (this.left_target_cm_s + this.right_target_cm_s);
        this.correction = 0.5 * (this.left_target_cm_s - this.right_target_cm_s);
        this.startup_elapsed_s = Math.min(STARTUP_RAMP_TIME_S, this.startup_elapsed_s + dt);
        this.exit_accelerating = true;
      } else if (this.route.pose_mode && this.route.currentAction() !== Action.STRAIGHT) {
        // Pose-mode pivot: saturated heading feedback towards the exact corner
        // heading. The grey line is deliberately not used inside the turn, so a
        // printed symbol or a dead-end short leg cannot pull the pivot.
        this.pid.reset();
        const targetHeading = this.route.entry_heading + direction * Math.abs(target);
        const err = Math.atan2(
          Math.sin(targetHeading - this.odometry.heading_rad),
          Math.cos(targetHeading - this.odometry.heading_rad)
        );
        // Bang-bang pivot down to 0.15 rad, then damped settling: keeping the
        // saturated rate early stops the arc from ballooning (a line-following
        // 90 degree turn travels >20 cm forward and misses the short leg).
        let corr: number;
        if (Math.abs(err) > 0.15) {
          corr = err > 0 ? base : -base;
        } else {
          corr = JUNCTION_SETTLE_HEADING_KP * err - 1.2 * this.turn_yaw_rate_rad_s;
          corr = Math.max(-base, Math.min(base, corr));
        }
        this.turnTargets(outer, corr, dt);
      } else if (this.route.usesCornerArc() && !this.route.corner_arc_complete &&
        !settling && !this.route.capture_line_seen) {
        // Legacy grey-only rolling corner (POSE_NAV_ENABLED=0 build): same law
        // as firmware line_follow.c, with the fixed CORNER_RADIUS_CM. Pose mode
        // uses the table-driven arc above instead.
        const track = this.odometry.calibration.track_cm;
        if (CORNER_RADIUS_CM <= track * 0.5) {
          this.stop(LineStopReason.LINE_STOP_CALIBRATION);
          return;
        }
        base = outer / (1 + track / (2 * CORNER_RADIUS_CM));
        const lead = GRAY_FORWARD_OFFSET_CM - CORNER_RADIUS_CM;
        const planned = Math.max(0, Math.min(Math.abs(target),
          (this.odometry.distance_cm - this.route.entry_cm - lead) / CORNER_RADIUS_CM));
        correction = direction * (base * track / (2 * CORNER_RADIUS_CM) +
          CORNER_HEADING_KP * (planned - turn));
        this.turnTargets(outer, correction, dt);
      } else if (this.route.currentAction() === Action.STRAIGHT) {
        const pass = Math.min(base_speed, Math.max(outer, JUNCTION_PASS_SPEED_CM_S));
        this.straightTargets(
          pass,
          HEADING_KP_CM_S_PER_RAD * (this.route.entry_heading - this.odometry.heading_rad),
          dt
        );
      } else {
        if (this.route.turn_capturing || this.route.capture_line_seen || settling) {
          let line = 0;
          if (this.route.turn_capturing) {
            const res = this.pid.update(obs.position, dt, base);
            line = res.output;
          } else {
            this.pid.reset();
          }
          const heading_kp = settling ? JUNCTION_SETTLE_HEADING_KP : JUNCTION_CAPTURE_HEADING_KP;
          correction = this.route.turn_capturing
            ? line : heading_kp * (this.route.entry_heading + target - this.odometry.heading_rad);
          if (this.route.turn_capturing && Math.abs(obs.position) < 0.01 &&
              Math.abs(line) < 0.05 && turn < Math.abs(target))
            correction = direction * Math.min(base, JUNCTION_CAPTURE_CENTER_BIAS_CM_S);
          if (!this.route.turn_capturing && turn < Math.abs(target))
            correction = direction * Math.max(direction * correction,
              Math.min(base, JUNCTION_CAPTURE_MIN_TURN_CM_S));
          correction -= JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s;
        } else {
          this.pid.reset();
        }
        this.turnTargets(outer, correction, dt);
      }
    } else if (this.route.state === RouteState.ROUTE_CLEARING && this.route.next_junction_samples > 0) {
      this.state = LineState.LINE_JUNCTION;
      this.abnormal = false;
      this.pid.reset();
      this.outerTargets(
        Math.min(base_speed, JUNCTION_SETTLE_SPEED_CM_S),
        HEADING_KP_CM_S_PER_RAD * (this.route.line_exit_heading_rad - this.odometry.heading_rad),
        dt
      );
    } else if (this.route.state === RouteState.ROUTE_CLEARING && this.route.currentAction() === Action.STRAIGHT) {
      this.state = LineState.LINE_JUNCTION;
      this.abnormal = false;
      const pass = base_speed;
      if (routeIsClearLine(this.route, obs) && stable && this.route.exit_count >= LINE_CONFIRM_SAMPLES) {
        const res = this.pid.update(obs.position, dt, 0.5 * pass);
        this.outerTargets(pass, res.output, dt);
      } else {
        this.pid.reset();
        this.outerTargets(
          pass,
          HEADING_KP_CM_S_PER_RAD * ((this.route.passing_junction ?
            this.passageHeading(this.route.entry_heading) : this.route.entry_heading) - this.odometry.heading_rad),
          dt
        );
      }
    } else if (narrow && (stable || !this.abnormal) && this.route.state !== RouteState.ROUTE_CANDIDATE) {
      if (this.abnormal) {
        this.pid.reset();
        this.speed.reset();
        this.abnormal = false;
        this.component_cross_latched = false;
      }
      this.state = LineState.LINE_TRACKING;
      if (stable) {
        this.have_reliable = true;
        this.last_reliable_ms = now_ms;
        this.reliable_position = obs.position;
      }
      const base = base_speed * (1.0 - (POSITION_CURVE_SLOWDOWN * Math.abs(obs.position)) / 7.0);
      const next = this.nextOuter(base, dt);
      const trusted = this.grayPlausible(obs);
      const res = trusted ? this.pid.update(obs.position, dt, 0.5 * next) : null;
      let correction = res ? res.output
        : HEADING_KP_CM_S_PER_RAD * (this.route.pose_bearing_rad - this.odometry.heading_rad);
      if (!trusted) {
        this.pid.reset();
        this.state = LineState.LINE_GAP_HOLD;
      }
      if (this.route.state === RouteState.ROUTE_CLEARING && this.route.currentAction() !== Action.STRAIGHT) {
        correction -= JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s;
      }
      this.outerTargets(base, correction, dt);
    } else {
      if (!this.abnormal) {
        this.abnormal = true;
        // Hold along the known segment, not towards the corner: on a
        // zero-length row the corner is behind the car and the bearing would
        // turn it around.
        this.hold_heading_rad = this.route.pose_mode && this.route.state === RouteState.ROUTE_FOLLOW
          ? (this.route.pose_cross_valid
            ? this.route.pose_seg_heading_rad
            : this.route.pose_bearing_rad)
          : this.odometry.heading_rad;
        if (
          this.route.state === RouteState.ROUTE_CLEARING &&
          this.route.line_exit_confirmed &&
          this.route.currentAction() !== Action.STRAIGHT
        ) {
          // Counteract the early pivot: rewind toward the inside of the corner so
          // the exit line comes back inside the sensor bar.
          const turnSign = this.route.turnTargetRad() < 0 ? -1 : 1;
          this.hold_heading_rad =
            this.route.line_exit_heading_rad - turnSign * JUNCTION_EXIT_REWIND_RAD;
        }
        this.abnormal_origin_cm = this.odometry.distance_cm;
      }
      this.state = LineState.LINE_GAP_HOLD;
      const base =
        obs.kind === GrayKind.GRAY_NONE
          ? Math.min(base_speed, HOLD_SPEED_CM_S)
          : Math.min(base_speed, AMBIGUOUS_SPEED_CM_S);
      let correction = HEADING_KP_CM_S_PER_RAD * (this.hold_heading_rad - this.odometry.heading_rad);
      // Pose-mode cross-track hold: a heading-only hold drives parallel to the
      // segment and never comes back, so add the lateral term while the wire is
      // hidden by a printed component or rejected by the plausibility gate.
      if (this.route.pose_mode && this.route.pose_cross_valid &&
        this.route.state === RouteState.ROUTE_FOLLOW) {
        correction -= JUNCTION_CROSS_KP * this.route.pose_cross_cm;
        correction = Math.max(-0.5 * base, Math.min(0.5 * base, correction));
      }
      if (
        (this.route.state === RouteState.ROUTE_CLEARING || this.route.arc_just_exited) &&
        this.route.line_exit_confirmed &&
        this.route.currentAction() !== Action.STRAIGHT
      ) {
        correction -= CORNER_ARC_EXIT_DAMPING * this.turn_yaw_rate_rad_s;
        this.outerTargets(base, correction, dt);
      } else {
        this.targets(base, correction, dt);
      }
    }

    const peak_target = Math.max(this.left_target_cm_s, this.right_target_cm_s);
    if (peak_target > navigation_wheel_cap && peak_target > 0) {
      const ratio = navigation_wheel_cap / peak_target;
      this.left_target_cm_s *= ratio;
      this.right_target_cm_s *= ratio;
      this.base = 0.5 * (this.left_target_cm_s + this.right_target_cm_s);
      this.correction = 0.5 * (this.left_target_cm_s - this.right_target_cm_s);
    }

    /* The reverse output is only valid while both wheels are measurably faster
     * than their targets; once the PID has caught up (or the line is no longer
     * a stable narrow centre) it must be released, otherwise the negative torque
     * would start pushing the car backwards. */
    if (
      this.state !== LineState.LINE_TRACKING ||
      Math.min(
        this.odometry.left_speed_cm_s - this.left_target_cm_s,
        this.odometry.right_speed_cm_s - this.right_target_cm_s
      ) <= REVERSE_BRAKE_MARGIN_CM_S
    ) {
      this.braking_reverse_active = false;
    }
    if (this.braking_reverse_active) {
      this.braking_reverse_ms += Math.round(dt * 1000);
    }
    const { success, command } = this.speed.update(
      this.left_target_cm_s,
      this.right_target_cm_s,
      this.odometry.left_speed_cm_s,
      this.odometry.right_speed_cm_s,
      dt,
      this.braking_reverse_active && this.state === LineState.LINE_TRACKING && narrow
    );

    if (!success) {
      this.stop(LineStopReason.LINE_STOP_FAULT);
      return;
    }

    this.command = command;
  }

  public haveRecentPosition(now_ms: number): boolean {
    return this.have_reliable && now_ms - this.last_reliable_ms < INTERFERENCE_TIMEOUT_MS;
  }
}
