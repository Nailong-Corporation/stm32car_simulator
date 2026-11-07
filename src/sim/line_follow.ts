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
  CONTROL_MAX_INTERVAL_MS,
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
  CORNER_ENTRY_DECEL_CM_S2,
  CORNER_ARC_RADIUS_CM,
  CORNER_ARC_SPEED_CM_S,
  CORNER_ARC_HEADING_KP,
  CORNER_ARC_DIFF_SLEW_CM_S2,
  CORNER_ARC_EXIT_DAMPING,
  GRAY_FORWARD_OFFSET_CM,
  WHEEL_TRACK_CM,
  JUNCTION_EXIT_REWIND_RAD,
  STEERING_FEEDBACK_MIN_CM_S,
  STEERING_FEEDBACK_MIN_HALF_CM_S,
  ROUTE_START_Y_CM,
  ROUTE_WAYPOINT_SLOW_CM,
  JUNCTION_POSE_GRAY_SPREAD,
  JUNCTION_CENTER_LIMIT,
  JUNCTION_STRAIGHT_EXIT_MAX_WIDTH,
  JUNCTION_CROSS_KP,
  POSE_PURSUIT_LOOKAHEAD_CM,
  POSE_GRAY_TRIM,
  ROUTE_WAYPOINT_GATE_CM,
  HEXAGON_TURN_TOL_RAD,
} from './config';
import { OdometryModel, defaultOdometryCalibration } from './odometry';
import { RoutePlannerModel, routeIsNarrowLine, routeIsClearLine } from './route';
import { SpeedControlModel } from './speed_control';
import { Pid } from './pid';
import { grayIsParallelPair } from './gray_sensor';
import { BrakingSlewModel, brakingPlan, brakingDecide, brakingSlew } from './braking_planner';

/** True while the grayscale pattern matches a printed component symbol
 * (hollow resistor interior / symmetric wide graphic band). */
export function grayIsComponentPattern(o: GrayObservation): boolean {
  if (grayIsParallelPair(o)) return false;
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
    if (outer >= requested) this.exit_accelerating = false;
    correction = Math.max(-outer * 0.5, Math.min(outer * 0.5, correction));
    const mean = outer - Math.abs(correction);
    this.left_target_cm_s = mean + correction;
    this.right_target_cm_s = mean - correction;
    if (Math.abs(correction) >= STEERING_FEEDBACK_MIN_HALF_CM_S && outer > 0) {
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

  /** A grey reading is trusted only when its lateral offset agrees with the
   * pose's offset from the active segment line (+-3 cm). A printed transistor
   * branch or resistor arm reads like a line but sits where the pose says the
   * car is centred, so it is rejected and the pose bearing takes over. */
  private grayPlausible(obs: GrayObservation): boolean {
    if (!this.route.pose_mode || !this.route.pose_cross_valid) return true;
    // A centred narrow line is unambiguous: after a fast pivot the pose can be
    // several cm off, and rejecting this reading would leave the car without
    // the very line it needs to recover on.
    if (routeIsNarrowLine(obs) && Math.abs(obs.position) <= JUNCTION_CENTER_LIMIT) return true;
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

  /** C line_follow_have_recent_position: a reliable sample decays after 400 ms. */
  public haveRecentPosition(now_ms: number): boolean {
    return this.have_reliable && now_ms - this.last_reliable_ms < INTERFERENCE_TIMEOUT_MS;
  }

  private lockedStop(): boolean {
    return this.stop_reason === LineStopReason.LINE_STOP_FAULT ||
      this.stop_reason === LineStopReason.LINE_STOP_ENCODER ||
      this.stop_reason === LineStopReason.LINE_STOP_IMU ||
      this.stop_reason === LineStopReason.LINE_STOP_TILT;
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

    // Mirror line_follow.c input guards: an invalid sample is a latched fault,
    // never a recovered reading.
    if (
      !Number.isFinite(dt) || dt <= 0 || dt > CONTROL_MAX_INTERVAL_MS / 1000 ||
      (observation.kind !== GrayKind.GRAY_VALID &&
        observation.kind !== GrayKind.GRAY_NONE &&
        observation.kind !== GrayKind.GRAY_AMBIGUOUS) ||
      (observation.kind === GrayKind.GRAY_VALID &&
        (!Number.isFinite(observation.position) || Math.abs(observation.position) > 7.0))
    ) {
      this.stop(LineStopReason.LINE_STOP_FAULT);
      return;
    }

    // IMU fallback latch: once the heading source degrades to the encoders it
    // stays there (motion_config.h / line_follow.c).
    let imu_usable = imu_valid && Number.isFinite(imu_heading_rad);
    if (this.heading_source === HeadingSource.HEADING_IMU && !imu_usable) {
      this.heading_source = HeadingSource.HEADING_ENCODER;
    }
    imu_usable = imu_usable && this.heading_source === HeadingSource.HEADING_IMU;

    const previous_heading = this.odometry.heading_rad;
    const previous_route = this.route.state;

    if (!this.odometry.updateWithHeading(counts, dt, imu_usable, imu_heading_rad)) {
      this.stop(LineStopReason.LINE_STOP_ENCODER);
      return;
    }

    if (dt > 0) {
      const yaw_rate = (this.odometry.heading_rad - previous_heading) / dt;
      const alpha = dt / (JUNCTION_YAW_FILTER_TAU_S + dt);
      this.turn_yaw_rate_rad_s += alpha * (yaw_rate - this.turn_yaw_rate_rad_s);
    }

    const obs = observation;
    const narrow = routeIsNarrowLine(obs) || grayIsParallelPair(obs);

    if (narrow) {
      if (this.valid_count < LINE_CONFIRM_SAMPLES) this.valid_count++;
    } else {
      this.valid_count = 0;
    }
    const stable = this.valid_count >= LINE_CONFIRM_SAMPLES;

    if (this.state === LineState.LINE_IDLE || this.state === LineState.LINE_STOPPED) {
      if (start_request) {
        if (this.lockedStop() || !stable || !routeIsNarrowLine(obs)) {
          return;
        }
        if (!this.route.valid) {
          this.stop(LineStopReason.LINE_STOP_ROUTE);
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
        this.route.start(this.odometry.distance_cm, now_ms);
      } else {
        return;
      }
    }


    // Printed component graphics (hollow resistors, transistor V, battery
    // plates) must be crossable. Latch the crossing window as soon as the
    // component signature appears; while latched, hold-and-cross is allowed for
    // a bounded distance/time. A real line loss or an unrelated ambiguity that
    // was never preceded by a component signature keeps 400 ms / 8 cm.
    // 2026-11-07 hardware: do not wait for `abnormal`. A one-sided resistor
    // stroke (1-4 channels, position 3-6) already looks like a valid offset
    // line; waiting lets the PID drag the car onto the artwork edge.
    if (!this.component_cross_latched && grayIsComponentPattern(obs)) {
      this.component_cross_latched = true;
      this.component_cross_ms = now_ms;
      this.component_cross_cm = this.odometry.distance_cm;
    }
    if (stable && (grayIsParallelPair(obs) ||
      (routeIsNarrowLine(obs) && obs.white_count <= JUNCTION_STRAIGHT_EXIT_MAX_WIDTH &&
        Math.abs(obs.position) <= JUNCTION_CENTER_LIMIT))) this.component_cross_latched = false;
    const component_visible = this.component_cross_latched &&
      now_ms - this.component_cross_ms < COMPONENT_CROSS_MS &&
      Math.abs(this.odometry.distance_cm - this.component_cross_cm) < COMPONENT_CROSS_MAX_CM;
    const component_crossing = this.abnormal && component_visible;
    // Pose mode: a printed component hides the wire for up to ~10 cm along the
    // segment, and the pose says the car is still on that segment, so reuse the
    // bounded component-crossing window instead of the plain 400 ms / 8 cm cut.
    // A genuine line loss always returns to the plain cut-off.
    const poseBoundedLoss = this.route.pose_mode && component_crossing &&
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

    const planning_index = this.route.planningIndex();
    const planning_step = this.route.steps[planning_index];
    const planning_seen = planning_index === this.route.index ? this.route.junctions_seen : 0;
    const target_approach = planning_step.junction_count <= 1 ||
      planning_seen + 1 >= planning_step.junction_count ||
      (planning_step.segment_length_cm > 0 &&
        this.odometry.distance_cm - this.route.planningOrigin() >= planning_step.segment_length_cm);
    const step = this.route.steps[this.route.index];
    const cruise = this.route.cruiseSpeed();
    const planning_origin = this.route.planningOrigin();
    const planning_entry_speed = this.route.planningEntrySpeedLimit();
    const entry_speed_limit = this.route.entrySpeedLimit();
    let base_speed = cruise;
    let navigation_wheel_cap = WHEEL_TARGET_MAX_CM_S;

    const passing_without_exit = this.route.passing_junction &&
      this.route.state === RouteState.ROUTE_CLEARING && obs.line_mask === 0;

    if (passing_without_exit) {
      base_speed = Math.min(base_speed, COMPONENT_RECOVERY_SPEED_CM_S);
      navigation_wheel_cap = Math.min(navigation_wheel_cap, COMPONENT_RECOVERY_SPEED_CM_S);
    }

    if (
      target_approach && planning_step.segment_length_cm > 0 &&
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
      // Reverse torque remains gated by the two hardware switches; the passive
      // braking decision above is always evaluated (mirror of line_follow.c).
      if (BRAKING_REVERSE_HARDWARE_CONFIRMED && MOTOR_REVERSE_BRAKE_ENABLED) {
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

    if (target_approach && this.route.search_pending && planning_step.segment_length_cm > 0) {
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
      // 2026-11-07 hardware: the real coast-down is ~100 cm/s^2, so sizing the
      // pre-slow window with the planner's 250 cm/s^2 starts braking too late.
      // Also use the remaining table distance, which does not depend on a
      // possibly drifted pose estimate.
      const brakeW = (this.odometry.left_speed_cm_s * this.odometry.left_speed_cm_s) /
        (2 * CORNER_ENTRY_DECEL_CM_S2);
      const slowWindow = Math.max(ROUTE_WAYPOINT_SLOW_CM, brakeW + 8.0);
      let segmentLeft = Infinity;
      if (step.segment_length_cm > 0) {
        segmentLeft = step.segment_length_cm -
          (this.odometry.distance_cm - this.route.origin_cm);
      }
      if (target_approach &&
        (this.route.pose_gap_bar_cm <= slowWindow || segmentLeft <= slowWindow)) {
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
      step.action !== Action.HEXAGON &&
      !component_visible
    ) {
      // 2026-11-07 hardware (mirror of line_follow.c): pose is only a fallback.
      // Whenever the white line is usable, steer with the position PID. The old
      // pose-primary trim made an encoder-heading calibration error drive the
      // car off the wire even though the 8-channel bar still saw it.
      const usable = narrow && (stable || !this.abnormal) &&
        !grayIsComponentPattern(obs);
      if (usable) {
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
        const res = this.pid.update(obs.position, dt, 0.5 * next);
        let correction = res.output;
        if (this.route.arc_just_exited)
          correction -= CORNER_ARC_EXIT_DAMPING * this.turn_yaw_rate_rad_s;
        this.outerTargets(next, correction, dt);
      } else {
        // Line unusable: bounded pose fallback, and really arm the loss window.
        if (!this.abnormal) {
          this.abnormal = true;
          this.abnormal_origin_cm = this.odometry.distance_cm;
        }
        const cross = this.route.pose_cross_cm;
        const desired = this.route.pose_seg_heading_rad +
          Math.atan2(-cross, POSE_PURSUIT_LOOKAHEAD_CM);
        let corr = HEADING_KP_CM_S_PER_RAD *
          Math.atan2(Math.sin(desired - this.odometry.heading_rad),
            Math.cos(desired - this.odometry.heading_rad));
        if (this.route.arc_just_exited)
          corr -= CORNER_ARC_EXIT_DAMPING * this.turn_yaw_rate_rad_s;
        const cap = 0.5 * Math.max(base_speed, 1.0);
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
    } else if (this.route.turn_exit_search) {
      this.state = LineState.LINE_JUNCTION;
      this.abnormal = false;
      this.exit_accelerating = false;
      this.pid.reset();
      const outer = Math.min(step.turn_speed, COMPONENT_RECOVERY_SPEED_CM_S);
      const err = Math.atan2(
        Math.sin(this.route.line_exit_heading_rad - this.odometry.heading_rad),
        Math.cos(this.route.line_exit_heading_rad - this.odometry.heading_rad)
      );
      let correction = HEADING_KP_CM_S_PER_RAD * err - JUNCTION_YAW_DAMPING * this.turn_yaw_rate_rad_s;
      correction = Math.max(-.5 * outer, Math.min(.5 * outer, correction));
      this.outerTargets(outer, correction, dt);
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
      const base = 0.5 * outer;
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
        if (remaining > 0)
          corr = direction * Math.max(direction * corr, Math.min(base, JUNCTION_CAPTURE_MIN_TURN_CM_S));
        this.turnTargets(outer, corr, dt);
      } else if (this.route.usesCornerArc() && !this.route.corner_arc_complete &&
        !settling && !this.route.capture_line_seen) {
        // Legacy (pose-off) rolling arc: radius-projected geometric differential
        // with a heading-error term, mirror of line_follow.c:955-963.
        const track = this.odometry.calibration.track_cm;
        if (CORNER_RADIUS_CM <= track * 0.5) {
          this.stop(LineStopReason.LINE_STOP_CALIBRATION);
          return;
        }
        const arcBase = outer / (1 + track / (2 * CORNER_RADIUS_CM));
        const lead = GRAY_FORWARD_OFFSET_CM - CORNER_RADIUS_CM;
        const planned = Math.max(0, Math.min(Math.abs(target),
          (this.odometry.distance_cm - this.route.entry_cm - lead) / CORNER_RADIUS_CM));
        correction = direction *
          (arcBase * track / (2 * CORNER_RADIUS_CM) + CORNER_HEADING_KP * (planned - turn));
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
        if (remaining > 0)
          correction = direction * Math.max(direction * correction, Math.min(base, JUNCTION_CAPTURE_MIN_TURN_CM_S));
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
      if (grayIsParallelPair(obs)) {
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
    } else if (this.route.state === RouteState.ROUTE_FOLLOW && component_visible) {
      // 2026-11-07 hardware rule (mirror of line_follow.c): a printed component
      // is treated like a junction for steering — hold straight and never feed
      // its one-sided white stroke to the position PID. The pose cross-track
      // trims the hold when available so blind passage does not drift.
      this.state = LineState.LINE_JUNCTION;
      this.pid.reset();
      const hold = this.route.pose_mode && this.route.pose_cross_valid
        ? this.route.pose_seg_heading_rad : this.odometry.heading_rad;
      let correction = HEADING_KP_CM_S_PER_RAD * (hold - this.odometry.heading_rad);
      if (this.route.pose_mode && this.route.pose_cross_valid) {
        correction -= JUNCTION_CROSS_KP * this.route.pose_cross_cm;
        correction = Math.max(-0.5 * base_speed, Math.min(0.5 * base_speed, correction));
      }
      if (obs.kind === GrayKind.GRAY_NONE) {
        if (!this.abnormal) {
          this.abnormal = true;
          this.abnormal_origin_cm = this.odometry.distance_cm;
        }
      } else {
        this.abnormal = false;
        this.last_reliable_ms = now_ms;
        if (stable) {
          this.have_reliable = true;
          this.reliable_position = obs.position;
        }
      }
      this.outerTargets(base_speed, correction, dt);
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
}
