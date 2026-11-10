/**
 * Track Map Geometry & Electrical Components  (2026-11-06 new track revision)
 *
 * Dimensions: 375.00 cm x 220.00 cm  (new CAD drawing, see car_stm32/docs/evidence/2026-11-06-new-track/)
 *
 * New-version geometry (changed versus the previous 375x220 draft):
 *   - middle horizontal is y = 140.00 on the left half (x = 55..245)
 *     and y = 110.00 on the right half (x = 245..375);
 *     the old single y = 120.00 line no longer exists;
 *   - x = 95 now runs 0..140 and 168.6..220 (old: 0..120 / 170..220);
 *   - x = 245 runs the full height y = 0..220 (the lower 170..220 piece was
 *     missing from the earlier plot and is the junction left of the ring);
 *   - x = 275 ends at y = 110; it must NOT be joined to the hexagon left
 *     vertex (that would create a non-existent spur next to the ring);
 *   - y = 50 extends to the right border (x = 275..375, old 275..345);
 *   - hexagon centre moved to (310, 165), side 35, right vertex (345, 165);
 *   - start / finish battery P sits at (0, 110) on the left border.
 *
 * The white "wires" are the line the car follows.  The PCB also carries
 * component symbols (hollow resistors, transistor V-notches, the supply
 * battery) printed on top of the wires: they are the track's distractor and
 * scoring items, so they are part of the physical line geometry and are
 * sampled by the grayscale sensor exactly like the wires.
 *
 * Component sizes are taken from the official circuit-artwork proportions
 * (component length / track width), i.e. they are expressed in cm on the
 * 375 x 220 cm track.
 */

export interface Point2D {
  x: number;
  y: number;
}

export interface TrackSegment {
  id: string;
  p1: Point2D;
  p2: Point2D;
  dashed?: boolean;
  dashLength?: number;
  gapLength?: number;
  width?: number; // Line width in cm (default TRACK_LINE_WIDTH_CM)
}

export type ComponentType = 'battery' | 'resistor' | 'transistor';

export interface TrackComponent {
  id: string;
  type: ComponentType;
  /** connector / centre point on the wire, cm */
  x: number;
  y: number;
  /** rotation of the through-line direction (canonical +x), radians */
  rotation: number;
  label: string;
  elementNumber?: number;
  /** resistor: outer length along the wire; battery: unused */
  length?: number;
  /** across-wire outer width */
  width?: number;
  /** white stroke thickness of the symbol */
  stroke?: number;
}

export const TRACK_WIDTH_CM = 375.0;
export const TRACK_HEIGHT_CM = 220.0;
export const TRACK_LINE_WIDTH_CM = 2.8;

/** Component artwork proportions, measured from the official circuit render. */
/* The artwork proportion (component length / 375 cm) gives ~14.5 cm resistors.
 * The transistor mouth is deliberately enlarged to 18 cm (depth 6 cm) so the
 * two arms clear the 8-channel bar when the car passes between them (0x00).
 * The resistor symbol is drawn at that full
 * artwork scale (k = 1.8125 on the previous 55 % calibration: 8.0 * 1.8125 =
 * 14.5 cm): its two long edges then straddle the 8-channel sensor bar so that
 * both side walls are seen by the outermost pairs S0/S1 and S6/S7 while the
 * centre channels S3/S4 stay dark (raw mask 0xC3, the same signature as a
 * parallel wire pair).  This is a SIMULATOR PICTURE SCALE only - it must never
 * be mistaken for a firmware parameter, and it does not touch the transistors,
 * the battery or the track wires.  The symbol keeps the artwork aspect ratio
 * (length : width : stroke = 8.0 : 3.8 : 1.3) so the printed resistor stays
 * recognisable; treat it as a simulator calibration assumption until the
 * physical board is measured (the artwork is bitmap-scaled, not dimensioned). */
export const RESISTOR_LENGTH_CM = 14.5;
export const RESISTOR_WIDTH_CM = 6.8875;
export const RESISTOR_STROKE_CM = 2.35625;
/* Transistor artwork (simulator picture scale, never a firmware parameter).
 *  2026-11-10 用户要求“三极管做大一点”，并按实车现象建模：
 *   - 从两边（V 的两条斜臂之间）通过时，光电管全黑（掩码 0x00）；
 *   - 从正面（基极横条）通过时，8 路全部检测到白线（掩码 0xFF）。
 *  The printed symbol uses the SAME stroke as the track wire
 *  (TRANSISTOR_STROKE_CM = TRACK_LINE_WIDTH_CM = 2.8 cm), and the board patch
 *  below the symbol (insideFootprint / track_renderer board rectangle) stops
 *  exactly at the round stroke caps of the V arms and the base bar, so the two
 *  V ends join the through wire without a black sliver.
 *  Depth 6 cm > outer channel 3.85 + half stroke 1.4 = 5.25 cm, so when the bar
 *  sits between the two V arms the arms are outside all eight channels and the
 *  board patch leaves them dark (0x00 for ~1.4 cm of travel).  The base bar is as long as the V mouth
 *  (BAR_HALF = MOUTH / 2 = 9 cm), so its painted span (+/-10.4 cm) covers every
 *  channel (+/-3.85 cm) when the car crosses along the branch (0xFF). */
export const TRANSISTOR_MOUTH_CM = 18.0; // distance between the two V arms on the wire
export const TRANSISTOR_DEPTH_CM = 6.0; // depth of the V notch towards the branch
export const TRANSISTOR_BAR_HALF_CM = TRANSISTOR_MOUTH_CM / 2; // 9.0
export const TRANSISTOR_STROKE_CM = TRACK_LINE_WIDTH_CM; // 2.8: same width as the track wire
export const BATTERY_PLATE_SPACING_CM = 1.8;
export const BATTERY_LONG_HALF_CM = 4.6;
export const BATTERY_SHORT_HALF_CM = 2.8;
export const BATTERY_STROKE_CM = 1.3;

// Start / Finish Point P (battery on the left border)
export const START_POINT: Point2D = { x: 0.0, y: 110.0 };
export const START_HEADING_RAD = -Math.PI / 2; // Facing North (-y direction)

export const HEXAGON_CENTER: Point2D = { x: 310.0, y: 165.0 };
export const HEXAGON_SIDE_CM = 35.0;
const HEX_DY = (HEXAGON_SIDE_CM * Math.sqrt(3)) / 2; // ~30.3109

export const HEXAGON_VERTICES: Point2D[] = [
  { x: 345.0, y: 165.0 },                      // Right vertex (entry/exit point)
  { x: 327.5, y: 165.0 - HEX_DY },             // Top-right
  { x: 292.5, y: 165.0 - HEX_DY },             // Top-left
  { x: 275.0, y: 165.0 },                      // Left vertex
  { x: 292.5, y: 165.0 + HEX_DY },             // Bottom-left
  { x: 327.5, y: 165.0 + HEX_DY },             // Bottom-right
];

export const TRACK_SEGMENTS: TrackSegment[] = [
  // Outer border (the driven rectangle)
  { id: 'top_border', p1: { x: 0, y: 0 }, p2: { x: 375, y: 0 } },
  { id: 'right_border', p1: { x: 375, y: 0 }, p2: { x: 375, y: 220 } },
  { id: 'bottom_border', p1: { x: 375, y: 220 }, p2: { x: 0, y: 220 } },
  { id: 'left_border_top', p1: { x: 0, y: 0 }, p2: { x: 0, y: 110 } },
  { id: 'left_border_bottom', p1: { x: 0, y: 110 }, p2: { x: 0, y: 220 } },

  // Internal vertical lines
  { id: 'v_55_top', p1: { x: 55, y: 0 }, p2: { x: 55, y: 140 } },
  {
    id: 'v_55_dashed',
    p1: { x: 55, y: 140 },
    p2: { x: 55, y: 220 },
    dashed: true,
    dashLength: 5.0,
    gapLength: 5.0,
  },
  { id: 'v_95_top', p1: { x: 95, y: 0 }, p2: { x: 95, y: 140 } },
  { id: 'v_95_bot', p1: { x: 95, y: 170 }, p2: { x: 95, y: 220 } },
  { id: 'v_150_bot', p1: { x: 150, y: 170 }, p2: { x: 150, y: 220 } },
  { id: 'v_165_mid', p1: { x: 165, y: 40 }, p2: { x: 165, y: 80 } },
  { id: 'v_245', p1: { x: 245, y: 0 }, p2: { x: 245, y: 220 } },
  { id: 'v_275', p1: { x: 275, y: 0 }, p2: { x: 275, y: 110 } },
  { id: 'v_345_top', p1: { x: 345, y: 0 }, p2: { x: 345, y: 50 } },
  { id: 'v_345_to_hex', p1: { x: 345, y: 110 }, p2: { x: 345, y: 165 } },
  { id: 'v_345_from_hex', p1: { x: 345, y: 165 }, p2: { x: 345, y: 220 } },

  // Internal horizontal lines
  { id: 'h_40_mid', p1: { x: 95, y: 40 }, p2: { x: 245, y: 40 } },
  { id: 'h_50_right', p1: { x: 275, y: 50 }, p2: { x: 375, y: 50 } },
  { id: 'h_80_mid', p1: { x: 165, y: 80 }, p2: { x: 245, y: 80 } },
  { id: 'h_140_left', p1: { x: 55, y: 140 }, p2: { x: 245, y: 140 } },
  { id: 'h_110_right', p1: { x: 245, y: 110 }, p2: { x: 375, y: 110 } },
  { id: 'h_170_mid', p1: { x: 95, y: 170 }, p2: { x: 245, y: 170 } },

  // Hexagon ring edges (Counter-clockwise: 0->1->2->3->4->5->0)
  { id: 'hex_0_1', p1: HEXAGON_VERTICES[0], p2: HEXAGON_VERTICES[1] },
  { id: 'hex_1_2', p1: HEXAGON_VERTICES[1], p2: HEXAGON_VERTICES[2] },
  { id: 'hex_2_3', p1: HEXAGON_VERTICES[2], p2: HEXAGON_VERTICES[3] },
  { id: 'hex_3_4', p1: HEXAGON_VERTICES[3], p2: HEXAGON_VERTICES[4] },
  { id: 'hex_4_5', p1: HEXAGON_VERTICES[4], p2: HEXAGON_VERTICES[5] },
  { id: 'hex_5_0', p1: HEXAGON_VERTICES[5], p2: HEXAGON_VERTICES[0] },
];

/**
 * Component inventory of the new track: 19 resistors + 7 transistors +
 * 1 supply battery = 27 printed items (matches the official artwork count).
 * Positions follow the official artwork, snapped onto the new wires.
 * rotation = direction of the through wire (canonical +x); a transistor's
 * branch points to +y in canonical coordinates.
 */
export const CIRCUIT_COMPONENTS: TrackComponent[] = [
  // ---- battery / start-finish P ----
  { id: 'bat1', type: 'battery', x: 0, y: 110, rotation: Math.PI / 2, label: 'BAT (P起点)', elementNumber: 0 },

  // ---- resistors ----
  { id: 'r1', type: 'resistor', x: 0, y: 44, rotation: Math.PI / 2, label: 'R1 (元件1)', elementNumber: 1 },
  { id: 'r2', type: 'resistor', x: 23, y: 0, rotation: 0, label: 'R2 (元件2)', elementNumber: 2 },
  { id: 'r3', type: 'resistor', x: 55, y: 55, rotation: Math.PI / 2, label: 'R3 (元件3)', elementNumber: 3 },
  { id: 'r5', type: 'resistor', x: 161, y: 140, rotation: 0, label: 'R5 (元件5)', elementNumber: 5 },
  { id: 'r8', type: 'resistor', x: 125.5, y: 220, rotation: 0, label: 'R8 (元件8)', elementNumber: 8 },
  { id: 'r9', type: 'resistor', x: 181, y: 220, rotation: 0, label: 'R9 (元件9)', elementNumber: 9 },
  { id: 'r10', type: 'resistor', x: 292, y: 220, rotation: 0, label: 'R10 (元件10)', elementNumber: 10 },
  { id: 'r12', type: 'resistor', x: 375, y: 151, rotation: Math.PI / 2, label: 'R12 (元件12)', elementNumber: 12 },
  { id: 'r14', type: 'resistor', x: 310, y: 165 + HEX_DY, rotation: 0, label: 'R14 (环岛元件14)', elementNumber: 14 },
  { id: 'r15', type: 'resistor', x: 375, y: 70, rotation: Math.PI / 2, label: 'R15 (元件15)', elementNumber: 15 },
  { id: 'r17', type: 'resistor', x: 300.6, y: 0, rotation: 0, label: 'R17 (元件17)', elementNumber: 17 },
  { id: 'r19', type: 'resistor', x: 334, y: 50, rotation: 0, label: 'R19 (元件19)', elementNumber: 19 },
  { id: 'r20', type: 'resistor', x: 196, y: 0, rotation: 0, label: 'R20 (元件20)', elementNumber: 20 },
  { id: 'r21', type: 'resistor', x: 145, y: 0, rotation: 0, label: 'R21 (元件21)', elementNumber: 21 },
  { id: 'r23', type: 'resistor', x: 130, y: 40, rotation: 0, label: 'R23 (元件23)', elementNumber: 23 },
  { id: 'r24', type: 'resistor', x: 206, y: 80, rotation: 0, label: 'R24 (元件24)', elementNumber: 24 },
  { id: 'r25', type: 'resistor', x: 206, y: 40, rotation: 0, label: 'R25 (元件25)', elementNumber: 25 },
  { id: 'r26', type: 'resistor', x: 55, y: 187, rotation: Math.PI / 2, label: 'R26 (虚线元件26)', elementNumber: 26 },
  { id: 'r27', type: 'resistor', x: 0, y: 177, rotation: Math.PI / 2, label: 'R27 (元件27)', elementNumber: 27 },

  // ---- transistors ----
  // (95,0): top wire through, branch down x=95
  { id: 'q22', type: 'transistor', x: 95, y: 0, rotation: 0, label: 'Q22 (三极管22)', elementNumber: 22 },
  // (345,0): top wire through, branch down x=345
  { id: 'q16', type: 'transistor', x: 345, y: 0, rotation: 0, label: 'Q16 (三极管16)', elementNumber: 16 },
  // (275,50): vertical x=275 through, branch east on y=50
  { id: 'q18', type: 'transistor', x: 275, y: 50, rotation: -Math.PI / 2, label: 'Q18 (三极管18)', elementNumber: 18 },
  // (95,140): horizontal y=140 through, branch north on x=95
  { id: 'q4', type: 'transistor', x: 95, y: 140, rotation: Math.PI, label: 'Q4 (节点4)', elementNumber: 4 },
  // (345,110): horizontal y=110 through, branch south x=345
  { id: 'q13', type: 'transistor', x: 345, y: 110, rotation: 0, label: 'Q13 (三极管13)', elementNumber: 13 },
  // (245,140): vertical x=245 through, branch west on y=140
  { id: 'q6', type: 'transistor', x: 245, y: 140, rotation: Math.PI / 2, label: 'Q6 (节点6)', elementNumber: 6 },
  // (150,170): horizontal y=170 through, branch south x=150
  { id: 'q11', type: 'transistor', x: 150, y: 170, rotation: 0, label: 'Q11 (三极管11)', elementNumber: 11 },
];

/**
 * Distance from point (px, py) to line segment (p1 -> p2)
 */
export function pointToSegmentDistance(
  px: number,
  py: number,
  x1: number,
  y1: number,
  x2: number,
  y2: number
): { distance: number; t: number; projX: number; projY: number } {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const lenSq = dx * dx + dy * dy;
  if (lenSq === 0) {
    const d = Math.hypot(px - x1, py - y1);
    return { distance: d, t: 0, projX: x1, projY: y1 };
  }
  let t = ((px - x1) * dx + (py - y1) * dy) / lenSq;
  t = Math.max(0, Math.min(1, t));
  const projX = x1 + t * dx;
  const projY = y1 + t * dy;
  const distance = Math.hypot(px - projX, py - projY);
  return { distance, t, projX, projY };
}

/** Rotate a world point into a component's local frame (u along wire, v across). */
function toLocal(comp: TrackComponent, px: number, py: number): { u: number; v: number } {
  const dx = px - comp.x;
  const dy = py - comp.y;
  const c = Math.cos(comp.rotation);
  const s = Math.sin(comp.rotation);
  return { u: dx * c + dy * s, v: -dx * s + dy * c };
}

function nearSegment(px: number, py: number, ax: number, ay: number, bx: number, by: number, tol: number): boolean {
  const dx = bx - ax;
  const dy = by - ay;
  const lenSq = dx * dx + dy * dy;
  let t = lenSq > 0 ? ((px - ax) * dx + (py - ay) * dy) / lenSq : 0;
  t = Math.max(0, Math.min(1, t));
  const qx = ax + t * dx;
  const qy = ay + t * dy;
  return Math.hypot(px - qx, py - qy) <= tol;
}

/** true if the local point is inside a component's footprint (the wire is hidden there) */
function insideFootprint(comp: TrackComponent, u: number, v: number): boolean {
  if (comp.type === 'resistor') {
    const L = comp.length ?? RESISTOR_LENGTH_CM;
    const W = comp.width ?? RESISTOR_WIDTH_CM;
    const s = comp.stroke ?? RESISTOR_STROKE_CM;
    // The outline stroke is centred on the L x W rectangle (see
    // track_renderer.renderComponent -> strokeRect with lineWidth = s), so the
    // white symbol reaches s / 2 beyond it.  The footprint must include that
    // half-stroke, exactly like the transistor/battery branches above; otherwise
    // the outer half of each long edge is invisible to the grayscale sensor even
    // though it is painted white on screen.
    return Math.abs(u) <= L / 2 + s / 2 && Math.abs(v) <= W / 2 + s / 2;
  }
  if (comp.type === 'transistor') {
    const s = comp.stroke ?? TRANSISTOR_STROKE_CM;
    // The symbol strokes have round caps, so the white V arms / base bar reach
    // s / 2 past their centre lines.  The footprint (the board patch that hides
    // the wire underneath) must stop exactly at that cap: a wider footprint
    // leaves a black sliver between the arm end and the wire it joins.
    const uHalf = Math.max(TRANSISTOR_MOUTH_CM / 2, TRANSISTOR_BAR_HALF_CM) + s / 2;
    return Math.abs(u) <= uHalf && v >= -s / 2 && v <= TRANSISTOR_DEPTH_CM + s / 2;
  }
  // battery: four plates straddling the wire
  return Math.abs(u) <= 2 * BATTERY_PLATE_SPACING_CM + BATTERY_STROKE_CM && Math.abs(v) <= BATTERY_LONG_HALF_CM + BATTERY_STROKE_CM;
}

/** true if the local point is on a component's white stroke */
function onComponentStroke(comp: TrackComponent, u: number, v: number): boolean {
  if (comp.type === 'resistor') {
    const L = comp.length ?? RESISTOR_LENGTH_CM;
    const W = comp.width ?? RESISTOR_WIDTH_CM;
    const s = comp.stroke ?? RESISTOR_STROKE_CM;
    const hu = L / 2;
    const hv = W / 2;
    // four edges of the hollow rectangle
    if (Math.abs(u) <= hu && Math.abs(Math.abs(v) - hv) <= s / 2) return true;
    if (Math.abs(v) <= hv && Math.abs(Math.abs(u) - hu) <= s / 2) return true;
    // rounded joins at the corners
    return Math.hypot(Math.abs(u) - hu, Math.abs(v) - hv) <= s / 2;
  }
  if (comp.type === 'transistor') {
    const s = comp.stroke ?? TRANSISTOR_STROKE_CM;
    const m = TRANSISTOR_MOUTH_CM / 2;
    const d = TRANSISTOR_DEPTH_CM;
    // V arms
    if (nearSegment(u, v, -m, 0, 0, d, s / 2)) return true;
    if (nearSegment(u, v, 0, d, m, 0, s / 2)) return true;
    // base bar (perpendicular to the branch)
    if (nearSegment(u, v, -TRANSISTOR_BAR_HALF_CM, d, TRANSISTOR_BAR_HALF_CM, d, s / 2)) return true;
    return false;
  }
  // battery plates: long/short alternating along the wire
  const s = comp.stroke ?? BATTERY_STROKE_CM;
  const half = [BATTERY_LONG_HALF_CM, BATTERY_SHORT_HALF_CM, BATTERY_LONG_HALF_CM, BATTERY_SHORT_HALF_CM];
  for (let i = 0; i < 4; ++i) {
    const pu = (i - 1.5) * BATTERY_PLATE_SPACING_CM;
    if (Math.abs(u - pu) <= s / 2 && Math.abs(v) <= half[i]) return true;
  }
  return false;
}

/**
 * Full white-pixel test: track wires plus the printed component symbols.
 * Component symbols sit on top of the wires; inside a symbol footprint only
 * the symbol stroke is white (the wire underneath is covered by the board).
 */
export function isWhiteAt(px: number, py: number): boolean {
  for (let i = 0; i < CIRCUIT_COMPONENTS.length; ++i) {
    const comp = CIRCUIT_COMPONENTS[i];
    const { u, v } = toLocal(comp, px, py);
    if (insideFootprint(comp, u, v)) {
      return onComponentStroke(comp, u, v);
    }
  }
  const halfWidth = TRACK_LINE_WIDTH_CM / 2;
  for (let i = 0; i < TRACK_SEGMENTS.length; ++i) {
    const seg = TRACK_SEGMENTS[i];
    const { distance, t } = pointToSegmentDistance(px, py, seg.p1.x, seg.p1.y, seg.p2.x, seg.p2.y);
    if (distance <= halfWidth) {
      if (seg.dashed) {
        const segLen = Math.hypot(seg.p2.x - seg.p1.x, seg.p2.y - seg.p1.y);
        const distAlong = t * segLen;
        const period = (seg.dashLength || 5.0) + (seg.gapLength || 5.0);
        const mod = distAlong % period;
        if (mod > (seg.dashLength || 5.0)) {
          continue;
        }
      }
      return true;
    }
  }
  return false;
}

/**
 * Backwards-compatible helper: checks whether a sensor point detects the line.
 * @param halfWidth retained for API compatibility; the physical line width and
 * component strokes are modelled by isWhiteAt().
 */
export function isSensorOnTrackLine(sx: number, sy: number, halfWidth = TRACK_LINE_WIDTH_CM / 2): boolean {
  void halfWidth;
  return isWhiteAt(sx, sy);
}
