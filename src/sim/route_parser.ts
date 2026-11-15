import { Action, type RouteStartPose, type RouteStep } from './types';

export interface ParsedScheme {
  id: number;
  name: string;
  steps: RouteStep[];
  /* Table-selected start anchor (route_scenario_start_pose). Omitted = the
   * production P(0,110) north anchor from motion_config.h. */
  start_pose?: RouteStartPose;
}

export interface ParseResult {
  activeSchemeId: number;
  schemes: ParsedScheme[];
  activeSteps: RouteStep[];
  error?: string;
}

function parseAction(actStr: string): Action {
  const clean = actStr.trim();
  switch (clean) {
    case 'LEFT':
      return Action.LEFT;
    case 'RIGHT':
      return Action.RIGHT;
    case 'STRAIGHT':
      return Action.STRAIGHT;
    case 'STOP':
      return Action.STOP;
    case 'CUSTOM_TURN':
      return Action.CUSTOM_TURN;
    case 'HEXAGON':
      return Action.HEXAGON;
    default:
      if (!isNaN(Number(clean))) return Number(clean) as Action;
      return Action.STRAIGHT;
  }
}

function numericValue(value: string, defines: Map<string, number>): number {
  const token = value.trim();
  const direct = defines.get(token);
  if (direct !== undefined) return direct;
  const literal = Number(token.replace(/[fF]$/, ''));
  if (Number.isFinite(literal)) return literal;
  /* The firmware table uses small arithmetic expressions for the new
   * turn_approach_cm column, e.g. `TURN_TRANSISTOR_STRAIGHT_CM + 5.0f`.
   * Substitute known macros and evaluate only a numeric expression. */
  const resolved = token.replace(/[A-Za-z_]\w*/g, (name) => {
    const macro = defines.get(name);
    return macro === undefined ? name : String(macro);
  }).replace(/(\d)[fF]\b/g, '$1');
  if (!/^[\d\s.eE+\-*/().]+$/.test(resolved)) return NaN;
  return evaluateNumericExpression(resolved);
}

/* Tiny + - * / parentheses evaluator for the few macro expressions in
 * route_table.c. No eval/Function (the bundled table is parsed in the browser). */
function evaluateNumericExpression(expr: string): number {
  const tokens = expr.match(/(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?|[()+\-*/]/g);
  if (!tokens || tokens.join('') !== expr.replace(/\s+/g, '')) return NaN;
  let at = 0;
  const peek = () => tokens[at];
  const parseFactor = (): number => {
    const token = peek();
    if (token === '+') { ++at; return parseFactor(); }
    if (token === '-') { ++at; return -parseFactor(); }
    if (token === '(') {
      ++at;
      const value = parseExpr();
      if (peek() !== ')') return NaN;
      ++at;
      return value;
    }
    ++at;
    return Number(token);
  };
  const parseTerm = (): number => {
    let value = parseFactor();
    while (peek() === '*' || peek() === '/') {
      const op = peek();
      ++at;
      const rhs = parseFactor();
      value = op === '*' ? value * rhs : value / rhs;
    }
    return value;
  };
  function parseExpr(): number {
    let value = parseTerm();
    while (peek() === '+' || peek() === '-') {
      const op = peek();
      ++at;
      const rhs = parseTerm();
      value = op === '+' ? value + rhs : value - rhs;
    }
    return value;
  }
  const result = parseExpr();
  return at === tokens.length ? result : NaN;
}

function parseStepLine(line: string, defines: Map<string, number>): RouteStep | null {
  const match = line.match(/\{\s*([^,}]+),\s*([^,}]+),\s*([^,}]+),\s*([^,}]+),\s*([^,}]+),\s*([^,}]+),\s*([^,}]+),([^}]*)\}/);
  if (!match) return null;

  const junction_count = numericValue(match[1], defines);
  const turn_speed = numericValue(match[2], defines);
  const action = parseAction(match[3].trim());
  const cruise_speed = numericValue(match[4], defines);
  const angle_deg = numericValue(match[5], defines);
  const corner_arc_enabled = match[6].trim().toLowerCase() === 'true';
  const segment_length_cm = numericValue(match[7], defines);
  // Missing optional columns preserve legacy tables; unresolved tokens stay
  // NaN rather than being interpreted as absent columns.
  const tail = match[8].split(',');
  const short_link_cm = tail[0]?.trim() ? numericValue(tail[0], defines) : 0;
  const turn_approach_cm = tail[1]?.trim() ? numericValue(tail[1], defines) : 0;

  let comment = '';
  const commentMatch = line.match(/\/\*\s*(.*?)\s*\*\//) || line.match(/\/\/\s*(.*)/);
  if (commentMatch) {
    comment = commentMatch[1].trim();
  }

  return {
    junction_count,
    turn_speed,
    action,
    cruise_speed,
    angle_deg,
    corner_arc_enabled,
    segment_length_cm,
    short_link_cm,
    turn_approach_cm,
    comment,
    rawLine: line.trim(),
  };
}

export function parseRouteTableC(sourceCode: string): ParseResult {
  const lines = sourceCode.split(/\r?\n/);
  let activeSchemeId = 1; // default to scheme 1 for full track simulation

  const schemeMacroMatch = sourceCode.match(/#define\s+ROUTE_SCHEME_SELECT\s+(\d+)/);
  if (schemeMacroMatch) {
    activeSchemeId = parseInt(schemeMacroMatch[1], 10);
  }

  const schemes: ParsedScheme[] = [
    { id: 0, name: '方案 0 (基础调试用赛道)', steps: [] },
    { id: 1, name: '方案 1 (正式全元件：走虚线+环岛)', steps: [] },
    { id: 2, name: '方案 2 (不走虚线，走环岛)', steps: [] },
    { id: 3, name: '方案 3 (外框测试圈)', steps: [] },
    { id: 4, name: '方案 4 (走虚线、不进环岛：固件默认)', steps: [] },
    { id: 5, name: '方案 5 (快速版：走虚线+环岛)', steps: [] },
    { id: 6, name: '方案 6 (快速版：不走虚线+环岛)', steps: [] },
    { id: 7, name: '方案 7 (快速版：外框测试圈)', steps: [] },
    { id: 8, name: '方案 8 (环岛单独测试)', steps: [] },
    { id: 9, name: '方案 9 (从方案 4 中段继续)', steps: [] },
  ];

  let currentSchemeId: number | null = null;
  let inRouteArray = false;
  let inBlockComment = false;
  const customSteps: RouteStep[] = [];
  const defines = new Map<string, number>();

  for (let i = 0; i < lines.length; ++i) {
    const line = lines[i];
    /* Structural parsing must ignore comments: a commented-out step line
     * (`//{ ... }`) is not part of the compiled table, and a block comment may
     * contain braces. Keep the raw line for the step's own trailing comment. */
    let code = '';
    let rest = line;
    while (rest.length > 0) {
      if (inBlockComment) {
        const end = rest.indexOf('*/');
        if (end < 0) {
          rest = '';
          break;
        }
        rest = rest.slice(end + 2);
        inBlockComment = false;
      } else {
        const block = rest.indexOf('/*');
        const lineComment = rest.indexOf('//');
        if (lineComment >= 0 && (block < 0 || lineComment < block)) {
          code += rest.slice(0, lineComment);
          rest = '';
          break;
        }
        if (block < 0) {
          code += rest;
          rest = '';
          break;
        }
        code += rest.slice(0, block);
        rest = rest.slice(block + 2);
        inBlockComment = true;
      }
    }

    const defineMatch = code.match(/^\s*#define\s+(\w+)\s+(.+?)\s*$/);
    if (defineMatch) {
      const value = numericValue(defineMatch[2], defines);
      if (Number.isFinite(value)) defines.set(defineMatch[1], value);
      continue;
    }

    const ifMatch = code.match(/#if\s*\(\s*ROUTE_SCHEME_SELECT\s*==\s*(\d+)\s*\)/) ||
                    code.match(/#elif\s*\(\s*ROUTE_SCHEME_SELECT\s*==\s*(\d+)\s*\)/);
    if (ifMatch) {
      currentSchemeId = parseInt(ifMatch[1], 10);
      continue;
    }

    if (code.includes('#endif')) {
      currentSchemeId = null;
      continue;
    }

    if (code.includes('RouteStep route[]')) {
      inRouteArray = true;
      continue;
    }

    if (inRouteArray && code.includes('};')) {
      inRouteArray = false;
      continue;
    }

    if (code.includes('{') && code.includes('}')) {
      const step = parseStepLine(code, defines);
      if (step) {
        step.rawLine = line.trim();
        const comment = line.match(/\/\*\s*(.*?)\s*\*\//) || line.match(/\/\/\s*(.*)/);
        if (comment) step.comment = comment[1].trim();
      }
      if (step) {
        if (currentSchemeId !== null) {
          const s = schemes.find((sc) => sc.id === currentSchemeId);
          if (s) {
            s.steps.push(step);
          } else {
            schemes.push({
              id: currentSchemeId,
              name: `方案 ${currentSchemeId}`,
              steps: [step],
            });
          }
        } else {
          customSteps.push(step);
        }
      }
    }
  }

  // Filter schemes that have steps
  const validSchemes = schemes.filter((s) => s.steps.length > 0);
  if (validSchemes.length === 0 && customSteps.length > 0) {
    validSchemes.push({
      id: 0,
      name: '自定义上传路径',
      steps: customSteps,
    });
    activeSchemeId = 0;
  }

  /* route_scenario_start_pose(): a #if gate of scheme ids selects a table
   * anchor other than the production P(0,110)-north one. The scheme-8 ring test
   * starts on the x=345 approach heading south; other tables keep the default. */
  const startPoseMatches = sourceCode.matchAll(
    /#(?:if|elif)\s*\(([^)]*ROUTE_SCHEME_SELECT[^)]*)\)\s*\n\s*static const RouteStartPose\s+route_scenario_pose\s*=\s*\{([^}]*)\}/g);
  for (const match of startPoseMatches) {
    const ids = [...match[1].matchAll(/ROUTE_SCHEME_SELECT\s*==\s*(\d+)/g)].map((m) => Number(m[1]));
    const values = match[2].split(',').map((v) => Number(v.trim().replace(/[fF]$/, '')));
    if (ids.length && values.length >= 3 && values.slice(0, 3).every(Number.isFinite)) {
      for (const scheme of validSchemes) if (ids.includes(scheme.id))
        scheme.start_pose = { x_cm: values[0], y_cm: values[1], heading_rad: values[2] };
    }
  }

  /* Mirror table-driven scenario rules in route_table.c. */
  const closeGate = sourceCode.match(
    /route_scenario_close_corner\s*\([^)]*\)\s*\{[\s\S]*?#if\s*\(([^)]*ROUTE_SCHEME_SELECT[^)]*)\)/);
  const closeIds = closeGate
    ? [...closeGate[1].matchAll(/ROUTE_SCHEME_SELECT\s*==\s*(\d+)/g)].map((m) => Number(m[1]))
    : validSchemes.map((s) => s.id);
  const dashedGate = sourceCode.match(
    /route_scenario_dashed_row\s*\([^)]*\)\s*\{[\s\S]*?#if\s*\(([^)]*ROUTE_SCHEME_SELECT[^)]*)/);
  const dashedIds = dashedGate
    ? [...dashedGate[1].matchAll(/ROUTE_SCHEME_SELECT\s*==\s*(\d+)/g)].map((m) => Number(m[1]))
    : [];
  for (const scheme of validSchemes) {
    if (closeIds.includes(scheme.id)) {
      for (let i = 1; i < scheme.steps.length; ++i) {
        const row = scheme.steps[i], prev = scheme.steps[i - 1];
        if (row.action !== Action.CUSTOM_TURN || row.corner_arc_enabled ||
            (row.short_link_cm ?? 0) !== 0 || row.angle_deg !== -90 ||
            !(row.turn_approach_cm! > 0)) continue;
        const long = i >= 2 ? scheme.steps[i - 2].segment_length_cm : 0;
        const close95 = row.segment_length_cm === 95 && (prev.segment_length_cm === 215 || prev.segment_length_cm === 220);
        const closeQ18 = row.segment_length_cm === 45 && prev.segment_length_cm === 95 && i >= 2 &&
          (long === 215 || long === 220);
        const closeQ22 = row.segment_length_cm === 245 && prev.segment_length_cm === 45 && i >= 2 &&
          scheme.steps[i - 2].segment_length_cm === 65;
        const closeQ16 = row.segment_length_cm === 45 && prev.segment_length_cm === 65 && i >= 2 &&
          scheme.steps[i - 2].segment_length_cm === 45;
        if (close95 || closeQ18 || closeQ22 || closeQ16) row.close_corner = true;
      }
    }
    if (dashedIds.includes(scheme.id)) {
      for (let i = 1; i < scheme.steps.length; ++i) {
        const row = scheme.steps[i], prev = scheme.steps[i - 1];
        if (row.segment_length_cm === 75 && row.angle_deg === 90 &&
            prev.segment_length_cm === 35 && (prev.short_link_cm ?? 0) > 0) row.dashed_row = true;
      }
    }
  }

  const selected = validSchemes.find((s) => s.id === activeSchemeId) || validSchemes[0];

  return {
    activeSchemeId: selected ? selected.id : 0,
    schemes: validSchemes,
    activeSteps: selected ? selected.steps : customSteps,
  };
}

export const DEFAULT_ROUTE_TABLE_C = `#include "route.h"

/* 转弯前直走总距离（cm）：普通行遇全排横线从首个滤波帧锁航向，无横线则从节点确认起。 */
#define TURN_RIGHT_ANGLE_STRAIGHT_CM  9.0f
#define TURN_TRANSISTOR_STRAIGHT_CM   14.0f
/* Q4 南下右转（方案 4/9 共用）：原 14+3，减 2 cm 后为 14+1。 */
#define TURN_Q4_STRAIGHT_CM (TURN_TRANSISTOR_STRAIGHT_CM + 1.0f)

/* 字段：最多路口数、转弯速度、动作、巡航速度、右正左负角度、圆弧开关、总段距离、短段几何长度、
 *       转弯前直走总距离（turn_approach_cm，非追加量）。
 * 全排横线首帧起按入口航向直走本列距离，不跟随元件笔画；非目标过口也保持，非90°同样生效。
 * 航向保持不授权动作：正段距未达标的横线只计数，之后仍须新的确认节点。
 * 普通行：正段距必须达到后再确认灰度节点；距离为0才按计数授权，位姿不独立触发。
 * short_link_cm>0：明确标注的短连接，使用前一路口确认后的有界编码器/航向投影；长度不减5。
 * 普通直角使用 TURN_RIGHT_ANGLE_STRAIGHT_CM；三极管角点使用 TURN_TRANSISTOR_STRAIGHT_CM。
 * 显式短连接仍按短段几何定位；STOP/STRAIGHT/HEXAGON 不使用此直走距离。
 * 电阻及其他印刷图形参与节点计数，使用连续证据及离口去重。
 * 未标定行距离 = 图纸几何长度 - 5 cm；实车标定例外见逐行注释，原始长度见 test/new_track_routes.json。
 * 传感器前伸 24 cm，短段节点可能在上一动作内通过，计数需逐段回放校准。
 * 方案 0：保留四步调试路线；1：走虚线全元件；2：不走虚线；3：外框测试圈。
 * 正式方案仍在仿真调试中，不能据表名宣称已经完赛或实车验证。
 * 地图：375x220，Q6(245,140)，右中线 y110，环岛中心(310,165)，P(0,110)。
 * 本文件由 test/sync_new_track_routes.py 生成。
 */
#ifndef ROUTE_SCHEME_SELECT
#define ROUTE_SCHEME_SELECT 4
#endif

/* 起跑位姿锚点（世界坐标：x 向东、y 向南）。
 * 方案 8 是单独的环岛测试，应从环岛北侧 x=345 的主线（y=110 路口）朝南出发：
 * HEXAGON 的环岛几何固定为右顶点 (345,165)、中心 (310,165)。如果仍用生产起点
 * P(0,110) 朝北，车与右顶点相距约 380 cm，而入环阶段只允许直行
 * HEXAGON_ENTRY_APPROACH_MAX_CM(35 cm)，必然 ENTRY_SPEED 后丢线停车。
 * 2026-11-11 只读现场（方案 8）：第 0 行 HEXAGON 在 d≈24 已确认环岛入口，
 * 但 d=84.7 的底部 T 口被当成入环口再次动作，d≈119.7 ENTRY_SPEED，恢复 12 cm
 * 后 EXIT_LOST 停车；hexagon.state 始终没到 FOLLOWING —— 即“不进入环岛”。
 * 其它方案（生产 1–7、调试 0）仍锚在 P。 */
/* 方案 9：从方案 4 的实际 index 12 开始，轮轴中心置于右下角朝北。 */
#if (ROUTE_SCHEME_SELECT == 9)
static const RouteStartPose route_scenario_pose={375.0f,220.0f,-1.570796327f};
#elif (ROUTE_SCHEME_SELECT == 8 || ROUTE_SCHEME_SELECT == 1)
static const RouteStartPose route_scenario_pose={345.0f,110.0f,1.570796327f}; /* +pi/2 = 朝南 */
#else
static const RouteStartPose route_scenario_pose={ROUTE_START_X_CM,ROUTE_START_Y_CM,
    ROUTE_START_HEADING_RAD};
#endif
const RouteStartPose *route_scenario_start_pose(void) { return &route_scenario_pose; }

const RouteStep route[] = {
#if (ROUTE_SCHEME_SELECT == 0)
    //{ 1, 50.0f, CUSTOM_TURN,  50,   -90.0f,false ,  100.0f, 0.0f}, /* 调试 0；几何 60 - 5 = 55 cm */
    /* 调试 1；几何 239 - 5 = 234 cm。2026-11-09 21:2x 实车（方案 0）：转弯比表早 4 cm，
     * 右转后小车在线内侧、与出弯线平行错约 4 cm（灰度过线但一直判丢线）。
     * 段距 230→234 让本行角点/位姿起转点整体后移 4 cm（滚动转弯的切点 E=K-R·u 也随之
     * 后移），即“转弯前再走 4 cm”；出弯切点 X=K+R·o 才落回出弯线上。 */
    { 2, 20.0f, CUSTOM_TURN,  50,    90.0f,false,   50.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM},
    { 3, 20.0f, STOP       ,  50,    90.0f,false,   40.0f, 0.0f, 0.0f},
    //{ 5, 50.0f, CUSTOM_TURN, 80,   90.0f, true ,  234.0f, 0.0f},
    //{ 3, 50.0f, STOP, 80,   90.0f,false ,   70.0f, 0.0f}, /* 调试 2；几何 60 - 5 = 55 cm */
    //{ 7, 50.0f, RIGHT       , 80,    0.0f,false,  220.0f, 0.0f}, /* 调试 3；几何 200 - 5 = 195 cm */
    //{ 1, 50.0f, STOP        , 50,    0.0f, false ,  100.0f, 0.0f}, /* 调试 0；几何 60 - 5 = 55 cm */

#elif (ROUTE_SCHEME_SELECT == 1)
    /* 正式赛道：走虚线、全元件（调试中） */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,  105.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 0: P 北上至 (0,0)；几何 110 - 5 = 105 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   30.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 1: 顶边东至 (55,0)；R2 在上一转弯传感器扫过区内，不再计数；几何 55 - 5 = 50 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  135.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 2: x55 南下至 (55,140)，过 R3；几何 140 - 5 = 135 cm */
    { 3, 28.0f, CUSTOM_TURN,  80,    90.0f,false,  185.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM + 5.0f}, /* 3: y140 东至 Q6(245,140)，右转南下；Q4 位于上一转弯传感器扫过区，计 R5/目标；几何 190 - 5 = 185 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   25.0f, 30.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 4: x245 南下至 (245,170)；几何 30 - 5 = 25 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  130.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 5: y170 西至 (95,170)，过 Q11；几何 150 - 5 = 145 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   45.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 6: x95 南下至 (95,220)；几何 50 - 5 = 45 cm */
    /* 2026-11-10 实车：角点在段进度239.61确认，275门禁247.15已晚7.54 cm。
     * 本行按动作完成点的实测里程校准为265，门禁237.15；前节点205.56仍只通过。
     * 几何原长280不变；此列现为实车门禁标定值，生成器会覆盖手工标定。 */
    { 5, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  265.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 7: 底边东至 (375,220)，过 R8/R9/R10；段距门禁实车校准265 cm */
    // 右下角
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  105.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 8: x375 北上至 (375,110)，过 R12；几何 110 - 5 = 105 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   25.0f, 30.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 9: y110 西至 Q13(345,110)；几何 30 - 5 = 25 cm */
    { 1, 28.0f, HEXAGON    ,  80,   360.0f,false,   50.0f, 0.0f, 0.0f}, /* 10: HEXAGON：x345 南下入环（北进），绕环一周后从南侧 x345 出环到 (345,220)；几何 55 - 5 = 50 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   50.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 11: x345 南下至 (345,220)；几何 55 - 5 = 50 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   25.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 12: 底边东至 (375,220)，左转北上；几何 30 - 5 = 25 cm */
    { 6, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  215.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 13: x375 北上至 (375,0)，过 R12/R15/y110/y50；几何 220 - 5 = 215 cm */
    { 3, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   95.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 14: 顶边西至 (275,0)，过 Q16/R17；几何 100 - 5 = 95 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   45.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 15: x275 南下至 Q18(275,50)；几何 50 - 5 = 45 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   65.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 16: y50 东至 (345,50)，过 R19；几何 70 - 5 = 65 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   45.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 17: x345 北上至 Q16(345,0)；几何 50 - 5 = 45 cm */
    { 6, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  245.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 18: 顶边西至 Q22(95,0)，过 R17/x275/x245/R20/R21；几何 250 - 5 = 245 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 19: x95 南下至 (95,40)；几何 40 - 5 = 35 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   65.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 20: y40 东至 (165,40)，过 R23；几何 70 - 5 = 65 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 21: x165 南下至 (165,80)；几何 40 - 5 = 35 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   75.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 22: y80 东至 (245,80)，过 R24；几何 80 - 5 = 75 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 23: x245 北上至 (245,40)；几何 40 - 5 = 35 cm */
    { 4, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  145.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 24: y40 西至 (95,40)，过 R25/x165/R23；几何 150 - 5 = 145 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   95.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 25: x95 南下至 Q4(95,140)；几何 100 - 5 = 95 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 26: y140 西至 (55,140)，左转进入虚线；几何 40 - 5 = 35 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   75.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 27: x55 虚线仅南下至 (55,220)，过 R26；几何 80 - 5 = 75 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   50.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 28: 底边西至 (0,220)；几何 55 - 5 = 50 cm */
    { 2, 28.0f, STOP       ,  80,     0.0f,false,  105.0f, 0.0f, 0.0f}, /* 29: STOP：x0 北上过 R27 至 P；几何 110 - 5 = 105 cm */

#elif (ROUTE_SCHEME_SELECT == 2)
    /* 正式赛道：不走虚线（调试中） */
    { 2, 28.0f, CUSTOM_TURN,  80,    90.0f,false,  105.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 0: P 北上至 (0,0)；几何 110 - 5 = 105 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   50.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 1: 顶边东至 (55,0)；R2 已在上一动作扫过，不再计数；几何 55 - 5 = 50 cm */
    { 2, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,  135.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 2: x55 南下至 (55,140)；几何 140 - 5 = 135 cm */
    { 2, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,  185.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 3: y140 东至 Q6(245,140)，左转北上；Q4 已被上一动作扫过，计 R5/目标；几何 190 - 5 = 185 cm */
    { 2, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,   95.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 4: x245 北上过 y80 至 (245,40)；几何 100 - 5 = 95 cm */
    { 2, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,   75.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 5: y40 西过 R25 至 (165,40)；几何 80 - 5 = 75 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 6: x165 南下至 (165,80)；几何 40 - 5 = 35 cm */
    { 2, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,   75.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 7: y80 东过 R24 至 (245,80)；几何 80 - 5 = 75 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 8: x245 北上至 (245,40)；几何 40 - 5 = 35 cm */
    { 3, 28.0f, CUSTOM_TURN, 160,    90.0f,false,  145.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 9: y40 西过 x165/R23 至 (95,40)，右转北上；R25在上一动作扫过，60/16回放计3口；几何 150 - 5 = 145 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   35.0f, 40.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 10: x95 北上至 Q22(95,0)；几何 40 - 5 = 35 cm */
    { 6, 28.0f, CUSTOM_TURN, 160,    90.0f,false,  245.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 11: 顶边东至 Q16(345,0)，过 R21/R20/x245/x275/R17；几何 250 - 5 = 245 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   45.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 12: x345 南下至 (345,50)；几何 50 - 5 = 45 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   65.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 13: y50 西至 Q18(275,50)；R19 在上一转弯扫过区内；几何 70 - 5 = 65 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   45.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 14: x275 北上至 (275,0)；几何 50 - 5 = 45 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   95.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 15: 顶边东过 Q16 至 (375,0)；R17 已扫过，Q16 仅短候选未确认，实跑计目标1；几何 100 - 5 = 95 cm */
    { 5, 28.0f, CUSTOM_TURN, 160,    90.0f,false,  215.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 16: x375 南下至 (375,220)，过 y50/R15/y110/R12；几何 220 - 5 = 215 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   25.0f, 30.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 17: 底边西至 (345,220)；几何 30 - 5 = 25 cm */
    { 1, 28.0f, HEXAGON    , 160,  -360.0f,false,   50.0f, 0.0f, 0.0f}, /* 18: HEXAGON：x345 北上入环（南进），逆时针绕环一周后从北侧 x345 出环到 (345,110)；几何 55 - 5 = 50 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   50.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 19: x345 北上至 Q13(345,110)；几何 55 - 5 = 50 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   25.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 20: y110 东至 (375,110)；几何 30 - 5 = 25 cm */
    { 2, 28.0f, CUSTOM_TURN, 160,    90.0f,false,  105.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 21: x375 南下过 R12 至 (375,220)；几何 110 - 5 = 105 cm */
    { 5, 28.0f, CUSTOM_TURN, 160,    90.0f,false,  275.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 22: 底边西至 (95,220)，过 x345/R10/R9/x150/R8；几何 280 - 5 = 275 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   45.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 23: x95 北上至 (95,170)；几何 50 - 5 = 45 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   50.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 24: y170 东至 Q11(150,170)；几何 55 - 5 = 50 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   45.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 25: x150 南下至 (150,220)；几何 50 - 5 = 45 cm */
    { 4, 28.0f, CUSTOM_TURN, 160,    90.0f,false,  145.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 26: 底边西过 R8/x95/x55 至 (0,220)；几何 150 - 5 = 145 cm */
    { 2, 28.0f, STOP       , 160,     0.0f,false,  105.0f, 0.0f, 0.0f}, /* 27: STOP：x0 北上过 R27 至 P；几何 110 - 5 = 105 cm */

#elif (ROUTE_SCHEME_SELECT == 3)
    /* 外框测试圈（普通） */
    { 2, 18.0f, CUSTOM_TURN, 160,    90.0f,false,  100.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 0: 外框 0：P 北上，电阻计1、左上角计2；几何 110 - 5 = 105 cm */
    { 11, 18.0f, CUSTOM_TURN, 160,    90.0f,false,  370.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 1: 外框 1：顶边东至 (375,0)；几何 375 - 5 = 370 cm */
    { 5, 18.0f, CUSTOM_TURN, 160,    90.0f,false,  215.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 2: 外框 2：x375 南下至 (375,220)；几何 220 - 5 = 215 cm */
    { 6, 18.0f, CUSTOM_TURN, 160,    90.0f,false,  370.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 3: 外框 3：底边西至 (0,220)；几何 375 - 5 = 370 cm */
    { 2, 18.0f, STOP       , 160,     0.0f,false,  105.0f, 0.0f, 0.0f}, /* 4: 外框 4：x0 北上至 P；几何 110 - 5 = 105 cm */

#elif (ROUTE_SCHEME_SELECT == 4)
    /* 走虚线、不进六边形环岛（普通） */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,  105.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 0: P 北上至 (0,0)；几何 110 - 5 = 105 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   30.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 1: 顶边东至 (55,0)；R2 在上一转弯传感器扫过区内，不再计数；几何 55 - 5 = 50 cm */
    { 2, 28.0f, CUSTOM_TURN,  120,   -90.0f,false,  135.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 2: x55 南下至 (55,140)，过 R3；几何 140 - 5 = 135 cm */
    { 3, 28.0f, CUSTOM_TURN,  160,    90.0f,false,  185.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM + 5.0f}, /* 3: y140 东至 Q6(245,140)，右转南下；Q4 位于上一转弯传感器扫过区，计 R5/目标；几何 190 - 5 = 185 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   25.0f, 30.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 4: x245 南下至 (245,170)；几何 30 - 5 = 25 cm */
    { 2, 28.0f, CUSTOM_TURN,  120,   -90.0f,false,  130.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 5: y170 西至 (95,170)，过 Q11；几何 150 - 5 = 145 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   45.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 6: x95 南下至 (95,220)；几何 50 - 5 = 45 cm */
    /* 2026-11-10 实车：角点在段进度239.61确认，275门禁247.15已晚7.54 cm。
     * 本行按动作完成点的实测里程校准为265，门禁237.15；前节点205.56仍只通过。
     * 几何原长280不变；此列现为实车门禁标定值，生成器会覆盖手工标定。 */
    { 5, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  265.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 7: 底边东至 (375,220)，过 R8/R9/R10；段距门禁实车校准265 cm */
    // 右下角
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  105.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 8: x375 北上至 (375,110)，过 R12；几何 110 - 5 = 105 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   25.0f, 30.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 9: y110 西至 Q13(345,110)；几何 30 - 5 = 25 cm */
    //{ 1, 28.0f, HEXAGON    ,  80,   360.0f,false,   50.0f, 0.0f, 0.0f}, /* 10: HEXAGON：x345 南下入环（北进），绕环一周后从南侧 x345 出环到 (345,220)；几何 55 - 5 = 50 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   90, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM + 5.0f}, /* 11: x345 南下至 (345,220)；几何 55 - 5 = 50 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   25.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 12: 底边东至 (375,220)，左转北上；几何 30 - 5 = 25 cm */
    { 5, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  215.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 13: x375 北上至 (375,0)，过 R12/R15/y110/y50；几何 220 - 5 = 215 cm */

    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   95.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 14: 顶边西至 (275,0)，过 Q16/R17；几何 100 - 5 = 95 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   45.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 15: x275 南下至 Q18(275,50)；几何 50 - 5 = 45 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   65.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 16: y50 东至 (345,50)，过 R19；几何 70 - 5 = 65 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   45.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 17: x345 北上至 Q16(345,0)；几何 50 - 5 = 45 cm */
    { 6, 28.0f, CUSTOM_TURN,  160,   -90.0f,false,  245.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 18: 顶边西至 Q22(95,0)，过 R17/x275/x245/R20/R21；几何 250 - 5 = 245 cm */

    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 19: x95 南下至 (95,40)；几何 40 - 5 = 35 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   65.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 20: y40 东至 (165,40)，过 R23；几何 70 - 5 = 65 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 21: x165 南下至 (165,80)；几何 40 - 5 = 35 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   75.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 22: y80 东至 (245,80)，过 R24；几何 80 - 5 = 75 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 23: x245 北上至 (245,40)；几何 40 - 5 = 35 cm */
    { 4, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  145.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 24: y40 西至 (95,40)，过 R25/x165/R23；几何 150 - 5 = 145 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   95.0f, 0.0f, TURN_Q4_STRAIGHT_CM}, /* 25: x95 南下至 Q4(95,140)；与方案 9 共用起转距离 */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 26: y140 西至 (55,140)，左转进入虚线；几何 40 - 5 = 35 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   75.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 27: x55 虚线仅南下至 (55,220)，过 R26；几何 80 - 5 = 75 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   50.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 28: 底边西至 (0,220)；几何 55 - 5 = 50 cm */
    { 2, 28.0f, STOP       ,  80,     0.0f,false,  105.0f, 0.0f, 0.0f}, /* 29: STOP：x0 北上过 R27 至 P；几何 110 - 5 = 105 cm */

#elif (ROUTE_SCHEME_SELECT == 5)
    /* 快速版：走虚线、含环岛（尽速） */
    { 2, 64.0f, CUSTOM_TURN, 220,    90.0f,false,  105.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 0: P 北上至 (0,0)；几何 110 - 5 = 105 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,    90.0f,false,   50.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 1: 顶边东至 (55,0)；R2 在上一转弯传感器扫过区内，不再计数；几何 55 - 5 = 50 cm */
    { 2, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,  135.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 2: x55 南下至 (55,140)，过 R3；几何 140 - 5 = 135 cm */
    { 2, 64.0f, CUSTOM_TURN, 220,    90.0f,false,  185.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 3: y140 东至 Q6(245,140)，右转南下；Q4 位于上一转弯传感器扫过区，计 R5/目标；几何 190 - 5 = 185 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,    90.0f,false,   25.0f, 30.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 4: x245 南下至 (245,170)；几何 30 - 5 = 25 cm */
    { 2, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,  145.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 5: y170 西至 (95,170)，过 Q11；几何 150 - 5 = 145 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,   45.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 6: x95 南下至 (95,220)；几何 50 - 5 = 45 cm */
    { 5, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,  275.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 7: 底边东至 (375,220)，过 R8/R9/R10；几何 280 - 5 = 275 cm */
    { 2, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,  105.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 8: x375 北上至 (375,110)，过 R12；几何 110 - 5 = 105 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,   25.0f, 30.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 9: y110 西至 Q13(345,110)；几何 30 - 5 = 25 cm */
    { 1, 64.0f, HEXAGON    , 220,   360.0f,false,   50.0f, 0.0f, 0.0f}, /* 10: HEXAGON：x345 南下入环（北进），绕环一周后从南侧 x345 出环到 (345,220)；几何 55 - 5 = 50 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,   50.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 11: x345 南下至 (345,220)；几何 55 - 5 = 50 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,   25.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 12: 底边东至 (375,220)，左转北上；几何 30 - 5 = 25 cm */
    { 6, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,  215.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 13: x375 北上至 (375,0)，过 R12/R15/y110/y50；几何 220 - 5 = 215 cm */
    { 3, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,   95.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 14: 顶边西至 (275,0)，过 Q16/R17；几何 100 - 5 = 95 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,   45.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 15: x275 南下至 Q18(275,50)；几何 50 - 5 = 45 cm */
    { 2, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,   65.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 16: y50 东至 (345,50)，过 R19；几何 70 - 5 = 65 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,   45.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 17: x345 北上至 Q16(345,0)；几何 50 - 5 = 45 cm */
    { 6, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,  245.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 18: 顶边西至 Q22(95,0)，过 R17/x275/x245/R20/R21；几何 250 - 5 = 245 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 19: x95 南下至 (95,40)；几何 40 - 5 = 35 cm */
    { 2, 64.0f, CUSTOM_TURN, 220,    90.0f,false,   65.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 20: y40 东至 (165,40)，过 R23；几何 70 - 5 = 65 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 21: x165 南下至 (165,80)；几何 40 - 5 = 35 cm */
    { 2, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,   75.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 22: y80 东至 (245,80)，过 R24；几何 80 - 5 = 75 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 23: x245 北上至 (245,40)；几何 40 - 5 = 35 cm */
    { 4, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,  145.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 24: y40 西至 (95,40)，过 R25/x165/R23；几何 150 - 5 = 145 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,    90.0f,false,   95.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 25: x95 南下至 Q4(95,140)；几何 100 - 5 = 95 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 26: y140 西至 (55,140)，左转进入虚线；几何 40 - 5 = 35 cm */
    { 2, 64.0f, CUSTOM_TURN, 220,    90.0f,false,   75.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 27: x55 虚线仅南下至 (55,220)，过 R26；几何 80 - 5 = 75 cm */
    { 1, 64.0f, CUSTOM_TURN, 220,    90.0f,false,   50.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 28: 底边西至 (0,220)；几何 55 - 5 = 50 cm */
    { 2, 64.0f, STOP       , 220,     0.0f,false,  105.0f, 0.0f, 0.0f}, /* 29: STOP：x0 北上过 R27 至 P；几何 110 - 5 = 105 cm */

#elif (ROUTE_SCHEME_SELECT == 6)
    /* 快速版：不走虚线（尽速） */
    { 2, 28.0f, CUSTOM_TURN, 160,    90.0f,false,  105.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 0: P 北上至 (0,0)；几何 110 - 5 = 105 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   50.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 1: 顶边东至 (55,0)；R2 已在上一动作扫过，不再计数；几何 55 - 5 = 50 cm */
    { 2, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,  135.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 2: x55 南下至 (55,140)；几何 140 - 5 = 135 cm */
    { 2, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,  185.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 3: y140 东至 Q6(245,140)，左转北上；Q4 已被上一动作扫过，计 R5/目标；几何 190 - 5 = 185 cm */
    { 2, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,   95.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 4: x245 北上过 y80 至 (245,40)；几何 100 - 5 = 95 cm */
    { 2, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,   75.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 5: y40 西过 R25 至 (165,40)；几何 80 - 5 = 75 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 6: x165 南下至 (165,80)；几何 40 - 5 = 35 cm */
    { 2, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,   75.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 7: y80 东过 R24 至 (245,80)；几何 80 - 5 = 75 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 8: x245 北上至 (245,40)；几何 40 - 5 = 35 cm */
    { 3, 28.0f, CUSTOM_TURN, 160,    90.0f,false,  145.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 9: y40 西过 x165/R23 至 (95,40)，右转北上；R25在上一动作扫过，60/16回放计3口；几何 150 - 5 = 145 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   35.0f, 40.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 10: x95 北上至 Q22(95,0)；几何 40 - 5 = 35 cm */
    { 6, 28.0f, CUSTOM_TURN, 160,    90.0f,false,  245.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 11: 顶边东至 Q16(345,0)，过 R21/R20/x245/x275/R17；几何 250 - 5 = 245 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   45.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 12: x345 南下至 (345,50)；几何 50 - 5 = 45 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   65.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 13: y50 西至 Q18(275,50)；R19 在上一转弯扫过区内；几何 70 - 5 = 65 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   45.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 14: x275 北上至 (275,0)；几何 50 - 5 = 45 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   95.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 15: 顶边东过 Q16 至 (375,0)；R17 已扫过，Q16 仅短候选未确认，实跑计目标1；几何 100 - 5 = 95 cm */
    { 5, 28.0f, CUSTOM_TURN, 160,    90.0f,false,  215.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 16: x375 南下至 (375,220)，过 y50/R15/y110/R12；几何 220 - 5 = 215 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   25.0f, 30.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 17: 底边西至 (345,220)；几何 30 - 5 = 25 cm */
    { 1, 28.0f, HEXAGON    , 160,  -360.0f,false,   50.0f, 0.0f, 0.0f}, /* 18: HEXAGON：x345 北上入环（南进），逆时针绕环一周后从北侧 x345 出环到 (345,110)；几何 55 - 5 = 50 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   50.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 19: x345 北上至 Q13(345,110)；几何 55 - 5 = 50 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   25.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 20: y110 东至 (375,110)；几何 30 - 5 = 25 cm */
    { 2, 28.0f, CUSTOM_TURN, 160,    90.0f,false,  105.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 21: x375 南下过 R12 至 (375,220)；几何 110 - 5 = 105 cm */
    { 5, 28.0f, CUSTOM_TURN, 160,    90.0f,false,  275.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 22: 底边西至 (95,220)，过 x345/R10/R9/x150/R8；几何 280 - 5 = 275 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   45.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 23: x95 北上至 (95,170)；几何 50 - 5 = 45 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   50.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 24: y170 东至 Q11(150,170)；几何 55 - 5 = 50 cm */
    { 1, 28.0f, CUSTOM_TURN, 160,    90.0f,false,   45.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 25: x150 南下至 (150,220)；几何 50 - 5 = 45 cm */
    { 4, 28.0f, CUSTOM_TURN, 160,    90.0f,false,  145.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 26: 底边西过 R8/x95/x55 至 (0,220)；几何 150 - 5 = 145 cm */
    { 2, 28.0f, STOP       , 160,     0.0f,false,  105.0f, 0.0f, 0.0f}, /* 27: STOP：x0 北上过 R27 至 P；几何 110 - 5 = 105 cm */

#elif (ROUTE_SCHEME_SELECT == 7)
    /* 快速版：外框测试圈（尽速） */
    { 2, 64.0f, CUSTOM_TURN, 260,    90.0f,false,  105.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 0: 外框 0：P 北上至 (0,0)；几何 110 - 5 = 105 cm */
    { 11, 64.0f, CUSTOM_TURN, 260,    90.0f,false,  370.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 1: 外框 1：顶边东至 (375,0)；几何 375 - 5 = 370 cm */
    { 5, 64.0f, CUSTOM_TURN, 260,    90.0f,false,  215.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 2: 外框 2：x375 南下至 (375,220)；几何 220 - 5 = 215 cm */
    { 6, 64.0f, CUSTOM_TURN, 260,    90.0f,false,  370.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 3: 外框 3：底边西至 (0,220)；几何 375 - 5 = 370 cm */
    { 2, 64.0f, STOP       , 260,     0.0f,false,  105.0f, 0.0f, 0.0f}, /* 4: 外框 4：x0 北上至 P；几何 110 - 5 = 105 cm */

#elif (ROUTE_SCHEME_SELECT == 8)
    /* 临时方案 */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,  105.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 0: P 北上至 (0,0)；几何 110 - 5 = 105 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   30.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 1: 顶边东至 (55,0)；R2 在上一转弯传感器扫过区内，不再计数；几何 55 - 5 = 50 cm */
    { 2, 28.0f, CUSTOM_TURN,  120,   -90.0f,false,  135.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 2: x55 南下至 (55,140)，过 R3；几何 140 - 5 = 135 cm */
    { 3, 28.0f, CUSTOM_TURN,  160,    90.0f,false,  185.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM + 5.0f}, /* 3: y140 东至 Q6(245,140)，右转南下；Q4 位于上一转弯传感器扫过区，计 R5/目标；几何 190 - 5 = 185 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   25.0f, 30.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 4: x245 南下至 (245,170)；几何 30 - 5 = 25 cm */
    { 2, 28.0f, CUSTOM_TURN,  120,   -90.0f,false,  130.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 5: y170 西至 (95,170)，过 Q11；几何 150 - 5 = 145 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   45.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 6: x95 南下至 (95,220)；几何 50 - 5 = 45 cm */
    /* 2026-11-10 实车：角点在段进度239.61确认，275门禁247.15已晚7.54 cm。
     * 本行按动作完成点的实测里程校准为265，门禁237.15；前节点205.56仍只通过。
     * 几何原长280不变；此列现为实车门禁标定值，生成器会覆盖手工标定。 */
    { 5, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  265.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 7: 底边东至 (375,220)，过 R8/R9/R10；段距门禁实车校准265 cm */
    // 右下角
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  105.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 8: x375 北上至 (375,110)，过 R12；几何 110 - 5 = 105 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   25.0f, 30.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 9: y110 西至 Q13(345,110)；几何 30 - 5 = 25 cm */
    //{ 1, 28.0f, HEXAGON    ,  80,   360.0f,false,   50.0f, 0.0f, 0.0f}, /* 10: HEXAGON：x345 南下入环（北进），绕环一周后从南侧 x345 出环到 (345,220)；几何 55 - 5 = 50 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   90, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM + 5.0f}, /* 11: x345 南下至 (345,220)；几何 55 - 5 = 50 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   25.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 12: 底边东至 (375,220)，左转北上；几何 30 - 5 = 25 cm */
    { 5, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  215.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 13: x375 北上至 (375,0)，过 R12/R15/y110/y50；几何 220 - 5 = 215 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   95.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 14: 顶边西至 (275,0)，过 Q16/R17；几何 100 - 5 = 95 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   45.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM + 5.0f}, /* 15: x275 南下至 Q18(275,50)；几何 50 - 5 = 45 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   65.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 16: y50 东至 (345,50)，过 R19；几何 70 - 5 = 65 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   45.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 17: x345 北上至 Q16(345,0)；几何 50 - 5 = 45 cm */
    { 9, 28.0f, CUSTOM_TURN,  160,   -90.0f,false,  245.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 18: 顶边西至 Q22(95,0)，过 R17/x275/x245/R20/R21；几何 250 - 5 = 245 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   95.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 25: x95 南下至 Q4(95,140)；几何 100 - 5 = 95 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 26: y140 西至 (55,140)，左转进入虚线；几何 40 - 5 = 35 cm */
    { 2, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   75.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 27: x55 虚线仅南下至 (55,220)，过 R26；几何 80 - 5 = 75 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   50.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 28: 底边西至 (0,220)；几何 55 - 5 = 50 cm */
    { 2, 28.0f, STOP       ,  80,     0.0f,false,  105.0f, 0.0f, 0.0f},

#elif (ROUTE_SCHEME_SELECT == 9)
    /* 方案 4 实际 index 12 起的完整后缀；手工表字段原样保留，共 17 项。
     * 起跑轮轴中心 (375,220) 朝北；末 STOP 在 P，此后缀不闭环。 */
    { 5, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  215.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 0: x375 北上至 (375,0)，原注释 13 */
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   95.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 1: 顶边西至 (275,0)，过 Q16/R17 */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   45.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 2: x275 南下至 Q18(275,50) */
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   65.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 3: y50 东至 (345,50)，过 R19 */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   45.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 4: x345 北上至 Q16(345,0) */
    { 6, 28.0f, CUSTOM_TURN, 160,   -90.0f,false,  245.0f, 0.0f, TURN_TRANSISTOR_STRAIGHT_CM}, /* 5: 顶边西至 Q22(95,0) */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 6: x95 南下至 (95,40) */
    { 2, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   65.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 7: y40 东至 (165,40)，过 R23 */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 8: x165 南下至 (165,80) */
    { 2, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   75.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 9: y80 东至 (245,80)，过 R24 */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 10: x245 北上至 (245,40) */
    { 4, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,  145.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 11: y40 西至 (95,40) */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   95.0f, 0.0f, TURN_Q4_STRAIGHT_CM}, /* 12: x95 南下至 Q4(95,140)；17→15 cm，起转提前 2 cm */
    { 1, 28.0f, CUSTOM_TURN,  80,   -90.0f,false,   35.0f, 40.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 13: y140 西至 (55,140)，左转进入虚线 */
    { 2, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   75.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 14: x55 虚线南下至 (55,220)，过 R26 */
    { 1, 28.0f, CUSTOM_TURN,  80,    90.0f,false,   50.0f, 0.0f, TURN_RIGHT_ANGLE_STRAIGHT_CM}, /* 15: 底边西至 (0,220) */
    { 2, 28.0f, STOP       ,  80,     0.0f,false,  105.0f, 0.0f, 0.0f}, /* 16: x0 北上过 R27 至 P(0,110) */
#endif
};

const size_t route_count = sizeof(route) / sizeof(route[0]);

/* 几何枢轴特例：R17 后的 95 cm、紧接的 Q18 45 cm、随后的 Q16 45 cm 与 Q22 245 cm。
 * 按相邻段内容识别，不绑定索引或正直走距离的具体值。保留前一弯的出口轴位移，
 * 段距门禁与角点前半轮距的枢轴使用同一原点，避免目标图形在门禁前被当作过口。
 * 三极管 L 形角点（Q16/Q22 所在顶边）只在车轴快到角点时才能给出中心+分支证据，
 * 若按“确认点 + 配置直走距离（14 cm）”起转，枢轴中心就落过角点，出弯与目标线
 * 平行偏开，探头再也压不到线（2026-11-12 实车：Q16 未标记 → Q22 段原点整体后移，
 * 出弯与 x=95 平行偏开，12 cm 出口搜索耗尽后 EXIT_RECAPTURE）。因此同一链上的
 * 角点必须共用“角点前半个轮距”的几何规则，与确认时刻无关。
 * 方案 1/4/5/8/9 的这四行内容相同，规则一并生效。 */
/* 虚线行：进入虚线的那一行是 35 cm + 40 cm 短连接，虚线行紧随其后（75 cm）。
 * 只按表内容识别，不绑定索引。 */
bool route_scenario_dashed_row(const RouteStep *step)
{
#if (ROUTE_SCHEME_SELECT == 1 || ROUTE_SCHEME_SELECT == 4 || \
     ROUTE_SCHEME_SELECT == 5 || ROUTE_SCHEME_SELECT == 8 || ROUTE_SCHEME_SELECT == 9)
    for(size_t i=1;i<route_count;++i) {
        const RouteStep *r=&route[i];
        if(step!=r) continue;
        if(r->segment_length_cm!=75.0f || r->angle_deg!=90.0f) return false;
        const RouteStep *prev=&route[i-1];
        return prev->segment_length_cm==35.0f && prev->short_link_cm>0.0f;
    }
#endif
    (void)step;return false;
}

bool route_scenario_close_corner(const RouteStep *step)
{
#if (ROUTE_SCHEME_SELECT == 1 || ROUTE_SCHEME_SELECT == 4 || \
     ROUTE_SCHEME_SELECT == 5 || ROUTE_SCHEME_SELECT == 8 || ROUTE_SCHEME_SELECT == 9)
    for(size_t i=1;i<route_count;++i) {
        const RouteStep *r=&route[i];
        if(step!=r || r->action!=CUSTOM_TURN || r->corner_arc_enabled ||
           r->short_link_cm!=0.0f || r->angle_deg!=-90.0f ||
           !(r->turn_approach_cm>0.0f)) continue;
        float prev=route[i-1].segment_length_cm;
        if(r->segment_length_cm==95.0f && (prev==215.0f || prev==220.0f))
            return true;
        if(i>=2 && r->segment_length_cm==45.0f && prev==95.0f) {
            const RouteStep *prior=&route[i-1];
            float long_leg=route[i-2].segment_length_cm;
            if(prior->action==CUSTOM_TURN && prior->angle_deg==-90.0f &&
               !prior->corner_arc_enabled && prior->short_link_cm==0.0f &&
               prior->turn_approach_cm>0.0f &&
               (long_leg==215.0f || long_leg==220.0f)) return true;
        }
        if(i>=2 && r->segment_length_cm==245.0f && prev==45.0f) {
            const RouteStep *prior=&route[i-1];
            if(prior->action==CUSTOM_TURN && prior->angle_deg==-90.0f &&
               !prior->corner_arc_enabled && prior->short_link_cm==0.0f &&
               prior->turn_approach_cm>0.0f && route[i-2].segment_length_cm==65.0f)
                return true;
        }
        /* Q16：紧接 Q18(45) 与 65 cm 横段的 45 cm 三极管角点。它若在自己的角点上
         * 起转，出口就晚半个轮距，245 cm 段的原点整体后移，Q22 枢轴便落到自己
         * 的角点上而不是角点前半个轮距，出弯与 x=95 平行偏半个轮距。 */
        if(i>=2 && r->segment_length_cm==45.0f && prev==65.0f &&
           route[i-2].segment_length_cm==45.0f) return true;
    }
#endif
    (void)step;return false;
}


`;
