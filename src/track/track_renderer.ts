import {
  TRACK_WIDTH_CM,
  TRACK_HEIGHT_CM,
  TRACK_LINE_WIDTH_CM,
  TRACK_SEGMENTS,
  CIRCUIT_COMPONENTS,
  RESISTOR_LENGTH_CM,
  RESISTOR_WIDTH_CM,
  RESISTOR_STROKE_CM,
  TRANSISTOR_MOUTH_CM,
  TRANSISTOR_DEPTH_CM,
  TRANSISTOR_BAR_HALF_CM,
  TRANSISTOR_STROKE_CM,
  BATTERY_PLATE_SPACING_CM,
  BATTERY_LONG_HALF_CM,
  BATTERY_SHORT_HALF_CM,
  BATTERY_STROKE_CM,
  type TrackComponent,
} from './map_data';

export interface ViewportTransform {
  scale: number;
  offsetX: number;
  offsetY: number;
  width: number;
  height: number;
}

export interface RenderOptions {
  showDimensions: boolean;
  showComponentLabels: boolean;
  showGrid: boolean;
  highContrast: boolean;
}

export class TrackRenderer {
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;

  public options: RenderOptions = {
    showDimensions: true,
    showComponentLabels: true,
    showGrid: true,
    highContrast: false,
  };

  public transform: ViewportTransform = {
    scale: 1,
    offsetX: 0,
    offsetY: 0,
    width: 800,
    height: 600,
  };

  constructor(canvas: HTMLCanvasElement) {
    this.canvas = canvas;
    const context = canvas.getContext('2d');
    if (!context) throw new Error('Cannot get 2D canvas context');
    this.ctx = context;
  }

  /**
   * Automatically compute scale and offset to fill the viewport
   */
  public fitViewport(paddingPx = 40): void {
    const w = this.canvas.width;
    const h = this.canvas.height;
    this.transform.width = w;
    this.transform.height = h;

    const availW = w - paddingPx * 2;
    const availH = h - paddingPx * 2;

    const scaleX = availW / TRACK_WIDTH_CM;
    const scaleY = availH / TRACK_HEIGHT_CM;
    const scale = Math.min(scaleX, scaleY);

    this.transform.scale = scale;
    this.transform.offsetX = (w - TRACK_WIDTH_CM * scale) / 2;
    this.transform.offsetY = (h - TRACK_HEIGHT_CM * scale) / 2;
  }

  public toScreen(xCm: number, yCm: number): { x: number; y: number } {
    return {
      x: this.transform.offsetX + xCm * this.transform.scale,
      y: this.transform.offsetY + yCm * this.transform.scale,
    };
  }

  public toWorld(screenX: number, screenY: number): { x: number; y: number } {
    return {
      x: (screenX - this.transform.offsetX) / this.transform.scale,
      y: (screenY - this.transform.offsetY) / this.transform.scale,
    };
  }

  public render(): void {
    const ctx = this.ctx;
    const { width, height } = this.canvas;

    ctx.save();
    ctx.clearRect(0, 0, width, height);

    // 1. Blueprint Deep Blue Background (Exact shade of Image 1: deep royal circuit blue)
    const bgGrad = ctx.createLinearGradient(0, 0, width, height);
    bgGrad.addColorStop(0, '#020b66');
    bgGrad.addColorStop(1, '#01084d');
    ctx.fillStyle = bgGrad;
    ctx.fillRect(0, 0, width, height);

    // Track frame background rectangle
    const p0 = this.toScreen(0, 0);
    const pEnd = this.toScreen(TRACK_WIDTH_CM, TRACK_HEIGHT_CM);
    const trackW = pEnd.x - p0.x;
    const trackH = pEnd.y - p0.y;

    ctx.fillStyle = '#020d75';
    ctx.fillRect(p0.x, p0.y, trackW, trackH);

    // 2. Blueprint Grid Lines (every 20 cm)
    if (this.options.showGrid) {
      ctx.save();
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.05)';
      ctx.lineWidth = 1;
      for (let x = 0; x <= TRACK_WIDTH_CM; x += 20) {
        const top = this.toScreen(x, 0);
        const bot = this.toScreen(x, TRACK_HEIGHT_CM);
        ctx.beginPath();
        ctx.moveTo(top.x, top.y);
        ctx.lineTo(bot.x, bot.y);
        ctx.stroke();
      }
      for (let y = 0; y <= TRACK_HEIGHT_CM; y += 20) {
        const left = this.toScreen(0, y);
        const right = this.toScreen(TRACK_WIDTH_CM, y);
        ctx.beginPath();
        ctx.moveTo(left.x, left.y);
        ctx.lineTo(right.x, right.y);
        ctx.stroke();
      }
      ctx.restore();
    }

    // Outer border frame (border border double line)
    ctx.save();
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.25)';
    ctx.lineWidth = 2;
    ctx.strokeRect(p0.x - 8, p0.y - 8, trackW + 16, trackH + 16);
    ctx.restore();

    // 3. Render White Track Line Segments
    ctx.save();
    const lineWidthPx = Math.max(2, TRACK_LINE_WIDTH_CM * this.transform.scale);
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    for (let i = 0; i < TRACK_SEGMENTS.length; ++i) {
      const seg = TRACK_SEGMENTS[i];
      const pt1 = this.toScreen(seg.p1.x, seg.p1.y);
      const pt2 = this.toScreen(seg.p2.x, seg.p2.y);

      ctx.beginPath();
      ctx.moveTo(pt1.x, pt1.y);
      ctx.lineTo(pt2.x, pt2.y);

      if (seg.dashed) {
        const dashPx = (seg.dashLength || 5.0) * this.transform.scale;
        const gapPx = (seg.gapLength || 5.0) * this.transform.scale;
        ctx.setLineDash([dashPx, gapPx]);
      } else {
        ctx.setLineDash([]);
      }

      ctx.lineWidth = lineWidthPx;
      ctx.strokeStyle = '#ffffff';
      ctx.stroke();
    }
    ctx.restore();

    // 4. Render Circuit Component Symbols (Resistors, Transistors, Battery, Diode)
    for (let i = 0; i < CIRCUIT_COMPONENTS.length; ++i) {
      this.renderComponent(CIRCUIT_COMPONENTS[i]);
    }

    // 5. Render CAD Dimension Annotations (matching Image 2)
    if (this.options.showDimensions) {
      this.renderDimensions();
    }

    ctx.restore();
  }

  /**
   * Draw one printed component symbol at its true physical size (cm) on the
   * track. The board covers the wire underneath the symbol footprint, then the
   * white symbol strokes are drawn, matching what the grayscale sensors see.
   */
  private renderComponent(comp: TrackComponent): void {
    const ctx = this.ctx;
    const { x, y } = this.toScreen(comp.x, comp.y);
    const scale = this.transform.scale;
    const px = (cm: number) => cm * scale;
    const board = '#020d75';

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(comp.rotation);
    ctx.strokeStyle = '#ffffff';
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    if (comp.type === 'battery') {
      // Four plates straddling the wire: long, short, long, short.
      const footprintHalfU = 2 * BATTERY_PLATE_SPACING_CM + BATTERY_STROKE_CM;
      const footprintHalfV = BATTERY_LONG_HALF_CM + BATTERY_STROKE_CM;
      ctx.fillStyle = board;
      ctx.fillRect(px(-footprintHalfU), px(-footprintHalfV), px(2 * footprintHalfU), px(2 * footprintHalfV));
      ctx.lineWidth = Math.max(1.2, px(BATTERY_STROKE_CM));
      const half = [BATTERY_LONG_HALF_CM, BATTERY_SHORT_HALF_CM, BATTERY_LONG_HALF_CM, BATTERY_SHORT_HALF_CM];
      for (let i = 0; i < 4; ++i) {
        const u = (i - 1.5) * BATTERY_PLATE_SPACING_CM;
        ctx.beginPath();
        ctx.moveTo(px(u), px(-half[i]));
        ctx.lineTo(px(u), px(half[i]));
        ctx.stroke();
      }
    } else if (comp.type === 'resistor') {
      const L = comp.length ?? RESISTOR_LENGTH_CM;
      const W = comp.width ?? RESISTOR_WIDTH_CM;
      const t = comp.stroke ?? RESISTOR_STROKE_CM;
      ctx.fillStyle = board;
      ctx.fillRect(px(-L / 2), px(-W / 2), px(L), px(W));
      ctx.lineWidth = Math.max(1.2, px(t));
      ctx.strokeRect(px(-L / 2), px(-W / 2), px(L), px(W));
    } else if (comp.type === 'transistor') {
      // Canonical: through-wire along u (+x), V dips toward the branch (+v),
      // base bar perpendicular to the branch at the V vertex.
      const m = TRANSISTOR_MOUTH_CM / 2;
      const d = TRANSISTOR_DEPTH_CM;
      const t = comp.stroke ?? TRANSISTOR_STROKE_CM;
      // Board patch = map_data.insideFootprint: stop exactly at the round
      // stroke caps (u half = max(mouth/2, bar half) + t/2, v = -t/2..d+t/2).
      // A larger rectangle would hide wire the sensor still sees (or the bar
      // would paint white over a rectangle the sensor ignores).
      const footprintHalfU = Math.max(m, TRANSISTOR_BAR_HALF_CM) + t / 2;
      ctx.fillStyle = board;
      ctx.fillRect(px(-footprintHalfU), px(-t / 2), px(2 * footprintHalfU), px(d + t));
      ctx.lineWidth = Math.max(1.2, px(t));
      ctx.beginPath();
      ctx.moveTo(px(-m), 0);
      ctx.lineTo(0, px(d));
      ctx.lineTo(px(m), 0);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(px(-TRANSISTOR_BAR_HALF_CM), px(d));
      ctx.lineTo(px(TRANSISTOR_BAR_HALF_CM), px(d));
      ctx.stroke();
    }

    ctx.restore();

    // Component label clear of the symbol
    if (this.options.showComponentLabels && comp.label) {
      const label = this.toScreen(comp.x, comp.y);
      ctx.save();
      ctx.fillStyle = 'rgba(255, 255, 255, 0.65)';
      ctx.font = `${Math.max(9, Math.min(14, 2.6 * scale))}px monospace`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      const outward = comp.type === 'resistor' ? RESISTOR_WIDTH_CM / 2 : TRANSISTOR_DEPTH_CM;
      ctx.fillText(comp.label, label.x, label.y + px(outward) + 4);
      ctx.restore();
    }
  }

  private renderDimensions(): void {
    const ctx = this.ctx;
    const scale = this.transform.scale;
    const fontSize = Math.max(9, Math.min(13, 2.6 * scale));

    ctx.save();
    ctx.font = `${fontSize}px sans-serif`;
    ctx.fillStyle = '#8bc34a';
    ctx.strokeStyle = 'rgba(139, 195, 74, 0.4)';
    ctx.lineWidth = 1;

    const drawHorizDim = (x1: number, x2: number, y: number, text: string, textAbove = true) => {
      const p1 = this.toScreen(x1, y);
      const p2 = this.toScreen(x2, y);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
      const a = 3;
      ctx.beginPath();
      ctx.moveTo(p1.x + a, p1.y - a);
      ctx.lineTo(p1.x, p1.y);
      ctx.lineTo(p1.x + a, p1.y + a);
      ctx.moveTo(p2.x - a, p2.y - a);
      ctx.lineTo(p2.x, p2.y);
      ctx.lineTo(p2.x - a, p2.y + a);
      ctx.stroke();
      ctx.textAlign = 'center';
      ctx.textBaseline = textAbove ? 'bottom' : 'top';
      ctx.fillText(text, (p1.x + p2.x) / 2, p1.y + (textAbove ? -2 : 2));
    };

    const drawVertDim = (x: number, y1: number, y2: number, text: string, textLeft = true) => {
      const p1 = this.toScreen(x, y1);
      const p2 = this.toScreen(x, y2);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
      const a = 3;
      ctx.beginPath();
      ctx.moveTo(p1.x - a, p1.y + a);
      ctx.lineTo(p1.x, p1.y);
      ctx.lineTo(p1.x + a, p1.y + a);
      ctx.moveTo(p2.x - a, p2.y - a);
      ctx.lineTo(p2.x, p2.y);
      ctx.lineTo(p2.x + a, p2.y - a);
      ctx.stroke();
      ctx.textAlign = textLeft ? 'right' : 'left';
      ctx.textBaseline = 'middle';
      ctx.fillText(text, p1.x + (textLeft ? -4 : 4), (p1.y + p2.y) / 2);
    };

    // Top dimension chain (new CAD drawing): 55/40/150/30/70/30
    const topY = -8;
    drawHorizDim(0, 55, topY, '55.00');
    drawHorizDim(55, 95, topY, '40.00');
    drawHorizDim(95, 245, topY, '150.00');
    drawHorizDim(245, 275, topY, '30.00');
    drawHorizDim(275, 345, topY, '70.00');
    drawHorizDim(345, 375, topY, '30.00');
    drawHorizDim(0, 375, topY - 10, '375.00');

    // Left chain: 140 + 80 = 220
    drawVertDim(-8, 0, 140, '140.00');
    drawVertDim(-8, 140, 220, '80.00');
    drawVertDim(-18, 0, 220, '220.00');

    // Right chain: 110 + 55 + 55
    drawVertDim(383, 110, 220, '110.00', false);
    drawVertDim(370, 110, 165, '55.00', false);
    drawVertDim(370, 50, 110, '60.00', false);

    // Middle box and hexagon dimensions
    drawHorizDim(165, 245, 34, '80.00');
    drawVertDim(160, 40, 80, '40.00');
    drawHorizDim(292.5, 327.5, 130, '35.00');
    drawVertDim(350, 110, 165, '55.00', false);

    // Bottom chain: 55/40/55/95/100/30
    const botY = 228;
    drawHorizDim(0, 55, botY, '55.00', false);
    drawHorizDim(55, 95, botY, '40.00', false);
    drawHorizDim(95, 150, botY, '55.00', false);
    drawHorizDim(150, 245, botY, '95.00', false);
    drawHorizDim(245, 345, botY, '100.00', false);
    drawHorizDim(345, 375, botY, '30.00', false);

    ctx.restore();
  }
}
