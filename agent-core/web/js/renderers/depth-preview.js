import { paintDepth } from './camera.js';

const WIDTH = 640;
const HEIGHT = 480;

const opticalFrame = info => info.vendor?.ros_frame_id || info.vendor?.frame_id;

function sameCalibration(a, b) {
  return a.width === WIDTH && a.height === HEIGHT && a.width === b.width && a.height === b.height
    && opticalFrame(a) && opticalFrame(a) === opticalFrame(b)
    && (!a.vendor?.serial || !b.vendor?.serial || String(a.vendor.serial) === String(b.vendor.serial))
    && a.distortion_model === b.distortion_model
    && a.K?.length === 9 && b.K?.length === 9 && a.K[0] > 0 && a.K[4] > 0
    && a.K.every((n, i) => Number.isFinite(n) && Number.isFinite(b.K[i]) && Math.abs(n - b.K[i]) < 0.01)
    && Array.isArray(a.D) && a.D.length === b.D?.length
    && a.D.every((n, i) => Number.isFinite(n) && Number.isFinite(b.D[i]) && Math.abs(n - b.D[i]) < 0.00001);
}

export function pairedColorTopic(depthTopic, declarations) {
  const depth = declarations.find(info => info.topic === depthTopic && info.format === 'image/depth-zlib');
  if (!depth) return null;
  const candidates = declarations.filter(info => info.format === 'image/jpeg' && sameCalibration(depth, info));
  const preferred = [...new Set(candidates.filter(info => info.id && info.id === depth.id).map(info => info.topic))];
  const topics = [...new Set(candidates.map(info => info.topic))];
  return preferred.length === 1 ? preferred[0] : topics.length === 1 ? topics[0] : null;
}

export function colorEdges(rgba, width, height) {
  const edges = new Uint8Array(width * height);
  const stride = width * 4;
  for (let y = 1; y < height - 1; y++) {
    for (let x = 1; x < width - 1; x++) {
      const i = y * width + x;
      for (let channel = 0; channel < 3; channel++) {
        const p = 4 * i + channel, a = p - stride, b = p + stride;
        const dx = rgba[a + 4] - rgba[a - 4] + 2 * (rgba[p + 4] - rgba[p - 4]) + rgba[b + 4] - rgba[b - 4];
        const dy = rgba[b - 4] - rgba[a - 4] + 2 * (rgba[b] - rgba[a]) + rgba[b + 4] - rgba[a + 4];
        if (Math.abs(dx) + Math.abs(dy) >= 112) {
          edges[i] = 1;
          break;
        }
      }
    }
  }
  return edges;
}

export class DepthPreview {
  constructor(topic, resolvePair, connect) {
    this.topic = topic;
    this.resolvePair = resolvePair;
    this.connect = connect;
    this.generation = 0;
    this.sequence = 0;
    this.displayed = 0;
    this.fpsAt = performance.now();
  }

  mount(container) {
    container.innerHTML = `
      <div class="depth-preview">
        <div class="depth-preview-tools">
          <label><input type="checkbox" class="depth-outline"> 轮廓叠加</label>
          <label>近 <input type="number" class="depth-near" min="1" max="65534" step="1" placeholder="默认" aria-label="深度色阶近端（毫米）"></label>
          <label>远 <input type="number" class="depth-far" min="2" max="65535" step="1" placeholder="默认" aria-label="深度色阶远端（毫米）"></label>
          mm <button type="button" class="depth-reset">重置</button>
        </div>
        <div class="depth-preview-image">
          <canvas width="${WIDTH}" height="${HEIGHT}" aria-label="深度预览"></canvas>
          <span class="depth-preview-fps"></span>
          <output class="depth-preview-status" role="status"></output>
        </div>
      </div>`;
    this.ctx = container.querySelector('canvas').getContext('2d');
    this.image = this.ctx.createImageData(WIDTH, HEIGHT);
    this.fps = container.querySelector('.depth-preview-fps');
    this.status = container.querySelector('.depth-preview-status');
    this.toggle = container.querySelector('.depth-outline');
    this.near = container.querySelector('.depth-near');
    this.far = container.querySelector('.depth-far');
    this.toggle.addEventListener('change', () => this.setOverlay());
    for (const input of [this.near, this.far]) input.addEventListener('input', () => this.updateRange());
    container.querySelector('.depth-reset').addEventListener('click', () => {
      this.near.value = this.far.value = '';
      this.updateRange();
    });
  }

  updateRange() {
    const near = this.near.valueAsNumber, far = this.far.valueAsNumber;
    const valid = this.near.validity.valid && this.far.validity.valid && near < far;
    const incomplete = this.near.value || this.far.value;
    for (const input of [this.near, this.far]) input.setAttribute('aria-invalid', String(Boolean(incomplete && !valid)));
    this.range = valid ? [near, far] : null;
    this.draw(this.frame, this.frame?.edges);
  }

  async setOverlay() {
    const id = ++this.generation;
    this.pairFraming = false;
    this.wantsPair = false;
    if (this.frame) this.frame.edges = null;
    this.draw(this.frame);
    this.connect('');
    this.status.textContent = this.toggle.checked ? '正在匹配彩色流…' : '';
    if (!this.toggle.checked) return;
    const rgb = await this.resolvePair(this.topic);
    if (id !== this.generation || !this.ctx) return;
    this.wantsPair = Boolean(rgb);
    this.status.textContent = rgb ? '等待同步画面…' : '暂无匹配的彩色流';
    if (rgb) this.connect(rgb);
  }

  onStreamMeta(meta) {
    this.pairFraming = meta.preview_pair === 'rgb-depth-v1';
  }

  async onData(bytes) {
    const generation = this.generation, sequence = ++this.sequence;
    let bitmap, colorSize = 0, pixels, edges;
    try {
      if (this.pairFraming) {
        if (bytes.byteLength < 5) return;
        colorSize = new DataView(bytes).getUint32(0);
        if (colorSize > bytes.byteLength - 5) return;
      }
      const depth = this.pairFraming ? bytes.slice(4 + colorSize) : bytes;
      const stream = new Blob([depth]).stream().pipeThrough(new DecompressionStream('deflate'));
      const raw = await new Response(stream).arrayBuffer();
      if (raw.byteLength !== WIDTH * HEIGHT * 2 || generation !== this.generation) return;
      pixels = new Uint16Array(raw);
      if (colorSize) {
        bitmap = await createImageBitmap(new Blob([bytes.slice(4, 4 + colorSize)], { type: 'image/jpeg' }));
        if (generation !== this.generation) return;
        if (bitmap.width === WIDTH && bitmap.height === HEIGHT) {
          if (!this.colorCtx) {
            const canvas = document.createElement('canvas');
            canvas.width = WIDTH;
            canvas.height = HEIGHT;
            this.colorCtx = canvas.getContext('2d', { willReadFrequently: true });
          }
          this.colorCtx.drawImage(bitmap, 0, 0);
          edges = colorEdges(this.colorCtx.getImageData(0, 0, WIDTH, HEIGHT).data, WIDTH, HEIGHT);
        }
      }
    } catch (error) {
      console.warn('[depth-preview] frame decode:', error);
      return;
    } finally {
      bitmap?.close();
    }
    if (!this.ctx || generation !== this.generation || sequence <= (this.presentedSequence || 0)) return;
    this.presentedSequence = sequence;
    this.frame = { pixels, edges };
    if (this.wantsPair) this.status.textContent = edges ? '' : '等待同步画面…';
    this.draw(this.frame, edges);
  }

  draw(frame, edges) {
    if (!frame || !this.ctx) return;
    const { pixels } = frame;
    const rgba = this.image.data;
    paintDepth(pixels, rgba, pixels.length, this.range);
    if (edges) {
      for (let i = 0; i < edges.length; i++) {
        if (!edges[i] || !pixels[i]) continue;
        const p = 4 * i;
        rgba[p] = 46;
        rgba[p + 1] = 246;
        rgba[p + 2] = 209;
      }
    }
    this.ctx.putImageData(this.image, 0, 0);
    this.displayed++;
    const now = performance.now();
    if (now - this.fpsAt >= 1000) {
      this.fps.textContent = `${Math.round(this.displayed * 1000 / (now - this.fpsAt))} fps`;
      this.displayed = 0;
      this.fpsAt = now;
    }
  }

  unmount() {
    ++this.generation;
    this.ctx = this.colorCtx = null;
    this.frame = null;
  }
}
