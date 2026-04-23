import { render } from "preact";
import { useEffect, useMemo, useRef, useState } from "preact/hooks";
import "./styles.css";

type Status = {
  fw_version: string;
  ip: string;
  wifi_rssi: number;
  wifi_tx_dbm: number;
  free_heap_kb: number;
  min_free_heap_kb: number;
  uptime: string;
  rtsp_server_enabled: boolean;
  client: string;
  streaming: boolean;
  current_rate_pkt_s: number;
  rtp_packets_sent: number;
  rtp_packets_dropped: number;
  last_rtsp_connect: string;
  last_stream_start: string;
  rtsp_transport: string;
  active_rtsp_clients: number;
  connected_rtsp_clients: number;
  max_rtsp_clients: number;
  rtsp_rejected_clients: number;
  stream_formats: string;
  multi_client_policy: string;
  hardware_profile: string;
};

type AudioStatus = {
  sample_rate: number;
  gain: number;
  buffer_size: number;
  i2s_shift: number;
  peak_pct: number;
  peak_dbfs: number;
  clip: boolean;
  clip_count: number;
  latency_ms: number;
  hp_enable: boolean;
  hp_cutoff_hz: number;
  agc_enable: boolean;
  agc_multiplier: number;
  effective_gain: number;
  noise_filter_enable: boolean;
  noise_floor_dbfs: number;
  noise_gate_dbfs: number;
  noise_reduction_db: number;
  audio_pipeline_load_pct: number;
  i2s_fallback_blocks: number;
  i2s_last_gap_ms: number;
  i2s_driver_ok: boolean;
  i2s_last_error: number;
  i2s_link_ok: boolean;
  i2s_hint: string;
  led_mode: number;
};

type PerfStatus = {
  restart_threshold_pkt_s: number;
  check_interval_min: number;
  auto_recovery: boolean;
  auto_threshold: boolean;
  recommended_min_rate: number;
  scheduled_reset: boolean;
  reset_hours: number;
};

type ThermalStatus = {
  current_c: number | null;
  current_valid: boolean;
  max_c: number;
  cpu_mhz: number;
  protection_enabled: boolean;
  shutdown_c: number;
  latched: boolean;
  latched_persist: boolean;
  sensor_fault: boolean;
  last_trip_c: number;
  last_reason: string;
  last_trip_ts: string;
  last_trip_since: string;
  manual_restart: boolean;
};

type FftStatus = {
  seq: number;
  bins: number[];
  sample_rate: number;
  max_hz: number;
};

type Drafts = Record<string, string>;

type Rgb = [number, number, number];
type SpectrogramPalette = "classic" | "viridis" | "fire" | "ice" | "gray";
type SpectrogramStyle = "timeline" | "waterfall" | "contour" | "bands";
type SpectrogramScale = "linear" | "log";

const paletteOptions: Array<{ value: SpectrogramPalette; label: string }> = [
  { value: "classic", label: "Classic" },
  { value: "viridis", label: "Viridis" },
  { value: "fire", label: "Fire" },
  { value: "ice", label: "Ice" },
  { value: "gray", label: "Gray" }
];

const styleOptions: Array<{ value: SpectrogramStyle; label: string }> = [
  { value: "timeline", label: "Timeline" },
  { value: "waterfall", label: "Waterfall" },
  { value: "contour", label: "Contours" },
  { value: "bands", label: "Live bands" }
];

const scaleOptions: Array<{ value: SpectrogramScale; label: string }> = [
  { value: "linear", label: "Linear Hz" },
  { value: "log", label: "Log Hz" }
];

const paletteValues: readonly SpectrogramPalette[] = ["classic", "viridis", "fire", "ice", "gray"];
const styleValues: readonly SpectrogramStyle[] = ["timeline", "waterfall", "contour", "bands"];
const scaleValues: readonly SpectrogramScale[] = ["linear", "log"];
const supportedSampleRates = ["16000", "24000", "32000", "48000"] as const;

const paletteStops: Record<SpectrogramPalette, Array<[number, Rgb]>> = {
  classic: [
    [0, [0, 0, 12]],
    [0.24, [0, 40, 120]],
    [0.48, [0, 190, 210]],
    [0.72, [250, 238, 80]],
    [1, [255, 95, 34]]
  ],
  viridis: [
    [0, [68, 1, 84]],
    [0.25, [59, 82, 139]],
    [0.5, [33, 145, 140]],
    [0.75, [94, 201, 98]],
    [1, [253, 231, 37]]
  ],
  fire: [
    [0, [2, 0, 12]],
    [0.32, [82, 18, 90]],
    [0.58, [190, 48, 67]],
    [0.82, [248, 149, 64]],
    [1, [255, 246, 168]]
  ],
  ice: [
    [0, [0, 8, 18]],
    [0.3, [20, 70, 110]],
    [0.58, [54, 169, 191]],
    [0.8, [153, 231, 208]],
    [1, [245, 255, 244]]
  ],
  gray: [
    [0, [0, 0, 0]],
    [1, [245, 245, 245]]
  ]
};

const emptyStatus: Status = {
  fw_version: "",
  ip: "",
  wifi_rssi: 0,
  wifi_tx_dbm: 0,
  free_heap_kb: 0,
  min_free_heap_kb: 0,
  uptime: "",
  rtsp_server_enabled: false,
  client: "",
  streaming: false,
  current_rate_pkt_s: 0,
  rtp_packets_sent: 0,
  rtp_packets_dropped: 0,
  last_rtsp_connect: "",
  last_stream_start: "",
  rtsp_transport: "tcp",
  active_rtsp_clients: 0,
  connected_rtsp_clients: 0,
  max_rtsp_clients: 2,
  rtsp_rejected_clients: 0,
  stream_formats: "RTSP/RTP L16, WebAudio PCM, WAV PCM chunk, JSON PCM",
  multi_client_policy: "bounded_multi_rtsp_tcp",
  hardware_profile: "PDM input profile"
};

const emptyAudio: AudioStatus = {
  sample_rate: 16000,
  gain: 1,
  buffer_size: 1024,
  i2s_shift: 0,
  peak_pct: 0,
  peak_dbfs: -90,
  clip: false,
  clip_count: 0,
  latency_ms: 64,
  hp_enable: true,
  hp_cutoff_hz: 180,
  agc_enable: false,
  agc_multiplier: 1,
  effective_gain: 1,
  noise_filter_enable: false,
  noise_floor_dbfs: -90,
  noise_gate_dbfs: -90,
  noise_reduction_db: 0,
  audio_pipeline_load_pct: 0,
  i2s_fallback_blocks: 0,
  i2s_last_gap_ms: 0,
  i2s_driver_ok: true,
  i2s_last_error: 0,
  i2s_link_ok: true,
  i2s_hint: "",
  led_mode: 1
};

const emptyPerf: PerfStatus = {
  restart_threshold_pkt_s: 5,
  check_interval_min: 15,
  auto_recovery: false,
  auto_threshold: true,
  recommended_min_rate: 8,
  scheduled_reset: false,
  reset_hours: 24
};

const emptyThermal: ThermalStatus = {
  current_c: null,
  current_valid: false,
  max_c: 0,
  cpu_mhz: 160,
  protection_enabled: true,
  shutdown_c: 80,
  latched: false,
  latched_persist: false,
  sensor_fault: false,
  last_trip_c: 0,
  last_reason: "",
  last_trip_ts: "",
  last_trip_since: "",
  manual_restart: false
};

async function getJson<T>(url: string): Promise<T> {
  const response = await fetch(url, { cache: "no-store" });
  const body = await response.text();
  if (!response.ok) throw new Error(body || `${response.status} ${response.statusText}`);
  return (body ? JSON.parse(body) : {}) as T;
}

async function postJson<T>(url: string): Promise<T> {
  const response = await fetch(url, { method: "POST", cache: "no-store" });
  const body = await response.text();
  if (!response.ok) throw new Error(body || `${response.status} ${response.statusText}`);
  return (body ? JSON.parse(body) : {}) as T;
}

async function action(name: string) {
  const result = await getJson<{ ok: boolean; error?: string }>(`/api/action/${encodeURIComponent(name)}`);
  if (!result.ok) throw new Error(result.error || "Action was rejected");
}

async function setValue(key: string, value: string) {
  const result = await getJson<{ ok: boolean; error?: string }>(
    `/api/set?key=${encodeURIComponent(key)}&value=${encodeURIComponent(value)}`
  );
  if (!result.ok) throw new Error(result.error || "Setting was rejected");
}

function clamp01(value: number) {
  if (!Number.isFinite(value)) return 0;
  return Math.max(0, Math.min(1, value));
}

function lerp(a: number, b: number, amount: number) {
  return a + (b - a) * amount;
}

function colorFromPalette(palette: SpectrogramPalette, value: number, quantized = false): Rgb {
  let level = Math.pow(clamp01(value), 0.72);
  if (quantized) level = Math.floor(level * 7) / 7;
  const stops = paletteStops[palette];
  for (let i = 1; i < stops.length; i++) {
    const [stopAt, stopColor] = stops[i];
    const [prevAt, prevColor] = stops[i - 1];
    if (level <= stopAt) {
      const mix = stopAt === prevAt ? 0 : (level - prevAt) / (stopAt - prevAt);
      return [
        Math.round(lerp(prevColor[0], stopColor[0], mix)),
        Math.round(lerp(prevColor[1], stopColor[1], mix)),
        Math.round(lerp(prevColor[2], stopColor[2], mix))
      ];
    }
  }
  return stops[stops.length - 1][1];
}

function binForPosition(position: number, count: number, scale: SpectrogramScale) {
  if (count <= 1) return 0;
  const t = clamp01(position);
  const mapped = scale === "log" ? Math.pow(count, t) - 1 : t * (count - 1);
  return Math.max(0, Math.min(count - 1, Math.round(mapped)));
}

function getStoredOption<T extends string>(key: string, fallback: T, allowed: readonly T[]) {
  const stored = window.localStorage.getItem(key);
  return stored && allowed.indexOf(stored as T) >= 0 ? (stored as T) : fallback;
}

function resizeCanvasToDisplay(canvas: HTMLCanvasElement) {
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const width = Math.max(240, Math.round((rect.width || 320) * dpr));
  const height = Math.max(140, Math.round((rect.height || 220) * dpr));
  if (canvas.width === width && canvas.height === height) return false;
  canvas.width = width;
  canvas.height = height;
  return true;
}

function drawSpectrogramBase(canvas: HTMLCanvasElement, message = "") {
  const ctx = canvas.getContext("2d");
  if (!ctx) return;
  resizeCanvasToDisplay(canvas);
  const { width, height } = canvas;
  ctx.fillStyle = "#090b09";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "rgba(244, 246, 241, 0.08)";
  ctx.lineWidth = 1;
  for (let i = 1; i < 4; i++) {
    const y = (height * i) / 4;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }
  if (message) {
    ctx.fillStyle = "#a9aea5";
    ctx.font = `${Math.max(12, Math.round(height * 0.055))}px system-ui, sans-serif`;
    ctx.fillText(message, 14, Math.round(height / 2));
  }
}

function drawTimelineFrame(
  canvas: HTMLCanvasElement,
  bins: number[],
  palette: SpectrogramPalette,
  scale: SpectrogramScale,
  quantized = false
) {
  const ctx = canvas.getContext("2d");
  if (!ctx || !bins.length) return;
  if (resizeCanvasToDisplay(canvas)) drawSpectrogramBase(canvas);
  const { width, height } = canvas;
  if (width > 1) {
    const previous = ctx.getImageData(1, 0, width - 1, height);
    ctx.putImageData(previous, 0, 0);
  }
  const column = ctx.createImageData(1, height);
  for (let y = 0; y < height; y++) {
    const frequencyPos = 1 - y / Math.max(1, height - 1);
    const bin = binForPosition(frequencyPos, bins.length, scale);
    const color = colorFromPalette(palette, (bins[bin] || 0) / 255, quantized);
    const offset = y * 4;
    column.data[offset] = color[0];
    column.data[offset + 1] = color[1];
    column.data[offset + 2] = color[2];
    column.data[offset + 3] = 255;
  }
  ctx.putImageData(column, width - 1, 0);
}

function drawWaterfallFrame(canvas: HTMLCanvasElement, bins: number[], palette: SpectrogramPalette, scale: SpectrogramScale) {
  const ctx = canvas.getContext("2d");
  if (!ctx || !bins.length) return;
  if (resizeCanvasToDisplay(canvas)) drawSpectrogramBase(canvas);
  const { width, height } = canvas;
  if (height > 1) {
    const previous = ctx.getImageData(0, 0, width, height - 1);
    ctx.putImageData(previous, 0, 1);
  }
  const row = ctx.createImageData(width, 1);
  for (let x = 0; x < width; x++) {
    const frequencyPos = x / Math.max(1, width - 1);
    const bin = binForPosition(frequencyPos, bins.length, scale);
    const color = colorFromPalette(palette, (bins[bin] || 0) / 255);
    const offset = x * 4;
    row.data[offset] = color[0];
    row.data[offset + 1] = color[1];
    row.data[offset + 2] = color[2];
    row.data[offset + 3] = 255;
  }
  ctx.putImageData(row, 0, 0);
}

function drawBandsFrame(canvas: HTMLCanvasElement, bins: number[], palette: SpectrogramPalette, scale: SpectrogramScale) {
  const ctx = canvas.getContext("2d");
  if (!ctx || !bins.length) return;
  resizeCanvasToDisplay(canvas);
  const { width, height } = canvas;
  drawSpectrogramBase(canvas);
  const visible = Math.min(48, bins.length);
  const gap = Math.max(1, Math.floor(width / 240));
  const bandWidth = width / visible;
  for (let i = 0; i < visible; i++) {
    const position = visible <= 1 ? 0 : i / (visible - 1);
    const bin = binForPosition(position, bins.length, scale);
    const level = clamp01((bins[bin] || 0) / 255);
    const color = colorFromPalette(palette, level);
    const x = Math.round(i * bandWidth);
    const barWidth = Math.max(2, Math.ceil(bandWidth) - gap);
    const barHeight = Math.max(2, Math.round(level * (height - 16)));
    ctx.fillStyle = `rgb(${color[0]}, ${color[1]}, ${color[2]})`;
    ctx.fillRect(x, height - barHeight, barWidth, barHeight);
  }
}

function drawSpectrogramFrame(
  canvas: HTMLCanvasElement | null,
  bins: number[],
  palette: SpectrogramPalette,
  style: SpectrogramStyle,
  scale: SpectrogramScale
) {
  if (!canvas) return;
  if (style === "waterfall") {
    drawWaterfallFrame(canvas, bins, palette, scale);
  } else if (style === "bands") {
    drawBandsFrame(canvas, bins, palette, scale);
  } else {
    drawTimelineFrame(canvas, bins, palette, scale, style === "contour");
  }
}

function Pill({
  children,
  tone = "neutral"
}: {
  children: preact.ComponentChildren;
  tone?: "neutral" | "ok" | "warn" | "bad";
}) {
  return <span class={`pill ${tone}`}>{children}</span>;
}

function Stat({ label, value }: { label: string; value: preact.ComponentChildren }) {
  return (
    <div class="stat">
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

function Metric({
  label,
  value,
  detail,
  tone = "neutral"
}: {
  label: string;
  value: preact.ComponentChildren;
  detail: string;
  tone?: "neutral" | "ok" | "warn" | "bad";
}) {
  return (
    <article class={`metric ${tone}`}>
      <span>{label}</span>
      <strong>{value}</strong>
      <small>{detail}</small>
    </article>
  );
}

function Setting({
  label,
  detail,
  children
}: {
  label: string;
  detail?: preact.ComponentChildren;
  children: preact.ComponentChildren;
}) {
  return (
    <div class="setting">
      <div>
        <span>{label}</span>
        {detail && <small>{detail}</small>}
      </div>
      {children}
    </div>
  );
}

function SpectrogramPanel({ sampleRate, streaming }: { sampleRate: number; streaming: boolean }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const lastSeqRef = useRef(0);
  const optionsRef = useRef<{
    palette: SpectrogramPalette;
    style: SpectrogramStyle;
    scale: SpectrogramScale;
  }>({ palette: "classic", style: "timeline", scale: "linear" });
  const [palette, setPalette] = useState<SpectrogramPalette>(() =>
    getStoredOption("spectrogram.palette", "classic", paletteValues)
  );
  const [style, setStyle] = useState<SpectrogramStyle>(() =>
    getStoredOption("spectrogram.style", "timeline", styleValues)
  );
  const [scale, setScale] = useState<SpectrogramScale>(() =>
    getStoredOption("spectrogram.scale", "linear", scaleValues)
  );
  const [latest, setLatest] = useState<FftStatus>({ seq: 0, bins: [], sample_rate: sampleRate, max_hz: sampleRate / 2 });
  const [fftError, setFftError] = useState("");

  useEffect(() => {
    optionsRef.current = { palette, style, scale };
    window.localStorage.setItem("spectrogram.palette", palette);
    window.localStorage.setItem("spectrogram.style", style);
    window.localStorage.setItem("spectrogram.scale", scale);
    if (canvasRef.current) drawSpectrogramBase(canvasRef.current, latest.seq ? "" : "Waiting for FFT...");
  }, [palette, style, scale]);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const resize = () => drawSpectrogramBase(canvas, lastSeqRef.current ? "" : "Waiting for FFT...");
    resize();
    window.addEventListener("resize", resize);
    const observer = typeof ResizeObserver !== "undefined" ? new ResizeObserver(resize) : null;
    if (observer) observer.observe(canvas);
    return () => {
      window.removeEventListener("resize", resize);
      if (observer) observer.disconnect();
    };
  }, []);

  useEffect(() => {
    let alive = true;
    let timer = 0;

    const poll = async () => {
      try {
        const frame = await getJson<FftStatus>("/api/fft");
        if (!alive) return;
        const bins = Array.isArray(frame.bins)
          ? frame.bins.map((value) => Math.max(0, Math.min(255, Number(value) || 0)))
          : [];
        if (bins.length) setFftError("");
        if (bins.length && frame.seq !== lastSeqRef.current) {
          const nextFrame = { ...frame, bins };
          lastSeqRef.current = frame.seq;
          setLatest(nextFrame);
          drawSpectrogramFrame(
            canvasRef.current,
            bins,
            optionsRef.current.palette,
            optionsRef.current.style,
            optionsRef.current.scale
          );
        }
      } catch (err) {
        if (alive) setFftError(err instanceof Error ? err.message : "FFT unavailable");
      }
      if (alive) timer = window.setTimeout(poll, 240);
    };

    poll();
    return () => {
      alive = false;
      window.clearTimeout(timer);
    };
  }, []);

  const maxHz = latest.max_hz || sampleRate / 2;
  const bandCount = latest.bins.length || 32;
  const stateTone = fftError ? "bad" : latest.seq ? "ok" : "neutral";
  const horizontalFrequency = style === "waterfall" || style === "bands";

  return (
    <section class="spectrogram-panel">
      <div class="spectrogram-head">
        <div>
          <span>Live spectrogram</span>
          <strong>{`${bandCount} FFT bands`}</strong>
          <p>Amplitude by color, processed audio</p>
        </div>
        <div class="spectrogram-state">
          <Pill tone={stateTone}>{fftError ? "FFT unavailable" : latest.seq ? "FFT live" : "Waiting"}</Pill>
          <Pill>{`0-${Math.round(maxHz)} Hz`}</Pill>
          <Pill tone={streaming ? "ok" : "neutral"}>{streaming ? "Streaming" : "Diagnostics"}</Pill>
        </div>
      </div>

      <div class="spectrogram-frame">
        <canvas ref={canvasRef} aria-label="Live audio spectrogram" />
        {horizontalFrequency ? (
          <>
            <span class="axis axis-low-x">Low Hz</span>
            <span class="axis axis-high-x">High Hz</span>
          </>
        ) : (
          <>
            <span class="axis axis-high">High Hz</span>
            <span class="axis axis-low">Low Hz</span>
          </>
        )}
        <span class="axis axis-time">{style === "waterfall" ? "Time downward" : style === "bands" ? "Amplitude" : "Time rightward"}</span>
      </div>

      <div class="spectrogram-controls">
        <label>
          <span>Palette</span>
          <select value={palette} onInput={(event) => setPalette(event.currentTarget.value as SpectrogramPalette)}>
            {paletteOptions.map((option) => (
              <option value={option.value} key={option.value}>
                {option.label}
              </option>
            ))}
          </select>
        </label>
        <label>
          <span>Style</span>
          <select value={style} onInput={(event) => setStyle(event.currentTarget.value as SpectrogramStyle)}>
            {styleOptions.map((option) => (
              <option value={option.value} key={option.value}>
                {option.label}
              </option>
            ))}
          </select>
        </label>
        <label>
          <span>Scale</span>
          <select value={scale} onInput={(event) => setScale(event.currentTarget.value as SpectrogramScale)}>
            {scaleOptions.map((option) => (
              <option value={option.value} key={option.value}>
                {option.label}
              </option>
            ))}
          </select>
        </label>
      </div>
    </section>
  );
}

function App() {
  const [status, setStatus] = useState<Status>(emptyStatus);
  const [audio, setAudio] = useState<AudioStatus>(emptyAudio);
  const [perf, setPerf] = useState<PerfStatus>(emptyPerf);
  const [thermal, setThermal] = useState<ThermalStatus>(emptyThermal);
  const [draft, setDraft] = useState<Drafts>({});
  const [dirty, setDirty] = useState<Record<string, boolean>>({});
  const [error, setError] = useState("");
  const dirtyRef = useRef<Record<string, boolean>>({});

  const rtspUrl = useMemo(() => {
    const ip = status.ip || window.location.hostname;
    return `rtsp://${ip}:8554/audio`;
  }, [status.ip]);

  const levelPct = Math.max(0, Math.min(100, ((audio.peak_dbfs + 60) / 60) * 100));
  const levelTone = audio.clip ? "bad" : audio.peak_pct >= 90 ? "warn" : "ok";
  const wifiTone = status.wifi_rssi > -67 ? "ok" : status.wifi_rssi > -75 ? "warn" : "bad";
  const heapTone = status.free_heap_kb > 96 ? "ok" : status.free_heap_kb > 72 ? "warn" : "bad";
  const tempTone = thermal.sensor_fault || thermal.latched_persist ? "bad" : thermal.current_valid && (thermal.current_c || 0) > thermal.shutdown_c - 5 ? "warn" : "ok";
  const clientSlots = Array.from({ length: Math.max(status.max_rtsp_clients, 1) }, (_, index) => index < status.active_rtsp_clients);
  const visualBars = Array.from({ length: 22 }, (_, index) => {
    const wave = 0.42 + Math.abs(Math.sin((index + 1) * 0.74)) * 0.58;
    return Math.max(10, Math.min(100, levelPct * wave));
  });

  const syncDraft = (values: Drafts) => {
    setDraft((previous) => {
      const next = { ...previous };
      for (const [key, value] of Object.entries(values)) {
        if (!dirtyRef.current[key]) next[key] = value;
      }
      return next;
    });
  };

  const load = async () => {
    const [nextStatus, nextAudio, nextPerf, nextThermal] = await Promise.all([
      getJson<Status>("/api/status"),
      getJson<AudioStatus>("/api/audio_status"),
      getJson<PerfStatus>("/api/perf_status"),
      getJson<ThermalStatus>("/api/thermal")
    ]);
    setStatus(nextStatus);
    setAudio(nextAudio);
    setPerf(nextPerf);
    setThermal(nextThermal);
    syncDraft({
      rate: String(nextAudio.sample_rate),
      gain: nextAudio.gain.toFixed(2),
      buffer: String(nextAudio.buffer_size),
      hp_enable: nextAudio.hp_enable ? "on" : "off",
      hp_cutoff: String(nextAudio.hp_cutoff_hz),
      agc_enable: nextAudio.agc_enable ? "on" : "off",
      noise_filter: nextAudio.noise_filter_enable ? "on" : "off",
      led_mode: String(nextAudio.led_mode || 0),
      wifi_tx: nextStatus.wifi_tx_dbm.toFixed(1),
      auto_recovery: nextPerf.auto_recovery ? "on" : "off",
      thr_mode: nextPerf.auto_threshold ? "auto" : "manual",
      min_rate: String(nextPerf.restart_threshold_pkt_s),
      check_interval: String(nextPerf.check_interval_min),
      sched_reset: nextPerf.scheduled_reset ? "on" : "off",
      reset_hours: String(nextPerf.reset_hours),
      cpu_freq: String(nextThermal.cpu_mhz),
      oh_enable: nextThermal.protection_enabled ? "on" : "off",
      oh_limit: String(nextThermal.shutdown_c)
    });
  };

  useEffect(() => {
    let alive = true;
    const loadSafe = async () => {
      try {
        await load();
        if (alive) setError("");
      } catch (err) {
        if (alive) setError(err instanceof Error ? err.message : "Unable to load status");
      }
    };
    loadSafe();
    const timer = window.setInterval(loadSafe, 1500);
    return () => {
      alive = false;
      window.clearInterval(timer);
    };
  }, []);

  const editDraft = (key: string, value: string) => {
    const nextDirty = { ...dirtyRef.current, [key]: true };
    dirtyRef.current = nextDirty;
    setDirty(nextDirty);
    setDraft((previous) => ({ ...previous, [key]: value }));
  };

  const clearDirty = (key: string) => {
    const nextDirty = { ...dirtyRef.current };
    delete nextDirty[key];
    dirtyRef.current = nextDirty;
    setDirty(nextDirty);
  };

  const applySetting = async (key: string) => {
    try {
      await setValue(key, draft[key] || "");
      clearDirty(key);
      await load();
      setError("");
    } catch (err) {
      setError(err instanceof Error ? err.message : "Setting failed");
    }
  };

  const runAction = async (name: string) => {
    try {
      await action(name);
      await load();
      setError("");
    } catch (err) {
      setError(err instanceof Error ? err.message : "Action failed");
    }
  };

  const clearThermalLatch = async () => {
    try {
      const result = await postJson<{ ok: boolean }>("/api/thermal/clear");
      if (!result.ok) throw new Error("Thermal latch was not active");
      await load();
      setError("");
    } catch (err) {
      setError(err instanceof Error ? err.message : "Thermal clear failed");
    }
  };

  const input = (key: string, fallback = "") => draft[key] ?? fallback;
  const controlClass = (key: string) => (dirty[key] ? "dirty" : "");
  const setButton = (key: string) => (
    <button class="apply" onClick={() => applySetting(key)} disabled={!dirty[key]}>
      Set
    </button>
  );

  return (
    <main class="shell">
      <section class="topbar">
        <div class="brand-block">
          <div class="eyebrow">AtomS3 Lite + Unit Mini PDM</div>
          <h1>M5 Atom RTSP Microphone</h1>
          <p>{status.hardware_profile}</p>
        </div>
        <div class="command-strip">
          <div class="state">
            <Pill tone={status.rtsp_server_enabled ? "ok" : "bad"}>{status.rtsp_server_enabled ? "Server on" : "Server off"}</Pill>
            <Pill tone={status.streaming ? "ok" : "neutral"}>{status.streaming ? "Streaming" : "Ready"}</Pill>
            {status.fw_version && <Pill>v{status.fw_version}</Pill>}
          </div>
          <div class="quick-actions">
            <button class="primary" onClick={() => runAction("server_start")} disabled={status.rtsp_server_enabled}>Server On</button>
            <button onClick={() => runAction("server_stop")} disabled={!status.rtsp_server_enabled}>Server Off</button>
            <button onClick={() => runAction("reset_i2s")}>Reset I2S</button>
          </div>
        </div>
      </section>

      {error && <div class="notice bad">{error}</div>}

      <section class="overview" aria-label="Live summary">
        <Metric label="WiFi" value={`${status.wifi_rssi} dBm`} detail={`${status.wifi_tx_dbm.toFixed(1)} dBm TX`} tone={wifiTone} />
        <Metric label="RTSP" value={`${status.active_rtsp_clients} / ${status.max_rtsp_clients}`} detail={`${status.connected_rtsp_clients} connected`} tone={status.streaming ? "ok" : "neutral"} />
        <Metric label="Packets" value={`${status.current_rate_pkt_s} pkt/s`} detail={`${status.rtp_packets_dropped} dropped`} tone={status.rtp_packets_dropped ? "warn" : "ok"} />
        <Metric label="Temp" value={thermal.current_valid && thermal.current_c !== null ? `${thermal.current_c.toFixed(1)} C` : "N/A"} detail={`${thermal.shutdown_c.toFixed(0)} C limit`} tone={tempTone} />
        <Metric label="Heap" value={`${status.free_heap_kb} KB`} detail={`${status.min_free_heap_kb} KB low`} tone={heapTone} />
      </section>

      <section class="level-panel">
        <div class="level-head">
          <div>
            <span>Live input</span>
            <strong>{audio.peak_dbfs.toFixed(1)} dBFS</strong>
          </div>
          <Pill tone={levelTone}>{audio.clip ? "Clipping" : `${audio.peak_pct.toFixed(0)}% peak`}</Pill>
        </div>
        <div class="visualizer" aria-hidden="true">
          {visualBars.map((height, index) => (
            <i key={index} style={{ height: `${height}%` }} />
          ))}
        </div>
        <div class="meter" aria-label="Live input level">
          <div class={`meter-fill ${levelTone}`} style={{ width: `${levelPct}%` }} />
        </div>
        <p>
          {audio.i2s_driver_ok
            ? `${audio.peak_pct.toFixed(0)}% peak, ${audio.audio_pipeline_load_pct.toFixed(1)}% load, ${audio.i2s_fallback_blocks} fallback blocks`
            : `I2S driver error ${audio.i2s_last_error}`}
        </p>
        {!audio.i2s_link_ok && <p class="warn-text">{audio.i2s_hint}</p>}
      </section>

      <SpectrogramPanel sampleRate={audio.sample_rate} streaming={status.streaming} />

      <section class="grid">
        <article class="panel network-panel">
          <div class="panel-title">
            <h2>Network</h2>
            <Pill tone={wifiTone}>{status.ip || "Waiting"}</Pill>
          </div>
          <Stat label="IP address" value={status.ip || "Waiting"} />
          <Stat label="RTSP URL" value={<a href={rtspUrl}>{rtspUrl}</a>} />
          <Stat label="WiFi RSSI" value={`${status.wifi_rssi} dBm`} />
          <Stat label="Uptime" value={status.uptime || "Starting"} />
          <Setting label="WiFi TX">
            <select class={controlClass("wifi_tx")} value={input("wifi_tx", "19.5")} onInput={(event) => editDraft("wifi_tx", event.currentTarget.value)}>
              {["-1.0", "2.0", "5.0", "7.0", "8.5", "11.0", "13.0", "15.0", "17.0", "18.5", "19.0", "19.5"].map((v) => <option key={v}>{v}</option>)}
            </select>
            <span class="unit">dBm</span>
            {setButton("wifi_tx")}
          </Setting>
        </article>

        <article class="panel">
          <div class="panel-title">
            <h2>Streams</h2>
            <div class="client-slots" aria-label="RTSP client slots">
              {clientSlots.map((active, index) => <i key={index} class={active ? "active" : ""} />)}
            </div>
          </div>
          <Stat label="RTSP clients" value={`${status.active_rtsp_clients} playing, ${status.connected_rtsp_clients} connected / ${status.max_rtsp_clients}`} />
          <Stat label="Transport" value={status.rtsp_transport.toUpperCase()} />
          <Stat label="Clients" value={status.client || "Waiting"} />
          <Stat label="Packet rate" value={`${status.current_rate_pkt_s} pkt/s`} />
          <Stat label="Dropped packets" value={status.rtp_packets_dropped} />
          <div class="chips">
            <Pill>RTSP/RTP L16</Pill>
            <Pill>WebAudio PCM</Pill>
            <Pill>WAV chunk</Pill>
            <Pill>JSON PCM</Pill>
          </div>
        </article>

        <article class="panel settings-panel">
          <div class="panel-title">
            <h2>Audio</h2>
            <Pill tone={audio.i2s_driver_ok ? "ok" : "bad"}>{audio.i2s_driver_ok ? "I2S ready" : "I2S fault"}</Pill>
          </div>
          <Setting label="Sample rate" detail={`${audio.latency_ms.toFixed(1)} ms effective latency • PDM-safe choices`}>
            <select class={controlClass("rate")} value={input("rate", "48000")} onInput={(event) => editDraft("rate", event.currentTarget.value)}>
              {supportedSampleRates.map((rate) => (
                <option key={rate} value={rate}>{rate}</option>
              ))}
            </select>
            <span class="unit">Hz</span>
            {setButton("rate")}
          </Setting>
          <Setting label="Gain" detail={`Effective ${audio.effective_gain.toFixed(2)}x`}>
            <input class={controlClass("gain")} type="number" min="0.1" max="100" step="0.1" value={input("gain", "1.0")} onInput={(event) => editDraft("gain", event.currentTarget.value)} />
            <span class="unit">x</span>
            {setButton("gain")}
          </Setting>
          <Setting label="Buffer" detail={`${audio.buffer_size} configured, RTP chunk is capped internally`}>
            <select class={controlClass("buffer")} value={input("buffer", "1024")} onInput={(event) => editDraft("buffer", event.currentTarget.value)}>
              {["256", "512", "1024", "2048", "4096", "8192"].map((v) => <option key={v}>{v}</option>)}
            </select>
            <span class="unit">samples</span>
            {setButton("buffer")}
          </Setting>
          <Setting label="High-pass" detail={`${audio.hp_cutoff_hz} Hz cutoff • start low`}>
            <select class={controlClass("hp_enable")} value={input("hp_enable", "on")} onInput={(event) => editDraft("hp_enable", event.currentTarget.value)}>
              <option value="on">ON</option>
              <option value="off">OFF</option>
            </select>
            {setButton("hp_enable")}
          </Setting>
          <Setting label="HPF cutoff">
            <input class={controlClass("hp_cutoff")} type="number" min="10" max="10000" step="10" value={input("hp_cutoff", "180")} onInput={(event) => editDraft("hp_cutoff", event.currentTarget.value)} />
            <span class="unit">Hz</span>
            {setButton("hp_cutoff")}
          </Setting>
          <Setting label="AGC" detail={audio.agc_enable ? `${audio.agc_multiplier.toFixed(2)}x multiplier` : "Manual level"}>
            <select class={controlClass("agc_enable")} value={input("agc_enable", "off")} onInput={(event) => editDraft("agc_enable", event.currentTarget.value)}>
              <option value="on">ON</option>
              <option value="off">OFF</option>
            </select>
            {setButton("agc_enable")}
          </Setting>
          <Setting label="Noise filter" detail={audio.noise_filter_enable ? `${audio.noise_reduction_db.toFixed(1)} dB reduction` : "Bypassed"}>
            <select class={controlClass("noise_filter")} value={input("noise_filter", "off")} onInput={(event) => editDraft("noise_filter", event.currentTarget.value)}>
              <option value="on">ON</option>
              <option value="off">OFF</option>
            </select>
            {setButton("noise_filter")}
          </Setting>
          <Setting label="LED mode" detail={`PDM shift is fixed at ${audio.i2s_shift} bits`}>
            <select class={controlClass("led_mode")} value={input("led_mode", "1")} onInput={(event) => editDraft("led_mode", event.currentTarget.value)}>
              <option value="0">Off</option>
              <option value="1">Static</option>
              <option value="2">Level</option>
            </select>
            {setButton("led_mode")}
          </Setting>
        </article>

        <article class="panel settings-panel">
          <div class="panel-title">
            <h2>Reliability</h2>
            <Pill tone={perf.auto_recovery ? "ok" : "neutral"}>{perf.auto_recovery ? "Recovery on" : "Manual"}</Pill>
          </div>
          <Setting label="Auto recovery">
            <select class={controlClass("auto_recovery")} value={input("auto_recovery", "off")} onInput={(event) => editDraft("auto_recovery", event.currentTarget.value)}>
              <option value="on">ON</option>
              <option value="off">OFF</option>
            </select>
            {setButton("auto_recovery")}
          </Setting>
          <Setting label="Threshold mode" detail={`Recommended ${perf.recommended_min_rate} pkt/s`}>
            <select class={controlClass("thr_mode")} value={input("thr_mode", "auto")} onInput={(event) => editDraft("thr_mode", event.currentTarget.value)}>
              <option value="auto">Auto</option>
              <option value="manual">Manual</option>
            </select>
            {setButton("thr_mode")}
          </Setting>
          <Setting label="Restart threshold">
            <input class={controlClass("min_rate")} type="number" min="5" max="200" step="1" value={input("min_rate", "5")} disabled={perf.auto_threshold} onInput={(event) => editDraft("min_rate", event.currentTarget.value)} />
            <span class="unit">pkt/s</span>
            {setButton("min_rate")}
          </Setting>
          <Setting label="Check interval">
            <input class={controlClass("check_interval")} type="number" min="1" max="60" step="1" value={input("check_interval", "15")} onInput={(event) => editDraft("check_interval", event.currentTarget.value)} />
            <span class="unit">min</span>
            {setButton("check_interval")}
          </Setting>
          <Setting label="Scheduled reset">
            <select class={controlClass("sched_reset")} value={input("sched_reset", "off")} onInput={(event) => editDraft("sched_reset", event.currentTarget.value)}>
              <option value="on">ON</option>
              <option value="off">OFF</option>
            </select>
            {setButton("sched_reset")}
          </Setting>
          <Setting label="Reset after">
            <input class={controlClass("reset_hours")} type="number" min="1" max="168" step="1" value={input("reset_hours", "24")} onInput={(event) => editDraft("reset_hours", event.currentTarget.value)} />
            <span class="unit">h</span>
            {setButton("reset_hours")}
          </Setting>
        </article>

        <article class="panel settings-panel">
          <div class="panel-title">
            <h2>Thermal</h2>
            <Pill tone={tempTone}>{thermal.sensor_fault ? "Sensor fault" : thermal.latched_persist ? "Latched" : "Ready"}</Pill>
          </div>
          <Stat label="Current" value={thermal.current_valid && thermal.current_c !== null ? `${thermal.current_c.toFixed(1)} C` : "N/A"} />
          <Stat label="Peak" value={`${thermal.max_c.toFixed(1)} C`} />
          <Setting label="Protection">
            <select class={controlClass("oh_enable")} value={input("oh_enable", "on")} onInput={(event) => editDraft("oh_enable", event.currentTarget.value)}>
              <option value="on">ON</option>
              <option value="off">OFF</option>
            </select>
            {setButton("oh_enable")}
          </Setting>
          <Setting label="Shutdown limit">
            <select class={controlClass("oh_limit")} value={input("oh_limit", "80")} onInput={(event) => editDraft("oh_limit", event.currentTarget.value)}>
              {["30", "35", "40", "45", "50", "55", "60", "65", "70", "75", "80", "85", "90", "95"].map((v) => <option key={v}>{v}</option>)}
            </select>
            <span class="unit">C</span>
            {setButton("oh_limit")}
          </Setting>
          <Setting label="CPU clock">
            <select class={controlClass("cpu_freq")} value={input("cpu_freq", "160")} onInput={(event) => editDraft("cpu_freq", event.currentTarget.value)}>
              {["80", "120", "160", "240"].map((v) => <option key={v}>{v}</option>)}
            </select>
            <span class="unit">MHz</span>
            {setButton("cpu_freq")}
          </Setting>
          {thermal.last_reason && <p class="quiet">{thermal.last_reason}</p>}
          {thermal.latched_persist && <button onClick={clearThermalLatch}>Acknowledge & re-enable RTSP</button>}
        </article>

        <article class="panel control-panel">
          <div class="panel-title">
            <h2>Controls</h2>
            <Pill>{status.uptime || "Starting"}</Pill>
          </div>
          <div class="actions">
            <button onClick={() => runAction("reboot")}>Reboot</button>
            <button onClick={() => runAction("factory_reset")}>Defaults</button>
          </div>
          <p class="quiet">Two RTSP clients can play at once. Browser preview formats stay on the diagnostics ring buffer.</p>
          <div class="links">
            <a href="/streamer" target="_blank" rel="noreferrer">Browser PCM</a>
            <a href="/api/web_audio_wav" target="_blank" rel="noreferrer">WAV Chunk</a>
            <a href="/api/stream_options" target="_blank" rel="noreferrer">Stream API</a>
          </div>
        </article>
      </section>
    </main>
  );
}

render(<App />, document.getElementById("app")!);
