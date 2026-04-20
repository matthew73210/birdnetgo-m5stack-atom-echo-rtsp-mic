import { render } from "preact";
import { useEffect, useMemo, useState } from "preact/hooks";
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
  agc_enable: boolean;
  agc_multiplier: number;
  noise_filter_enable: boolean;
  noise_reduction_db: number;
  audio_pipeline_load_pct: number;
  i2s_driver_ok: boolean;
  i2s_last_error: number;
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
  agc_enable: false,
  agc_multiplier: 1,
  noise_filter_enable: false,
  noise_reduction_db: 0,
  audio_pipeline_load_pct: 0,
  i2s_driver_ok: true,
  i2s_last_error: 0
};

async function getJson<T>(url: string): Promise<T> {
  const response = await fetch(url, { cache: "no-store" });
  if (!response.ok) throw new Error(`${response.status} ${response.statusText}`);
  return response.json() as Promise<T>;
}

async function action(name: string) {
  await getJson(`/api/action?name=${encodeURIComponent(name)}`);
}

function Pill({ children, tone = "neutral" }: { children: preact.ComponentChildren; tone?: "neutral" | "ok" | "warn" | "bad" }) {
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

function Metric({ label, value, detail, tone = "neutral" }: {
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

function App() {
  const [status, setStatus] = useState<Status>(emptyStatus);
  const [audio, setAudio] = useState<AudioStatus>(emptyAudio);
  const [error, setError] = useState("");

  const rtspUrl = useMemo(() => {
    const ip = status.ip || window.location.hostname;
    return `rtsp://${ip}:8554/audio`;
  }, [status.ip]);

  const levelPct = Math.max(0, Math.min(100, ((audio.peak_dbfs + 60) / 60) * 100));
  const levelTone = audio.clip ? "bad" : audio.peak_pct >= 90 ? "warn" : "ok";
  const wifiTone = status.wifi_rssi > -67 ? "ok" : status.wifi_rssi > -75 ? "warn" : "bad";
  const heapTone = status.free_heap_kb > 96 ? "ok" : status.free_heap_kb > 72 ? "warn" : "bad";
  const clientSlots = Array.from({ length: Math.max(status.max_rtsp_clients, 1) }, (_, index) => index < status.active_rtsp_clients);
  const visualBars = Array.from({ length: 22 }, (_, index) => {
    const wave = 0.42 + Math.abs(Math.sin((index + 1) * 0.74)) * 0.58;
    return Math.max(10, Math.min(100, levelPct * wave));
  });

  useEffect(() => {
    let alive = true;
    const load = async () => {
      try {
        const [nextStatus, nextAudio] = await Promise.all([
          getJson<Status>("/api/status"),
          getJson<AudioStatus>("/api/audio_status")
        ]);
        if (!alive) return;
        setStatus(nextStatus);
        setAudio(nextAudio);
        setError("");
      } catch (err) {
        if (alive) setError(err instanceof Error ? err.message : "Unable to load status");
      }
    };
    load();
    const timer = window.setInterval(load, 1000);
    return () => {
      alive = false;
      window.clearInterval(timer);
    };
  }, []);

  const runAction = async (name: string) => {
    try {
      await action(name);
      setError("");
    } catch (err) {
      setError(err instanceof Error ? err.message : "Action failed");
    }
  };

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
            ? audio.clip
              ? `Clipping detected, ${audio.clip_count} clipped blocks`
              : `${audio.peak_pct.toFixed(0)}% peak, ${audio.audio_pipeline_load_pct.toFixed(1)}% pipeline load`
            : `I2S driver error ${audio.i2s_last_error}`}
        </p>
      </section>

      <section class="grid">
        <article class="panel network-panel">
          <div class="panel-title">
            <h2>Network</h2>
            <Pill tone={wifiTone}>{status.ip || "Waiting"}</Pill>
          </div>
          <Stat label="IP address" value={status.ip || "Waiting"} />
          <Stat label="RTSP URL" value={<a href={rtspUrl}>{rtspUrl}</a>} />
          <Stat label="WiFi RSSI" value={`${status.wifi_rssi} dBm`} />
          <Stat label="WiFi TX" value={`${status.wifi_tx_dbm.toFixed(1)} dBm`} />
          <Stat label="Uptime" value={status.uptime || "Starting"} />
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

        <article class="panel">
          <div class="panel-title">
            <h2>Audio</h2>
            <Pill tone={audio.i2s_driver_ok ? "ok" : "bad"}>{audio.i2s_driver_ok ? "I2S ready" : "I2S fault"}</Pill>
          </div>
          <Stat label="Sample rate" value={`${audio.sample_rate} Hz`} />
          <Stat label="Gain" value={`${audio.gain.toFixed(2)}x`} />
          <Stat label="Buffer" value={`${audio.buffer_size} samples`} />
          <Stat label="Latency" value={`${audio.latency_ms.toFixed(1)} ms`} />
          <Stat label="PDM shift" value={`${audio.i2s_shift} bits fixed`} />
          <Stat label="Processing" value={`${audio.agc_enable ? `AGC ${audio.agc_multiplier.toFixed(1)}x` : "Manual"}${audio.noise_filter_enable ? `, noise ${audio.noise_reduction_db.toFixed(1)} dB` : ""}`} />
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
