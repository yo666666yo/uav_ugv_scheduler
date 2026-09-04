/* 数字孪生城市指挥中心 · 前端逻辑
 *
 * 数据来源按优先级：
 *   1. master_agent (FastAPI)  GET /api/agents, /api/missions, WS /ws/events, POST /api/chat
 *   2. DashboardBridge 快照     telemetry.json（ZRDDS C++ 沙盘写出）
 *   3. 都不可用时进入 DEMO 模式，顶栏 DEMO 标记常亮
 *
 * 城市级指标（VEHICLES / PEDESTRIANS / AQI / ALERTS）与 CPU/GPU/MEM/NET
 * 本仓库没有数据源，为演示态推演值；FPS 与 LINK 为实测值。
 */

const CONFIG = Object.assign({
  masterBase: `${location.protocol}//${location.hostname}:8100`,
  telemetryUrl: "telemetry.json",
  uavStreamUrl: "",              // 例：http://192.168.1.50:8080/?action=stream
  ugvStreamUrl: "",
  pollMs: 1000,
  fleetRoster: { UAV: 4, UGV: 4 },                   // 编队席位：未上报的席位显示为离线

  carlaExtent: { x: [-260, 260], y: [-260, 260] },   // CARLA 坐标 -> 鸟瞰图
  poseExtent: { x: [0, 8], y: [0, 8] },              // uwb_map 米 -> 鸟瞰图
  sandbox: { w: 640, h: 440 },                       // telemetry.json 沙盘像素
}, window.URBAN_CONFIG || {});

const $ = (id) => document.getElementById(id);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
const pad2 = (n) => String(n).padStart(2, "0");
const fmt = (n, d = 0) => Number(n).toFixed(d);
const group = (n) => Math.round(n).toLocaleString("en-US");

const STATE_TEXT = {
  OFFLINE: "离线", IDLE: "待命", RESERVED: "已预约", PREFLIGHT: "起飞前检查",
  EXECUTING: "执行中", RETURNING: "返航", MANUAL_CONTROL: "人工接管", ERROR: "故障",
  COMPLETED: "已完成", ASSIGNED: "已分配", ENROUTE: "前往中", UNKNOWN: "未知",
};

const state = {
  source: "none",          // master | telemetry | demo
  agents: [],
  events: [],
  linkMs: null,
  selected: null,
  marks: [],               // 手动标记
  sim: null,               // 运行中的场景
  layers: { vehicles: true, pedestrians: false, signals: true, heatmap: false, pixelstream: true },
  city: { vehicles: 802, peds: 30000, aqi: 35, alerts: 1 },
  sys: { cpu: 52, gpu: 34, mem: 35, net: 10 },
  fps: 50,
  showEvents: false,
};

/* ------------------------------------------------------------------ 工具 */

function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>'"]/g, (c) =>
    ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", "'": "&#39;", '"': "&quot;" }[c]));
}

function project(value, [lo, hi]) {
  return clamp(((value - lo) / (hi - lo)) * 100, 0, 100);
}

function log(text, kind = "") {
  const line = document.createElement("div");
  line.className = `cmd-line${kind ? " is-" + kind : ""}`;
  const now = new Date();
  line.innerHTML = `<time>${pad2(now.getHours())}:${pad2(now.getMinutes())}:${pad2(now.getSeconds())}</time><span>${escapeHtml(text)}</span>`;
  const box = $("cmd-log");
  box.prepend(line);
  while (box.children.length > 40) box.lastElementChild.remove();
}

/* ------------------------------------------------------------- 智能体归一化 */

function labelFor(kind, indexByKind) {
  return `${kind}-${pad2(indexByKind)}`;
}

function statusOf(agent) {
  if (!agent.online || agent.state === "OFFLINE") return "error";
  if (agent.state === "ERROR" || agent.state === "MANUAL_CONTROL") return "error";
  if (agent.battery !== null && agent.battery < 25) return "warn";
  if (agent.state === "EXECUTING" || agent.state === "RETURNING") return "live";
  return "";
}

function finalize(list) {
  const ordered = [...list.filter((a) => a.kind === "UAV"), ...list.filter((a) => a.kind === "UGV")];
  const seen = { UAV: 0, UGV: 0 };
  return ordered.map((a) => {
    seen[a.kind] += 1;
    a.label = labelFor(a.kind, seen[a.kind]);
    a.status = a.placeholder ? "idle" : statusOf(a);
    return a;
  });
}

/** 上报的设备先占席位，剩余席位补成离线格，编队规模始终等于 CONFIG.fleetRoster。 */
function padRoster(list) {
  const out = [...list];
  for (const kind of ["UAV", "UGV"]) {
    const have = out.filter((a) => a.kind === kind).length;
    for (let i = have; i < (CONFIG.fleetRoster[kind] || 0); i += 1) {
      out.push({
        id: `${kind.toLowerCase()}-slot-${i + 1}`, kind,
        online: false, state: "OFFLINE", battery: null, progress: null,
        taskId: "", pos: null, frame: "none", placeholder: true,
      });
    }
  }
  return finalize(out);
}

function fromMasterAgents(payload) {
  return padRoster((payload.agents || []).map((a) => ({
    id: a.agent_id,
    kind: a.agent_type === "UGV" ? "UGV" : "UAV",
    online: Boolean(a.device_online),
    state: a.work_state || "UNKNOWN",
    battery: (a.battery || {}).remaining_percent ?? null,
    progress: null,
    taskId: a.current_task_id || "",
    pos: a.pose ? { x: a.pose.x_m, y: a.pose.y_m, z: a.pose.z_m, yaw: a.pose.yaw_rad } : null,
    frame: "pose",
  })));
}

function fromTelemetry(payload) {
  const tasks = payload.tasks || [];
  return padRoster((payload.vehicles || []).map((v) => {
    const task = tasks.find((t) => t.id === v.task_id);
    return {
      id: v.id,
      kind: v.kind === "UGV" ? "UGV" : "UAV",
      online: true,
      state: v.phase || "UNKNOWN",
      battery: v.battery_percent ?? null,
      progress: v.progress_percent ?? null,
      taskId: v.task_id || "",
      pos: v.enu ? { x: v.enu.east, y: v.enu.north, z: v.enu.up, yaw: null } : null,
      sandbox: task && task.sandbox ? task.sandbox : null,
      frame: "sandbox",
    };
  }));
}

function demoAgents(tick) {
  const spec = [
    ["UAV", "EXECUTING", 92], ["UAV", "RETURNING", 61], ["UAV", "IDLE", 78], ["UAV", "IDLE", 44],
    ["UGV", "EXECUTING", 83], ["UGV", "IDLE", 70], ["UGV", "IDLE", 55], ["UGV", "ERROR", 18],
  ];
  return padRoster(spec.map(([kind, st, bat], i) => {
    const phase = tick / 900 + i * 0.7;
    return {
      id: `${kind.toLowerCase()}-${pad2(i + 1)}`,
      kind,
      online: st !== "ERROR",
      state: st,
      battery: clamp(bat + Math.sin(phase) * 2, 0, 100),
      progress: null,
      taskId: st === "EXECUTING" ? `demo-${pad2(i + 1)}` : "",
      pos: { x: 1 + ((i * 1.7) % 6) + Math.sin(phase) * 0.5, y: 1 + ((i * 2.3) % 6) + Math.cos(phase) * 0.5, z: kind === "UAV" ? 80 : 0, yaw: null },
      frame: "pose",
    };
  }));
}

/* ------------------------------------------------------------------ 渲染 */

function iconFor(kind) {
  return kind === "UGV" ? "#i-rover" : "#i-drone";
}

function renderFleet() {
  const html = state.agents.map((a) => {
    const battery = a.battery === null ? 0 : clamp(a.battery, 0, 100);
    const barClass = a.status === "error" ? " class=\"is-error\"" : a.status === "warn" ? " class=\"is-warn\"" : "";
    const dot = a.status ? `<span class="dot is-${a.status}"></span>` : "";
    const selected = state.selected === a.id ? " is-selected" : "";
    const offline = a.placeholder ? " is-offline" : "";
    const title = a.placeholder
      ? `${a.label} · 席位空闲，无设备上报`
      : `${a.id} · ${STATE_TEXT[a.state] || a.state}${a.battery === null ? "" : ` · 电量 ${fmt(a.battery, 0)}%`}`;
    return `<button class="fleet-item${selected}${offline}" data-agent="${escapeHtml(a.id)}" title="${escapeHtml(title)}">
      <span class="fleet-icon"><svg aria-hidden="true"><use href="${iconFor(a.kind)}"/></svg></span>
      <span class="fleet-main">
        <span class="fleet-name">${escapeHtml(a.label)}${dot}</span>
        <span class="bar"><i${barClass} style="width:${battery}%"></i></span>
      </span>
    </button>`;
  }).join("");
  $("fleet").innerHTML = html;
  $("stat-agents").textContent = state.agents.length || "--";
}

function markPosition(agent) {
  if (agent.frame === "sandbox" && agent.sandbox) {
    return { left: (agent.sandbox.x / CONFIG.sandbox.w) * 100, top: (agent.sandbox.y / CONFIG.sandbox.h) * 100 };
  }
  if (!agent.pos) return null;
  return { left: project(agent.pos.x, CONFIG.poseExtent.x), top: 100 - project(agent.pos.y, CONFIG.poseExtent.y) };
}

function renderMap() {
  const items = [];

  if (state.layers.vehicles) {
    for (const agent of state.agents) {
      const at = markPosition(agent);
      if (!at) continue;
      const live = agent.state === "EXECUTING" || agent.state === "RETURNING" ? " is-active" : "";
      items.push(`<div class="mark" style="left:${at.left}%;top:${at.top}%">
        <span class="mark-agent ${agent.kind.toLowerCase()}${live}"><svg aria-hidden="true"><use href="${iconFor(agent.kind)}"/></svg></span>
        <span class="mark-label">${escapeHtml(agent.label)}</span>
      </div>`);
    }
  }

  for (const mark of state.marks) {
    items.push(`<div class="mark" style="left:${mark.left}%;top:${mark.top}%">
      <span class="${mark.fire ? "mark-fire" : "mark-target"}"></span>
      <span class="mark-label">${escapeHtml(mark.label)}</span>
    </div>`);
  }

  $("map-overlay").innerHTML = items.join("");
}

function renderFeedMetrics() {
  const uav = state.agents.find((a) => a.kind === "UAV" && a.state === "EXECUTING")
    || state.agents.find((a) => a.kind === "UAV");
  const ugv = state.agents.find((a) => a.kind === "UGV" && a.state === "EXECUTING")
    || state.agents.find((a) => a.kind === "UGV");

  const link = state.linkMs === null ? "--" : fmt(state.linkMs, 0);
  $("uav-link").textContent = link;
  $("ugv-link").textContent = link;

  if (uav) {
    $("uav-alt").textContent = uav.pos ? fmt(Math.max(0, uav.pos.z), 1) : "--";
    $("uav-spd").textContent = fmt(uav.state === "EXECUTING" ? 6.4 : 0.5, 1);
    $("uav-bat").textContent = uav.battery === null ? "--" : fmt(uav.battery, 0);
  }
  if (ugv) {
    $("ugv-spd").textContent = fmt(ugv.state === "EXECUTING" ? 3.2 : 0.6, 1);
    $("ugv-hdg").textContent = fmt(ugv.pos && ugv.pos.yaw !== null && ugv.pos.yaw !== undefined
      ? ((ugv.pos.yaw * 180) / Math.PI + 360) % 360
      : 333, 0);
    const obstacle = ugv.state === "ERROR" || ugv.state === "MANUAL_CONTROL";
    const cell = $("ugv-obs");
    cell.textContent = obstacle ? "阻塞" : "安全";
    cell.className = `metric-value ${obstacle ? "is-alert" : "is-safe"}`;
  }
}

function renderCity() {
  $("city-vehicles").textContent = group(state.city.vehicles);
  $("city-peds").textContent = group(state.city.peds);
  $("city-aqi").textContent = group(state.city.aqi);
  $("city-alerts").textContent = group(state.city.alerts);
}

function renderSys() {
  for (const key of ["cpu", "gpu", "mem", "net"]) {
    const value = Math.round(state.sys[key]);
    $(`sys-${key}`).textContent = `${value}%`;
    $(`sys-${key}-bar`).style.width = `${value}%`;
  }
  $("sys-fps").textContent = `${Math.round(state.fps)} fps`;
  $("stat-fps").textContent = Math.round(state.fps);
}

function setLink(mode) {
  state.source = mode;
  const dot = $("link-dot");
  const text = $("link-text");
  const pill = $("mode-pill");
  if (mode === "master") {
    dot.className = "dot is-live"; text.textContent = "Master Agent"; pill.hidden = true;
  } else if (mode === "telemetry") {
    dot.className = "dot is-live"; text.textContent = "ZRDDS Bridge"; pill.hidden = true;
  } else if (mode === "connecting") {
    dot.className = "dot is-warn"; text.textContent = "Connecting"; pill.hidden = false;
  } else {
    dot.className = "dot is-error"; text.textContent = "Offline"; pill.hidden = false;
  }
  $("agent-dot").className = `dot ${mode === "master" ? "is-live" : "is-warn"}`;
}

/* ------------------------------------------------------------------ 取数 */

async function timedFetch(url, options) {
  const started = performance.now();
  const response = await fetch(url, Object.assign({ cache: "no-store" }, options));
  state.linkMs = performance.now() - started;
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  return response.json();
}

async function poll() {
  try {
    const payload = await timedFetch(`${CONFIG.masterBase}/api/agents`);
    if ((payload.agents || []).length) {
      state.agents = fromMasterAgents(payload);
      setLink("master");
      afterData();
      return;
    }
  } catch (_) { /* 落到下一个数据源 */ }

  try {
    const payload = await timedFetch(`${CONFIG.telemetryUrl}?ts=${Date.now()}`);
    state.agents = fromTelemetry(payload);
    state.events = payload.events || [];
    if (state.showEvents) pushLatestEvent();
    setLink("telemetry");
    afterData();
    return;
  } catch (_) { /* 进入演示态 */ }

  state.linkMs = null;
  if (state.source !== "demo") {
    setLink("demo");
    log("未发现总 Agent 与 ZRDDS 遥测快照，切换到演示数据", "error");
  }
  state.source = "demo";
  state.agents = demoAgents(performance.now());
  afterData();
}

let lastEventId = "";
function pushLatestEvent() {
  const latest = [...state.events].sort((a, b) => b.occurred_at_ms - a.occurred_at_ms)[0];
  if (!latest || latest.id === lastEventId) return;
  lastEventId = latest.id;
  log(`${latest.source} · ${String(latest.code || latest.kind).replace(/_/g, " ")}`);
}

function afterData() {
  renderFleet();
  renderMap();
  renderFeedMetrics();
}

/* ------------------------------------------------------------ WebSocket */

function connectEvents() {
  let socket;
  try {
    socket = new WebSocket(`${CONFIG.masterBase.replace(/^http/, "ws")}/ws/events`);
  } catch (_) { return; }

  socket.addEventListener("message", (message) => {
    let payload;
    try { payload = JSON.parse(message.data); } catch (_) { return; }
    if (payload.event === "mission_created") {
      log(`任务已创建 ${payload.data.mission_id}`);
    } else if (payload.event === "task_feedback") {
      const d = payload.data || {};
      log(`${d.agent_id || "设备"} · ${STATE_TEXT[d.status] || d.status || "反馈"}${d.message ? " · " + d.message : ""}`);
    }
  });
  socket.addEventListener("close", () => window.setTimeout(connectEvents, 5000));
  socket.addEventListener("error", () => socket.close());
}

/* ------------------------------------------------------------ 演示态推演 */

function drift(value, lo, hi, amount) {
  return clamp(value + (Math.random() - 0.5) * amount, lo, hi);
}

function stepDemoNumbers() {
  state.city.vehicles = Math.round(drift(state.city.vehicles, 640, 980, 14));
  state.city.peds = Math.round(drift(state.city.peds, 26000, 34000, 400));
  state.city.aqi = Math.round(drift(state.city.aqi, 22, 68, 2));
  state.sys.cpu = drift(state.sys.cpu, 28, 84, 5);
  state.sys.gpu = drift(state.sys.gpu, 18, 76, 6);
  state.sys.mem = drift(state.sys.mem, 30, 62, 2);
  state.sys.net = drift(state.sys.net, 6, 42, 3);
  renderCity();
  renderSys();
}

/* --------------------------------------------------------------- 动画帧 */

const route = $("route-runner");
const ego = $("ego-car");
const routeLength = route && route.getTotalLength ? route.getTotalLength() : 0;

let frames = 0;
let fpsMark = performance.now();

function tick(now) {
  frames += 1;
  if (now - fpsMark >= 1000) {
    // 页面不可见时浏览器会暂停 rAF，此时保留上一次读数而不是显示 0
    if (frames > 0 && !document.hidden) state.fps = (frames * 1000) / (now - fpsMark);
    frames = 0;
    fpsMark = now;
  }

  if (routeLength && state.layers.signals) {
    const offset = (now / 22) % routeLength;
    route.setAttribute("stroke-dashoffset", String(-offset));
    if (ego && state.layers.vehicles) {
      const point = route.getPointAtLength(offset % routeLength);
      const ahead = route.getPointAtLength((offset + 12) % routeLength);
      const angle = (Math.atan2(ahead.y - point.y, ahead.x - point.x) * 180) / Math.PI;
      ego.setAttribute("transform", `translate(${point.x} ${point.y}) rotate(${angle})`);
    }
  }

  requestAnimationFrame(tick);
}

/* ------------------------------------------------------------------ 时钟 */

function stepClock() {
  const now = new Date();
  $("clock-time").textContent = `${pad2(now.getHours())}:${pad2(now.getMinutes())}:${pad2(now.getSeconds())}`;
  $("clock-date").textContent = `${now.getFullYear()}.${pad2(now.getMonth() + 1)}.${pad2(now.getDate())} UTC+8`;
}

/* ------------------------------------------------------------------ 视频 */

function setFeed(which, url) {
  const box = $(`feed-${which}`);
  const img = $(`feed-${which}-img`);
  if (!url) {
    box.classList.add("is-dark");
    img.removeAttribute("src");
    return;
  }
  box.classList.remove("is-dark");
  img.onerror = () => box.classList.add("is-dark");
  img.src = `${url}${url.includes("?") ? "&" : "?"}ts=${Date.now()}`;
}

/* ------------------------------------------------------------------ 场景 */

const SCENARIOS = {
  "sim-1": { name: "热成像勘测", kind: "UAV", alerts: 1 },
  "sim-2": { name: "医疗物资投送", kind: "UGV", alerts: 2 },
};

function startScenario(id) {
  const scenario = SCENARIOS[id];
  stopScenario();
  state.sim = id;
  $(id).setAttribute("aria-pressed", "true");
  state.city.alerts = scenario.alerts;
  renderCity();
  $("tool-state").textContent = `· ${scenario.name}`;
  log(`场景「${scenario.name}」开始推演，目标编队 ${scenario.kind}`);

  if (state.source === "demo") {
    for (const agent of state.agents) {
      if (agent.kind === scenario.kind) agent.state = "EXECUTING";
    }
    renderFleet();
    renderMap();
    renderFeedMetrics();
  }
}

function stopScenario() {
  if (state.sim) $(state.sim).setAttribute("aria-pressed", "false");
  state.sim = null;
}

function resetConsole() {
  stopScenario();
  state.marks = [];
  state.city.alerts = 1;
  $("map-viewport").style.transform = "";
  $("compass").querySelector("svg").style.transform = "";
  $("tool-state").textContent = "· 待命";
  $("cmd-log").innerHTML = "";
  renderCity();
  renderMap();
  log("控制台已复位");
}

/* ------------------------------------------------------------------ 交互 */

function bind() {
  // 折叠 / 全屏
  document.addEventListener("click", (event) => {
    const collapse = event.target.closest("[data-collapse]");
    if (collapse) {
      $(collapse.dataset.collapse).classList.toggle("is-collapsed");
      return;
    }
    const full = event.target.closest("[data-full]");
    if (full) {
      const panel = $(full.dataset.full);
      const on = panel.classList.toggle("is-full");
      full.setAttribute("aria-pressed", String(on));
      return;
    }
    const refeed = event.target.closest("[data-refeed]");
    if (refeed) {
      const which = refeed.dataset.refeed;
      const url = which === "uav" ? CONFIG.uavStreamUrl : CONFIG.ugvStreamUrl;
      setFeed(which, url);
      log(url ? `重新拉取 ${which.toUpperCase()} 实时流` : `${which.toUpperCase()} 实时流地址未配置（URBAN_CONFIG.${which}StreamUrl）`, url ? "" : "error");
      return;
    }
    const agentBtn = event.target.closest("[data-agent]");
    if (agentBtn) {
      state.selected = state.selected === agentBtn.dataset.agent ? null : agentBtn.dataset.agent;
      renderFleet();
      return;
    }
  });

  document.addEventListener("keydown", (event) => {
    if (event.key !== "Escape") return;
    document.querySelectorAll(".panel.is-full").forEach((panel) => {
      panel.classList.remove("is-full");
      const button = document.querySelector(`[data-full="${panel.id}"]`);
      if (button) button.setAttribute("aria-pressed", "false");
    });
  });

  // 视图分段
  document.querySelectorAll(".seg button").forEach((button) => {
    button.addEventListener("click", () => {
      button.parentElement.querySelectorAll("button").forEach((b) => b.setAttribute("aria-selected", "false"));
      button.setAttribute("aria-selected", "true");
      $("feed-uav").querySelector(".feed-label").textContent =
        button.dataset.view === "detect" ? "UAV Live Detection" : "UAV Patrol View";
    });
  });

  // 图层
  document.querySelectorAll(".toggle").forEach((button) => {
    button.addEventListener("click", () => {
      const layer = button.dataset.layer;
      const on = button.getAttribute("aria-pressed") !== "true";
      button.setAttribute("aria-pressed", String(on));
      state.layers[layer] = on;
      const node = $(`layer-${layer}`);
      if (node) node.style.opacity = on ? "1" : "0";
      if (layer === "vehicles" || layer === "signals") {
        const vehicles = $("layer-vehicles");
        if (vehicles) vehicles.style.opacity = state.layers.vehicles ? "1" : "0";
        renderMap();
      }
      if (layer === "pixelstream") $("stream-badge").hidden = !on;
    });
  });

  // 顶栏
  $("sim-1").addEventListener("click", () => (state.sim === "sim-1" ? (stopScenario(), log("场景已停止")) : startScenario("sim-1")));
  $("sim-2").addEventListener("click", () => (state.sim === "sim-2" ? (stopScenario(), log("场景已停止")) : startScenario("sim-2")));
  $("sim-reset").addEventListener("click", resetConsole);
  $("force-sync").addEventListener("click", () => { poll(); log("已请求刷新遥测快照"); });
  $("toggle-log").addEventListener("click", (event) => {
    state.showEvents = !state.showEvents;
    event.currentTarget.setAttribute("aria-pressed", String(state.showEvents));
    log(state.showEvents ? "DDS 审计事件已接入指令日志" : "DDS 审计事件已停止推送");
  });
  $("open-settings").addEventListener("click", (event) => {
    event.currentTarget.setAttribute("aria-pressed", "false");
    log(`链路：总 Agent ${CONFIG.masterBase} · 快照 ${CONFIG.telemetryUrl} · 当前来源 ${state.source}`);
  });

  // 鸟瞰图控件
  $("compass").addEventListener("click", () => {
    $("map-viewport").style.transform = "";
    log("鸟瞰视角已复位");
  });
  $("stream-badge").querySelector("button").addEventListener("click", () => {
    $("stream-badge").hidden = true;
    const toggle = document.querySelector('[data-layer="pixelstream"]');
    toggle.setAttribute("aria-pressed", "false");
    state.layers.pixelstream = false;
  });

  // 调度指令
  $("cmd-form").addEventListener("submit", async (event) => {
    event.preventDefault();
    const input = $("cmd-input");
    const message = input.value.trim();
    if (!message) return;
    const button = event.currentTarget.querySelector("button");
    button.disabled = true;
    log(`[${$("cmd-target").value} · ${$("cmd-priority").options[$("cmd-priority").selectedIndex].text}] ${message}`);
    input.value = "";
    try {
      const response = await fetch(`${CONFIG.masterBase}/api/chat`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ message }),
      });
      const payload = await response.json();
      if (payload.ok) log(payload.reply || `任务 ${payload.mission_id} 已下发`);
      else log(payload.error || "总 Agent 拒绝了该指令", "error");
    } catch (_) {
      log("总 Agent 未连接，指令仅记录未下发", "error");
    } finally {
      button.disabled = false;
    }
  });

  // 坐标工具
  $("tool-current").addEventListener("click", () => {
    const agent = state.agents.find((a) => a.id === state.selected) || state.agents[0];
    if (!agent || !agent.pos) { log("当前没有可读取位置的设备", "error"); return; }
    $("coord-x").value = fmt(agent.pos.x, 1);
    $("coord-y").value = fmt(agent.pos.y, 1);
    $("coord-z").value = fmt(agent.pos.z || 0, 1);
    $("coord-label").value = agent.label;
    log(`已读取 ${agent.label} 位置`);
  });

  $("tool-mark").addEventListener("click", () => addMark(false));
  $("tool-fire").addEventListener("click", () => addMark(true));

  $("tool-jump").addEventListener("click", () => {
    const at = coordToMap();
    const viewport = $("map-viewport");
    viewport.style.transform = `scale(1.85) translate(${(50 - at.left) * 0.55}%, ${(50 - at.top) * 0.55}%)`;
    $("tool-state").textContent = "· 已跳转";
    log(`视角跳转至 X ${$("coord-x").value} / Y ${$("coord-y").value}`);
  });
}

function coordToMap() {
  return {
    left: project(Number($("coord-x").value || 0), CONFIG.carlaExtent.x),
    top: 100 - project(Number($("coord-y").value || 0), CONFIG.carlaExtent.y),
  };
}

function addMark(isFire) {
  const at = coordToMap();
  const label = ($("coord-label").value || "TARGET").trim();
  state.marks.push({ left: at.left, top: at.top, label, fire: isFire });
  if (isFire) {
    state.city.alerts += 1;
    renderCity();
  }
  $("tool-state").textContent = isFire ? "· 火点已标记" : "· 已标记";
  renderMap();
  log(`${isFire ? "火点" : "标记"}「${label}」已落图`);
}

/* -------------------------------------------------------------------- 启动 */

setLink("connecting");
bind();
setFeed("uav", CONFIG.uavStreamUrl);
setFeed("ugv", CONFIG.ugvStreamUrl);
renderCity();
renderSys();
stepClock();
poll();
connectEvents();

window.setInterval(stepClock, 1000);
window.setInterval(poll, CONFIG.pollMs);
window.setInterval(stepDemoNumbers, 1600);
requestAnimationFrame(tick);
