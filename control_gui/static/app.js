(() => {
  const $ = (id) => document.getElementById(id);

  const portSelect = $("portSelect");
  const connectBtn = $("connectBtn");
  const refreshBtn = $("refreshBtn");
  const linkDot = $("linkDot");
  const linkLabel = $("linkLabel");
  const linkDetail = $("linkDetail");

  const baseSlider = $("baseSlider");
  const heightSlider = $("heightSlider");
  const gripSlider = $("gripSlider");
  const speedSlider = $("speedSlider");

  const COLOR_HEX = {
    NONE: "#9aa89d",
    RED: "#e53935",
    YELLOW: "#f4d03f",
    GREEN: "#43a047",
  };

  let connected = false;
  let armDirty = false;
  let holdDir = null;
  let pollTimer = null;

  async function api(path, opts) {
    const res = await fetch(path, {
      headers: { "Content-Type": "application/json" },
      ...opts,
    });
    return res.json();
  }

  async function refreshPorts() {
    const data = await api("/api/ports");
    const cur = portSelect.value;
    portSelect.innerHTML = "";
    for (const p of data.ports || []) {
      const opt = document.createElement("option");
      opt.value = p.device;
      opt.textContent = p.description ? `${p.device} — ${p.description}` : p.device;
      portSelect.appendChild(opt);
    }
    if ([...portSelect.options].some((o) => o.value === cur)) portSelect.value = cur;
    else if ([...portSelect.options].some((o) => o.value.toUpperCase() === "COM3")) {
      portSelect.value = "COM3";
    }
  }

  function setEnabled(on) {
    connected = on;
    [baseSlider, heightSlider, gripSlider, $("armSendBtn"), $("centerBtn"), $("demoBtn"), $("calBtn")].forEach(
      (el) => {
        el.disabled = !on;
      }
    );
    connectBtn.textContent = on ? "Disconnect" : "Connect";
  }

  function applyFlags(flags) {
    document.querySelectorAll(".flag").forEach((el) => {
      const k = el.dataset.k;
      el.classList.toggle("on", !!(flags && flags[k]));
    });
  }

  function renderState(s) {
    if (!s) return;
    const live = !!s.connected;
    setEnabled(live);
    linkDot.dataset.state = !live ? "off" : s.stale ? "stale" : "on";
    linkLabel.textContent = !live ? "Disconnected" : s.stale ? "Stale telemetry" : "Live";
    linkDetail.textContent = live
      ? `${s.port || "hub"} · seq ${s.seq}${s.error ? " · " + s.error : ""}`
      : "Select the Nano USB port (usually COM3) · 500000 baud";

    $("leftVal").textContent = s.left ?? 0;
    $("rightVal").textContent = s.right ?? 0;
    $("encVal").textContent = `${s.enc_l ?? 0} / ${s.enc_r ?? 0}`;

    // Never overwrite sliders from telem — that was snapping everything back to 90°.
    // Labels show: setpoint (slider) → robot feedback
    $("baseVal").textContent = `${baseSlider.value}° → ${s.base ?? "—"}°`;
    $("heightVal").textContent = `${heightSlider.value}° → ${s.height ?? "—"}°`;
    $("gripVal").textContent = `${gripSlider.value}° → ${s.grip ?? "—"}°`;

    const color = s.color || "NONE";
    $("colorName").textContent = color;
    $("colorConf").textContent = `conf ${s.conf ?? 0}`;
    $("rgbUs").textContent = `R${s.r_us ?? 0} G${s.g_us ?? 0} B${s.b_us ?? 0} C${s.c_us ?? 0}`;
    $("swatch").style.background = COLOR_HEX[color] || COLOR_HEX.NONE;

    $("distVal").textContent = `${s.distance_mm ?? 0} mm`;
    const yaw = ((s.yaw_cdeg || 0) / 100).toFixed(1);
    const pitch = ((s.pitch_cdeg || 0) / 100).toFixed(1);
    const roll = ((s.roll_cdeg || 0) / 100).toFixed(1);
    $("yprVal").textContent = `${yaw}° / ${pitch}° / ${roll}°`;

    applyFlags(s.flags || {});
    $("seqLine").textContent = `seq ${s.seq ?? "—"} · t ${s.t_ms ?? "—"} ms`;

    const list = $("logList");
    list.innerHTML = "";
    for (const row of s.logs || []) {
      const li = document.createElement("li");
      li.innerHTML = `<span class="lvl">L${row.level}</span>${row.text}`;
      list.appendChild(li);
    }
  }

  async function poll() {
    try {
      const s = await api("/api/state");
      renderState(s);
    } catch (_) {
      /* ignore transient */
    }
  }

  async function toggleConnect() {
    if (connected) {
      await api("/api/disconnect", { method: "POST", body: "{}" });
      armDirty = false;
      await poll();
      return;
    }
    const port = portSelect.value;
    if (!port) return;
    const res = await api("/api/connect", {
      method: "POST",
      body: JSON.stringify({ port, baud: 500000 }),
    });
    if (!res.ok) alert(res.error || "connect failed");
    await poll();
  }

  async function sendDrive(left, right) {
    if (!connected) return;
    await api("/api/drive", {
      method: "POST",
      body: JSON.stringify({ left, right }),
    });
  }

  function dirVectors(dir) {
    const sp = Number(speedSlider.value) || 160;
    switch (dir) {
      case "fwd":
        return [sp, sp];
      case "back":
        return [-sp, -sp];
      case "left":
        return [-sp, sp];
      case "right":
        return [sp, -sp];
      default:
        return [0, 0];
    }
  }

  async function sendArm() {
    if (!connected) return;
    armDirty = true;
    await api("/api/arm", {
      method: "POST",
      body: JSON.stringify({
        base: Number(baseSlider.value),
        height: Number(heightSlider.value),
        grip: Number(gripSlider.value),
        speed_dps: 45,
      }),
    });
  }

  async function action(name) {
    if (!connected && name !== "ping") return;
    await api("/api/action", {
      method: "POST",
      body: JSON.stringify({ name }),
    });
  }

  let armTimer = null;
  function bindSlider(slider, labelId) {
    slider.addEventListener("input", () => {
      armDirty = true;
      $(labelId).textContent = `${slider.value}°`;
      if (!connected) return;
      if (armTimer) clearTimeout(armTimer);
      armTimer = setTimeout(() => {
        sendArm();
      }, 120);
    });
  }

  bindSlider(baseSlider, "baseVal");
  bindSlider(heightSlider, "heightVal");
  bindSlider(gripSlider, "gripVal");

  speedSlider.addEventListener("input", () => {
    $("speedOut").textContent = speedSlider.value;
  });

  let driveTimer = null;
  function startHoldDrive(dir) {
    holdDir = dir;
    const push = () => {
      if (!holdDir) return;
      const [l, r] = dirVectors(holdDir);
      sendDrive(l, r);
    };
    push();
    if (driveTimer) clearInterval(driveTimer);
    driveTimer = setInterval(push, 80);
  }
  function stopHoldDrive() {
    if (driveTimer) {
      clearInterval(driveTimer);
      driveTimer = null;
    }
    if (holdDir) {
      holdDir = null;
      sendDrive(0, 0);
    }
  }
  $("drivePad").addEventListener("pointerdown", (ev) => {
    const btn = ev.target.closest("[data-dir]");
    if (!btn) return;
    ev.preventDefault();
    if (btn.dataset.dir === "stop") {
      stopHoldDrive();
      sendDrive(0, 0);
      return;
    }
    startHoldDrive(btn.dataset.dir);
  });
  window.addEventListener("pointerup", stopHoldDrive);
  window.addEventListener("pointercancel", stopHoldDrive);

  const keyMap = {
    KeyW: "fwd",
    ArrowUp: "fwd",
    KeyS: "back",
    ArrowDown: "back",
    KeyA: "left",
    ArrowLeft: "left",
    KeyD: "right",
    ArrowRight: "right",
    Space: "stop",
  };
  const held = new Set();
  window.addEventListener("keydown", (ev) => {
    if (ev.repeat) return;
    const dir = keyMap[ev.code];
    if (!dir || !connected) return;
    ev.preventDefault();
    if (dir === "stop") {
      held.clear();
      sendDrive(0, 0);
      return;
    }
    held.add(dir);
    const [l, r] = dirVectors(dir);
    sendDrive(l, r);
  });
  window.addEventListener("keyup", (ev) => {
    const dir = keyMap[ev.code];
    if (!dir) return;
    held.delete(dir);
    if (!held.size) sendDrive(0, 0);
  });

  connectBtn.addEventListener("click", toggleConnect);
  refreshBtn.addEventListener("click", refreshPorts);
  $("armSendBtn").addEventListener("click", sendArm);
  $("centerBtn").addEventListener("click", () => action("center"));
  $("demoBtn").addEventListener("click", () => action("demo"));
  $("calBtn").addEventListener("click", () => action("cal"));
  $("pingBtn").addEventListener("click", () => action("ping"));
  $("eStopBtn").addEventListener("click", () => {
    held.clear();
    holdDir = null;
    sendDrive(0, 0);
    action("stop");
  });

  refreshPorts().then(() => {
    poll();
    pollTimer = setInterval(poll, 200);
  });
})();
