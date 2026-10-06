(() => {
  const $ = (id) => document.getElementById(id);

  const els = {
    portSelect: $("portSelect"),
    connectBtn: $("connectBtn"),
    refreshBtn: $("refreshBtn"),
    linkDot: $("linkDot"),
    linkLabel: $("linkLabel"),
    linkDetail: $("linkDetail"),
    logLine: $("logLine"),
    centerBtn: $("centerBtn"),
    openBtn: $("openBtn"),
    closeBtn: $("closeBtn"),
    demoBtn: $("demoBtn"),
    baseSlider: $("baseSlider"),
    heightSlider: $("heightSlider"),
    gripSlider: $("gripSlider"),
    baseVal: $("baseVal"),
    heightVal: $("heightVal"),
    gripVal: $("gripVal"),
    distVal: $("distVal"),
    distUnit: $("distUnit"),
    tofStatus: $("tofStatus"),
    yawVal: $("yawVal"),
    pitchVal: $("pitchVal"),
    rollVal: $("rollVal"),
    imuStatus: $("imuStatus"),
  };

  const axes = [
    { key: "base", slider: els.baseSlider, label: els.baseVal },
    { key: "height", slider: els.heightSlider, label: els.heightVal },
    { key: "grip", slider: els.gripSlider, label: els.gripVal },
  ];

  let connected = false;
  const timers = {};

  function setSliderVisual(slider) {
    const min = Number(slider.min);
    const max = Number(slider.max);
    const val = Number(slider.value);
    const pct = ((val - min) / (max - min)) * 100;
    slider.style.setProperty("--pct", `${pct}%`);
  }

  function setControlsEnabled(on) {
    axes.forEach(({ slider }) => {
      slider.disabled = !on;
    });
    [els.centerBtn, els.openBtn, els.closeBtn, els.demoBtn].forEach((btn) => {
      btn.disabled = !on;
    });
  }

  function applyStatus(data) {
    connected = !!data.connected;
    els.linkDot.dataset.state = data.error ? "err" : connected ? "on" : "off";
    els.linkLabel.textContent = connected ? "Connected" : data.error ? "Error" : "Disconnected";
    els.linkDetail.textContent = connected
      ? data.port || "serial"
      : data.error || "Select the Nano serial port";
    els.connectBtn.textContent = connected ? "Disconnect" : "Connect";
    els.connectBtn.classList.toggle("on", connected);
    els.portSelect.disabled = connected;
    setControlsEnabled(connected);

    if (typeof data.base === "number") {
      els.baseSlider.value = data.base;
      els.baseVal.textContent = data.base;
      setSliderVisual(els.baseSlider);
    }
    if (typeof data.height === "number") {
      els.heightSlider.value = data.height;
      els.heightVal.textContent = data.height;
      setSliderVisual(els.heightSlider);
    }
    if (typeof data.grip === "number") {
      els.gripSlider.value = data.grip;
      els.gripVal.textContent = data.grip;
      setSliderVisual(els.gripSlider);
    }
    if (els.distVal) {
      if (data.tof_ok && data.distance_mm > 0) {
        els.distVal.textContent = String(data.distance_mm);
        els.tofStatus.textContent = "VL53L0X ok";
        els.tofStatus.dataset.state = "on";
      } else {
        els.distVal.textContent = "—";
        els.tofStatus.textContent = connected
          ? "VL53L0X missing / timeout — check SDA/SCL VIN GND"
          : "Sensor offline";
        els.tofStatus.dataset.state = connected ? "err" : "off";
      }
    }

    if (els.yawVal) {
      if (data.imu_ok) {
        const fmt = (v) => (typeof v === "number" ? v.toFixed(0) : "—");
        els.yawVal.textContent = `${fmt(data.yaw_deg)}°`;
        els.pitchVal.textContent = `${fmt(data.pitch_deg)}°`;
        els.rollVal.textContent = `${fmt(data.roll_deg)}°`;
        els.imuStatus.textContent = "BNO085 ok";
        els.imuStatus.dataset.state = "on";
      } else {
        els.yawVal.textContent = "—";
        els.pitchVal.textContent = "—";
        els.rollVal.textContent = "—";
        els.imuStatus.textContent = connected
          ? "BNO085 missing — check 0x4A/0x4B VIN GND SDA SCL"
          : "Sensor offline";
        els.imuStatus.dataset.state = connected ? "err" : "off";
      }
    }

    if (data.last_line) {
      els.logLine.textContent = data.last_line;
      const fail = /pca=FAIL/i.test(data.last_line);
      els.linkDot.dataset.state = fail ? "err" : els.linkDot.dataset.state;
      if (fail) {
        els.linkLabel.textContent = "PCA9685 missing";
        els.linkDetail.textContent = "Nano OK — check I2C wiring / VCC / GND";
      }
    }
  }

  async function api(path, options) {
    const res = await fetch(path, {
      headers: { "Content-Type": "application/json" },
      ...options,
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.error || res.statusText);
    return data;
  }

  async function refreshPorts() {
    const data = await api("/api/ports");
    const current = els.portSelect.value;
    els.portSelect.innerHTML = "";
    const ports = data.ports || [];
    if (!ports.length) {
      const opt = document.createElement("option");
      opt.value = "";
      opt.textContent = "No serial ports found";
      els.portSelect.appendChild(opt);
      return;
    }
    ports.forEach((p) => {
      const opt = document.createElement("option");
      opt.value = p.device;
      opt.textContent = p.description
        ? `${p.device} — ${p.description}`
        : p.device;
      els.portSelect.appendChild(opt);
    });
    const preferred =
      ports.find((p) => p.device.includes("ttyUSB")) ||
      ports.find((p) => p.device.includes("ttyACM")) ||
      ports[0];
    els.portSelect.value = ports.some((p) => p.device === current)
      ? current
      : preferred.device;
  }

  async function pollStatus() {
    try {
      const data = await api("/api/status");
      applyStatus(data);
    } catch (err) {
      els.logLine.textContent = String(err.message || err);
    }
  }

  async function toggleConnect() {
    try {
      if (connected) {
        await api("/api/disconnect", { method: "POST", body: "{}" });
      } else {
        const port = els.portSelect.value;
        if (!port) throw new Error("No port selected");
        els.connectBtn.disabled = true;
        els.linkDetail.textContent = "Opening serial (Nano may reset)…";
        await api("/api/connect", {
          method: "POST",
          body: JSON.stringify({ port, baud: 115200 }),
        });
      }
      await pollStatus();
    } catch (err) {
      els.logLine.textContent = String(err.message || err);
      els.linkDot.dataset.state = "err";
      els.linkLabel.textContent = "Error";
      els.linkDetail.textContent = String(err.message || err);
    } finally {
      els.connectBtn.disabled = false;
    }
  }

  function bindAxis({ key, slider, label }) {
    const push = async (value) => {
      try {
        await api("/api/set", {
          method: "POST",
          body: JSON.stringify({ axis: key, value }),
        });
      } catch (err) {
        els.logLine.textContent = String(err.message || err);
      }
    };

    slider.addEventListener("input", () => {
      const value = Number(slider.value);
      label.textContent = value;
      setSliderVisual(slider);
      clearTimeout(timers[key]);
      // Debounce: let firmware lerp; don't spam frames
      timers[key] = setTimeout(() => push(value), 120);
    });

    slider.addEventListener("change", () => {
      clearTimeout(timers[key]);
      push(Number(slider.value));
    });

    setSliderVisual(slider);
  }

  axes.forEach(bindAxis);

  els.connectBtn.addEventListener("click", toggleConnect);
  els.refreshBtn.addEventListener("click", () => refreshPorts().catch(console.error));
  els.centerBtn.addEventListener("click", () =>
    api("/api/action", { method: "POST", body: JSON.stringify({ action: "center" }) })
      .then(pollStatus)
      .catch((err) => (els.logLine.textContent = String(err.message || err)))
  );
  els.demoBtn.addEventListener("click", () =>
    api("/api/action", { method: "POST", body: JSON.stringify({ action: "demo" }) })
      .then(() => {
        els.logLine.textContent = "demo running…";
        setTimeout(pollStatus, 2500);
      })
      .catch((err) => (els.logLine.textContent = String(err.message || err)))
  );
  els.openBtn.addEventListener("click", () =>
    api("/api/set", { method: "POST", body: JSON.stringify({ axis: "grip", value: 40 }) })
      .then(pollStatus)
      .catch((err) => (els.logLine.textContent = String(err.message || err)))
  );
  els.closeBtn.addEventListener("click", () =>
    api("/api/set", { method: "POST", body: JSON.stringify({ axis: "grip", value: 140 }) })
      .then(pollStatus)
      .catch((err) => (els.logLine.textContent = String(err.message || err)))
  );

  refreshPorts()
    .then(pollStatus)
    .then(async () => {
      // Auto-connect if Nano port is present and not already connected
      const status = await api("/api/status");
      if (!status.connected && els.portSelect.value) {
        await toggleConnect();
      }
    })
    .catch(console.error);

  // Faster while arm is moving so sliders track the lerp
  setInterval(async () => {
    await pollStatus();
  }, 250);
})();
