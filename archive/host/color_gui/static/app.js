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
    swatch: $("swatch"),
    guess: $("guess"),
    conf: $("conf"),
    hex: $("hex"),
    valR: $("valR"),
    valG: $("valG"),
    valB: $("valB"),
    valC: $("valC"),
    barR: $("barR"),
    barG: $("barG"),
    barB: $("barB"),
    barC: $("barC"),
  };

  let connected = false;

  async function api(url, opts = {}) {
    const res = await fetch(url, {
      headers: { "Content-Type": "application/json" },
      ...opts,
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.error || res.statusText);
    return data;
  }

  function setConnectedUi(on, detail = "") {
    connected = on;
    els.linkDot.dataset.state = on ? "on" : detail ? "err" : "off";
    els.linkLabel.textContent = on ? "Connected" : detail ? "Error" : "Disconnected";
    els.linkDetail.textContent = on ? detail || "serial" : detail || "Select the ESP32 serial port";
    els.connectBtn.textContent = on ? "Disconnect" : "Connect";
    els.connectBtn.classList.toggle("on", on);
    els.portSelect.disabled = on;
  }

  function pulseBar(el, value, maxHint = 120) {
    const pct = Math.max(0, Math.min(100, 100 - (value / maxHint) * 100));
    el.style.width = `${pct}%`;
  }

  function applyStatus(data) {
    if (data.connected !== connected) {
      setConnectedUi(!!data.connected, data.port || data.error || "");
    } else if (data.error && !data.connected) {
      setConnectedUi(false, data.error);
    }

    const hex = data.hex || "#000000";
    els.swatch.style.setProperty("--swatch", hex);
    els.guess.textContent = data.label || "—";
    if (els.conf) {
      const label = data.label || "—";
      const pending = data.pending && data.pending !== "—" ? data.pending : "";
      const prog = Math.round((data.lock_progress || 0) * 100);
      if (pending && pending !== label) {
        els.conf.textContent = `locking ${pending} ${prog}%`;
      } else if (label === "—" || !label) {
        els.conf.textContent = "hold colour 5–20 cm from sensor";
      } else {
        els.conf.textContent = `locked ${label}`;
      }
    }
    els.hex.textContent = hex;
    els.valR.textContent = String(data.r ?? 0);
    els.valG.textContent = String(data.g ?? 0);
    els.valB.textContent = String(data.b ?? 0);
    els.valC.textContent = String(data.c ?? 0);

    const peak = Math.max(data.r || 0, data.g || 0, data.b || 0, data.c || 0, 40);
    pulseBar(els.barR, data.r || 0, peak * 1.2);
    pulseBar(els.barG, data.g || 0, peak * 1.2);
    pulseBar(els.barB, data.b || 0, peak * 1.2);
    pulseBar(els.barC, data.c || 0, peak * 1.2);

    if (data.last_line) els.logLine.textContent = data.last_line;
  }

  async function refreshPorts() {
    const data = await api("/api/ports");
    const cur = els.portSelect.value;
    els.portSelect.innerHTML = "";
    (data.ports || []).forEach((p) => {
      const opt = document.createElement("option");
      opt.value = p.device;
      opt.textContent = p.description ? `${p.device} — ${p.description}` : p.device;
      els.portSelect.appendChild(opt);
    });
    if ([...els.portSelect.options].some((o) => o.value === cur)) {
      els.portSelect.value = cur;
    } else if ([...els.portSelect.options].some((o) => o.value.toUpperCase() === "COM4")) {
      els.portSelect.value = "COM4";
    }
  }

  els.refreshBtn.addEventListener("click", () => refreshPorts().catch(console.error));
  els.connectBtn.addEventListener("click", async () => {
    try {
      if (connected) {
        await api("/api/disconnect", { method: "POST", body: "{}" });
        setConnectedUi(false);
      } else {
        const port = els.portSelect.value;
        const data = await api("/api/connect", {
          method: "POST",
          body: JSON.stringify({ port, baud: 115200 }),
        });
        setConnectedUi(true, data.port || port);
      }
    } catch (err) {
      setConnectedUi(false, String(err.message || err));
    }
  });

  async function poll() {
    try {
      const data = await api("/api/status");
      applyStatus(data);
    } catch {
      /* ignore */
    }
  }

  refreshPorts().catch(console.error);
  setInterval(poll, 200);
})();
