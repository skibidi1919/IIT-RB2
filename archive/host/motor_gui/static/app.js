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
    motors: $("motors"),
    globalSpeed: $("globalSpeed"),
    globalSpeedVal: $("globalSpeedVal"),
    allFwd: $("allFwd"),
    allRev: $("allRev"),
    allStop: $("allStop"),
  };

  const names = ["M1 front A", "M2 front A", "M3 back B", "M4 back B"];
  const local = names.map((name, id) => ({ id, name, dir: "stop", speed: 180 }));
  let connected = false;

  function setSliderVisual(slider) {
    const min = Number(slider.min);
    const max = Number(slider.max);
    const val = Number(slider.value);
    const pct = ((val - min) / (max - min)) * 100;
    slider.style.setProperty("--pct", `${pct}%`);
  }

  async function api(url, opts = {}) {
    const res = await fetch(url, {
      headers: { "Content-Type": "application/json" },
      ...opts,
    });
    const data = await res.json().catch(() => ({}));
    if (!res.ok) throw new Error(data.error || res.statusText);
    return data;
  }

  function renderMotors() {
    els.motors.innerHTML = local
      .map(
        (m) => `
      <article class="motor" data-id="${m.id}">
        <header>
          <div>
            <span class="ch">CH${m.id}</span>
            <h2>${m.name}</h2>
          </div>
          <span class="state" data-state>${m.dir}@${m.speed}</span>
        </header>
        <label class="speed-field">
          <span>Speed</span>
          <input class="slider motor-speed" type="range" min="0" max="255" value="${m.speed}" data-id="${m.id}" ${connected ? "" : "disabled"} />
          <strong data-spd>${m.speed}</strong>
        </label>
        <div class="dirs">
          <button type="button" class="btn action dir-fwd ${m.dir === "fwd" ? "dir-on" : ""}" data-id="${m.id}" data-dir="fwd" ${connected ? "" : "disabled"}>Fwd</button>
          <button type="button" class="btn action dir-stop ${m.dir === "stop" ? "dir-on" : ""}" data-id="${m.id}" data-dir="stop" ${connected ? "" : "disabled"}>Stop</button>
          <button type="button" class="btn action dir-rev ${m.dir === "rev" ? "dir-on" : ""}" data-id="${m.id}" data-dir="rev" ${connected ? "" : "disabled"}>Rev</button>
        </div>
      </article>`
      )
      .join("");

    els.motors.querySelectorAll(".motor-speed").forEach((slider) => {
      setSliderVisual(slider);
      slider.addEventListener("input", () => {
        const id = Number(slider.dataset.id);
        const speed = Number(slider.value);
        local[id].speed = speed;
        slider.parentElement.querySelector("[data-spd]").textContent = String(speed);
        setSliderVisual(slider);
      });
      slider.addEventListener("change", async () => {
        const id = Number(slider.dataset.id);
        const speed = Number(slider.value);
        if (local[id].dir === "stop") return;
        try {
          await api("/api/motor", {
            method: "POST",
            body: JSON.stringify({ id, dir: local[id].dir, speed }),
          });
        } catch (err) {
          els.logLine.textContent = String(err.message || err);
        }
      });
    });

    els.motors.querySelectorAll("[data-dir]").forEach((btn) => {
      btn.addEventListener("click", async () => {
        const id = Number(btn.dataset.id);
        const dir = btn.dataset.dir;
        const speed = local[id].speed;
        try {
          await api("/api/motor", {
            method: "POST",
            body: JSON.stringify({ id, dir, speed }),
          });
          local[id].dir = dir;
          renderMotors();
        } catch (err) {
          els.logLine.textContent = String(err.message || err);
        }
      });
    });
  }

  function setConnectedUi(on, detail = "") {
    connected = on;
    els.linkDot.dataset.state = on ? "on" : detail ? "err" : "off";
    els.linkLabel.textContent = on ? "Connected" : detail ? "Error" : "Disconnected";
    els.linkDetail.textContent = on ? detail || "serial" : detail || "Select the Nano serial port";
    els.connectBtn.textContent = on ? "Disconnect" : "Connect";
    els.connectBtn.classList.toggle("on", on);
    els.portSelect.disabled = on;
    [els.allFwd, els.allRev, els.allStop].forEach((b) => {
      b.disabled = !on;
    });
    renderMotors();
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
    } else if ([...els.portSelect.options].some((o) => o.value.toUpperCase() === "COM3")) {
      els.portSelect.value = "COM3";
    }
  }

  async function poll() {
    try {
      const data = await api("/api/status");
      if (data.connected !== connected) {
        setConnectedUi(!!data.connected, data.port || data.error || "");
      } else if (data.error && !data.connected) {
        setConnectedUi(false, data.error);
      }
      if (Array.isArray(data.motors)) {
        data.motors.forEach((m, i) => {
          if (!local[i]) return;
          local[i].dir = m.dir || local[i].dir;
          if (typeof m.speed === "number" && m.dir !== "stop") {
            local[i].speed = m.speed;
          }
        });
        els.motors.querySelectorAll(".motor").forEach((card) => {
          const id = Number(card.dataset.id);
          const st = card.querySelector("[data-state]");
          if (st) st.textContent = `${local[id].dir}@${local[id].speed}`;
          card.querySelectorAll("[data-dir]").forEach((btn) => {
            btn.classList.toggle("dir-on", btn.dataset.dir === local[id].dir);
          });
        });
      }
      if (data.last_line) els.logLine.textContent = data.last_line;
    } catch {
      /* ignore poll blips */
    }
  }

  els.globalSpeed.addEventListener("input", () => {
    els.globalSpeedVal.textContent = els.globalSpeed.value;
    setSliderVisual(els.globalSpeed);
  });
  setSliderVisual(els.globalSpeed);

  async function allCmd(dir) {
    const speed = Number(els.globalSpeed.value);
    try {
      await api("/api/all", {
        method: "POST",
        body: JSON.stringify({ dir, speed }),
      });
      local.forEach((m) => {
        m.dir = dir;
        if (dir !== "stop") m.speed = speed;
      });
      renderMotors();
    } catch (err) {
      els.logLine.textContent = String(err.message || err);
    }
  }

  els.allFwd.addEventListener("click", () => allCmd("fwd"));
  els.allRev.addEventListener("click", () => allCmd("rev"));
  els.allStop.addEventListener("click", () => allCmd("stop"));

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

  renderMotors();
  refreshPorts().catch(console.error);
  setInterval(poll, 400);
})();
