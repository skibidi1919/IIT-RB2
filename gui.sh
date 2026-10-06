#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT/gui"
export PATH="$HOME/bin:$PATH"

if curl -fsS --max-time 1 http://127.0.0.1:5050/api/status >/dev/null 2>&1; then
  echo "Meowler GUI already running → http://127.0.0.1:5050"
else
  nohup python3 app.py >"$ROOT/gui/server.log" 2>&1 &
  SERVER_PID=$!
  for _ in $(seq 1 50); do
    if curl -fsS --max-time 1 http://127.0.0.1:5050/api/status >/dev/null 2>&1; then
      break
    fi
    sleep 0.1
  done
  if curl -fsS --max-time 1 http://127.0.0.1:5050/api/status >/dev/null 2>&1; then
    echo "Meowler GUI started (pid $SERVER_PID) → http://127.0.0.1:5050"
  else
    echo "Failed to start GUI. See $ROOT/gui/server.log"
    exit 1
  fi
fi

# Open browser without blocking this shell
if command -v cmd.exe >/dev/null 2>&1; then
  (cmd.exe /c start "" "http://127.0.0.1:5050" >/dev/null 2>&1 &) || true
elif command -v xdg-open >/dev/null 2>&1; then
  (xdg-open "http://127.0.0.1:5050" >/dev/null 2>&1 &) || true
fi
