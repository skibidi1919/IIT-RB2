# RFGYC Senior AM — color triage sketch (Arduino stack + host DSL)
# Record drop_red / drop_yellow / drop_green as .rpm first, then Run this.

log "triage: spin belt briefly"
conveyor 160
wait 900
conveyor 0

log "wait for color"
wait_color any 6000

if_color red
  log "RED -> hospital drop"
  play drop_red
elif_color yellow
  log "YELLOW -> PCC drop"
  play drop_yellow
elif_color green
  log "GREEN -> RZ drop"
  play drop_green
else
  log "unknown color — stop"
end

stop
