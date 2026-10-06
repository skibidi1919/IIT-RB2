# Colour — deep check fixes

Bugs found and fixed:

1. **False RED lock on empty field** — tiny ambient Δ% (R≈10) locked RED; conf had a floor of 30 so it never unlocked.  
   → Presence gate (clear gain + chroma). No lock without a real object.

2. **Yellow broken** — Δ%-only `min(R,G)` / boost never matched yellow plastics (often look red-heavy).  
   → Bars = **NodeMCU chroma scores** again (`yPair = min(R,G)−B`), mapped 0–100.

3. **Lock** — locks after 3 clear majority samples; holds; switches if another bar leads for 4 samples; unlocks at **conf ≤ 10** with no artificial conf floor.
