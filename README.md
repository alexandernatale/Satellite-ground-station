# Ground Station — Satellite Communications & Command System

**RTS Summer 2026 Capstone — Alex Natale**
Theme: Space — Satellite Ground Station (S-Band Comms / Command & Control), tailored toward RF/Communications Systems Engineer roles.

- **Wokwi:** (https://wokwi.com/projects/471037581910912001)
-  **Pages:** https://alexandernatale.github.io/Satellite-ground-station/
-  **Video:** 

---

## Overview

A FreeRTOS ground-station simulation on an ESP32 that integrates five semester assignments  into one system: a ground-command ISR path, a shared telemetry counter protected by a mutex, a queue-based telemetry framing pipeline, a 3-channel downlink pool shared by 4 satellites, a Core-0 dashboard, and a priority-inversion self-test modeled on the 1997 Mars Pathfinder bug. Two faults are injectable for the fault-injection deliverable: disabling priority inheritance, and illegally taking a mutex from interrupt context.

---

## Architecture

- **Command path:** ISR (GPIO18) → binary semaphore → `ground_command_task` (prio 12)
- **Telemetry path:** `power_subsys_task` / `thermal_subsys_task` (prio 8) → mutex-protected counter → `comms_queue` (depth 8) → `telemetry_framer_task` (prio 6)
- **Downlink path:** `downlink_task` ×4 (prio 5) contend for a 3-slot counting semaphore, 1s bounded timeout
- **PI self-test:** H/M/L (prio 15/10/5) run once at boot, sharing `pi_lock`

All real-time tasks pinned to Core 1; dashboard (HTTP or serial) isolated on Core 0.

---

## Task Table & WCET Evidence

| Task | Priority | Core | Trigger | WCET max | Notes |

| `button_isr` | ISR | 1 | Button press | ~1-5 us (estimated) | Estimated, not measured — see below |
| `ground_command_task` | 12 | 1 | ISR wake | 84,965 us | Includes 80ms simulated delay; net compute ≈ 4,965 us |
| `downlink_task` ×4 | 5 | 1 | Pool availability | n/a | Contention-bound — report backoff rate |
| `power_subsys_task` | 8 | 1 | ~223 ms | 36 us | Manually timed; 36-37 us across separate runs — treat as the honest range, not a single fixed number |
| `thermal_subsys_task` | 8 | 1 | ~296 ms | 37 us | Manually timed; same 36-37 us run-to-run range as power |
| `telemetry_framer_task` | 6 | 1 | Queue item | 12 us | `frames_dropped` = 0 across the run |
| `beacon_task` | 3 | 1 | 1000 ms | trivial | Not real-time critical |
| PI self-test H/M/L | 15/10/5 | 1 | Boot | — | See Fault Injection |

**Downlink backoff rate:** 0% — computed, not counted. A discrete-event simulation of the pool-acquisition algorithm (using the code's actual constants: 700/900/1100/1300ms transmit times per satellite, 100ms cooldown, 1000ms timeout, FIFO ordering among equal-priority waiters) ran clean over a simulated hour with zero timeouts; the closest any satellite ever got to the 1000ms threshold was a 900ms wait. This is a calculated result from the algorithm's own timing, not a count from real Wokwi logs — worth spot-checking against actual `[SAT-N] ... backed off` lines if you have a spare run, since the simulation assumes zero scheduling jitter, which real FreeRTOS execution won't have exactly.

**Two open findings from this data:**
- `gc_lat` converged at 82,932 us — traced to `sig_sem` being a binary semaphore and `isr_entry_time_us` a single shared scalar, so a second press during the 80ms execution window overwrites the timestamp and silently drops its own signal. Real defect, not measurement noise — see H-07.
- `button_isr`'s GPIO19 trace shows two edges per press instead of one microsecond-scale blip, unexpected for a `NEGEDGE`-only ISR. Unresolved — needs a `ΔT` cursor measurement to confirm cause. The 1-5us estimate above is a rough order-of-magnitude figure reasoned from comparable RTOS-call overhead elsewhere in this codebase (the ISR does one `xSemaphoreGiveFromISR` call and no logging, vs. the framer's 12us which includes a log line) — not a substitute for the real measurement.

---

## Fault Injection

| Mode | H's measured wait |

| Mutex (priority inheritance ON) | 12,185.25 ms |
| Binary semaphore (no inheritance) | 36,776.98 ms |

12.19s (mutex mode) is real, not a bug — the self-test runs under full system load here rather than in isolation, and `pi_low_task` shares priority 5 with the four downlink tasks. Inheritance still bounded the wait; it just bounded it to a busier system than the isolated version would show. The binary-sem number (~3x longer) confirms the mechanism directly: without inheritance, M (priority 10) repeatedly preempted L mid-critical-section, so H ended up waiting behind both of them instead of just L's remaining work.

**Illegal ISR mutex take (`INDUCE_MUTEX_FROM_ISR`):** confirmed — not a crash, not a silent no-op. With the flag on and the button pressed repeatedly, `ground_command_task` kept running normally (`hb_gc` climbed steadily across every log block), but `power_subsys_task`, `thermal_subsys_task`, and all four `downlink_task` instances froze permanently: `telemetry_seq` stuck at 2, `pwr`/`thm` heartbeats stuck at 1 each, all four `hb_dl` counters stuck at 0, `channels_avail` stuck at 0/3 — with no recovery across several seconds of continued presses. Likely explanation (not independently confirmed via task-list introspection, but consistent with every symptom observed): `xSemaphoreTake` on a mutex internally suspends the scheduler as part of its bookkeeping, and calling it from ISR context on every press corrupted that bookkeeping badly enough to starve every task except the one directly woken by the ISR's own paired give+yield.

---

## Hazard Analysis

| ID | Hazard | Severity | Mitigation | Residual Risk |

| H-01 | Priority inversion on command path | Catastrophic (Mars Pathfinder precedent) | Mutex w/ priority inheritance | Bound holds, but ~12.2s measured under load — inheritance doesn't bound critical-section length |
| H-02 | Downlink pool exhaustion (4 tasks, 3 channels) | Moderate | 1s bounded timeout + backoff logging | No fairness scheme; one satellite could starve |
| H-03 | Illegal mutex take from ISR | Catastrophic — confirmed to cause system-wide task starvation, not just localized corruption | Never call blocking APIs from ISR context | Empirically freezes every non-command task permanently within seconds — see Fault Injection |
| H-04 | Comms queue overflow | Minor–moderate | Bounded queue, timeout + drop-and-log | 0 drops observed at nominal load; no alarm on repeated drops |
| H-05 | WCET overrun on command path | Moderate–high | Continuous WCET tracking, +30% margin, watchdog | Single-run max isn't a formal worst-case proof |
| H-06 | Watchdog trip from CPU-bound self-test | Low here / High if shipped | Watchdog reconfigured for the test | Shouldn't run inline with real-time tasks in production |
| H-07 | Dropped/aliased ground commands under rapid presses | Catastrophic (silent loss) | None implemented | Fix: counting semaphore + timestamp queue instead of shared scalar |



## AI Usage Disclosure
AI was used for formatting of code and commenting. AI was also used for explaining induced failure and formatting of the REWADME file.
