# Changelog — OpenTrickler Autotune

All notable changes to **OpenTrickler Autotune** are documented here.
Versions follow [Semantic Versioning](https://semver.org/):
`MAJOR` for incompatible changes (e.g. an EEPROM layout change that resets
settings), `MINOR` for new features, `PATCH` for bug fixes.

The released version string lives in `RELEASE_VERSION`; it must match the git
tag and the GitHub release name exactly, because the on-device update check
compares the version strings literally.

## [Unreleased]

## [v1.2.0] — 2026-10-05

### Added

- **Basic / Advanced settings mode** (toggle in the settings header, remembered
  per browser, Basic by default). Basic hides expert options; each page shows
  "N advanced settings hidden · Show". Hidden fields keep their values and are
  still sent unchanged with Apply.
  - Profile page reorganised: name, controller, tolerance, decimal places,
    Manual Finish and auto-zero up front; everything else in collapsible
    groups (Stop Thresholds, PID Gains & Flow Speeds, Scale Settle,
    Kernel/Cut Kernel/Reverse Tube, Pre-Charge & Pulse Mode, LED Colours).
  - Motor, Neopixel LED and Servo Gate pages only in Advanced mode.
  - AI Tuning (live details, safety guards, characterization settings, runtime
    history), PID Tuning (safety margins and test settings; cup capacity stays
    visible), Firmware Update (repository, manual OTA), System (IDs, VCS hash,
    build type, Erase EEPROM) and Scale (serial format) partly in Advanced.
  - Menu: "AI / PID Tuning" opens the page matching the profile's controller;
    "Display & Buttons" and "System" renamed.
- **Compact WiFi page:** saved networks as a list with "connected" marker and
  whether a password is stored; inline edit, add, forget (now also for
  network 1); WiFi on/off switch; security type and connect timeout in Advanced.
  Firmware reports the joined slot (`cs`) and stored-password flags (`pk`).

### Fixed

- **REST parameters with `&` or `=` in a value** (e.g. WiFi passwords or SSIDs)
  were split wrongly because the URI was percent-decoded before being split
  into parameters. Parameters are now split first, then decoded one by one.
- WiFi connect timeout over 65 s wrapped around (parsed as 16 bit); now
  clamped to 5–120 s.

### Removed

- Leftover "Fork Build Marker" field from the TP fork on the System page.

## [v1.1.0] — 2026-10-04

### Added

- **Charge Timeline** card on the Trickler page (below Start/Stop): weight over
  time for the last charge with coarse / fine / settle phases, target and
  tolerance band, handoff point and motor speed; a zoomed "Fine approach"
  view of the last 3 gn showing motor stop, tail and final result; chips for
  final weight, error, coarse and fine time. Updates live while charging.
  "Last 5" overlays the previous charges (kept in the browser).
- Firmware records each charge at 10 Hz (`charge_timeline.c`, decimates
  automatically for long charges) and serves it at `/rest/charge_timeline`
  (incremental via `id` / `gen` / `from`).

## [v1.0.0] — 2026-10-04

Based on TP Custom Rifle Parts fork `v2026.09.23-fork.32` (commit `674afab`).

### Fixed

- **Setup hotspot:** `access_point_mode_start()` declared the DNS server as a
  local variable shadowing the static one, leaving lwIP with a dangling
  pointer to stack memory; most likely crashed the board on the first DNS
  query from a connecting client. Wireless task stack 512 -> 1024 words.
- **AI characterization stalled on "Remove Cup":** the coarse stop snapshot
  used the normal-charge plausibility check (> 3 % of the charge target), so
  small early characterization pulses returned NaN and were never recorded.
  The AI sample path now uses the raw stop measurement (the measured coarse
  on-time no longer includes that helper's up-to-320 ms wait).
- **Microstepping changed motor speed:** microsteps / current / R-sense set
  from the web UI only reached the step-rate maths, not the TMC2209, until a
  reboot. Now applied to the driver immediately with MRES read-back.
- **Fine trickler stopped at minimum speed with <= 64 microsteps:** maximum
  PIO step period raised from 10 ms to 100 ms (min. 10 steps/s); zero / tiny
  speeds guarded against `lroundf` overflow.
- WiFi SSIDs are JSON-escaped in the REST response.

### Added

- Animated 10 s boot splash "OpenTrickler / AUTOTUNE"; any button skips it.
  Runs in the menu task, so WiFi connects in the background meanwhile.
- "OpenTrickler Autotune" header in the web UI.
- PID Autotune, AI powder characterization and AI machine calibration can be
  started from the encoder menu (routed by the profile's controller), with
  on-device save / discard of the result.
- Reset button goes one page back in the menus (ends field editing first).
- Up to 5 known WiFi networks: scan, strongest visible first, then hidden
  SSIDs, then fall back to the setup hotspot.

### Changed

- Web UI restored to the original OpenTrickler (DaisyUI default) look without
  branding; Appearance / theming page removed; original OpenTrickler icons.
- Cleanup speed limited to +/-10 rps, adjusted in 0.5 rps steps.
- OTA update source defaults to `Bleach3D/OpenTrickler-Autotune`.
- HTTP request limits raised (URI 2000, request 2400 bytes).

---

# Earlier history — TP Custom Rifle Parts fork


This file documents the changes made in **this fork** (TP Custom Rifle Parts)
on top of [WhoKilledBambiLabs/Opentrickler_ML](https://github.com/WhoKilledBambiLabs/Opentrickler_ML),
which is itself a fork of the original [Eamars/OpenTrickler](https://github.com/eamars/OpenTrickler)
firmware. Upstream's own AI-algorithm-tuning history (coarse handoff logic,
recovery bounds, etc.) is tracked separately in
[`firmware_release_history/CHANGELOG.md`](firmware_release_history/CHANGELOG.md)
and is not duplicated here.

Base: `Opentrickler_ML` `2026.07.12-beta.14`.

## [Unreleased]

### Added

- **PID Tuning — automatic PID/speed characterization** — new dedicated
  "PID Tuning" settings page (the bottom-nav's 4th icon swaps to it
  automatically whenever the active profile's Controller is set to PID)
  that runs a ladder of fixed-speed coarse and fine throws, measures flow
  rate (gn/s per rps) and scale-report lag from them, and fits a
  coarse/fine handoff speed, fine landing speed, and coarse stop threshold
  sized to your configured safety margins (a coarse-stop safety multiple
  of measured scatter, and a fine-landing sigma against your Accepted
  Charge Tolerance) and a target total throw time. The run stays on the
  physical trickler the whole time — no simulation. Results (Kp, min/max
  speeds, fine taper, stop threshold, predicted timing, whether the time
  goal was met) can be applied to the active profile in RAM for an
  immediate test-throw, or saved straight to EEPROM. Adapted from the
  "Learn Powder" characterization approach in
  [magnaludus/OpenTrickler-RP2040-Controller](https://github.com/magnaludus/OpenTrickler-RP2040-Controller),
  reworked for this fork's PID-only `charge_mode` (no lag-compensated stop
  prediction) and wired into the existing REST/menu/EEPROM architecture.
  v1 does not include that reference project's automated confirm-throws
  loop or automatic cup-dump handling — it aborts rather than pausing if a
  throw would overfill the cup. New REST endpoints:
  `/rest/pid_autotune_state`, `/rest/pid_autotune_config`,
  `/rest/pid_autotune_config_set`, `/rest/pid_autotune_start`,
  `/rest/pid_autotune_action`. New EEPROM region
  (`EEPROM_PID_AUTOTUNE_CONFIG_BASE_ADDR`, 17K) — brand new, so it doesn't
  affect or reset any existing settings.
- **Controller moved to the top of Profile settings** — the PID/Adaptive
  Controller selector (previously buried mid-page among the LED colour
  fields) now sits directly under the profile picker, since it decides
  which of the settings below actually apply. The PID Gains fields (only
  relevant while Controller is set to PID) moved into their own section
  at the very bottom of the page to match.
- **Bottom-nav tuning shortcut now follows your Controller choice** — the
  4th bottom-nav icon reads "PID Tuning" when the active profile's
  Controller is set to PID, or "AI Tuning" when it's set to Adaptive,
  updating live as you change the Controller dropdown and re-checked
  automatically whenever you switch profiles. Both pages remain reachable
  from the Settings drawer regardless of which one the shortcut points at.
- **AI Tuning — Extra Safety Margin** — new adjustable slider (0–50%,
  default 0%) in the AI Tuning page's Advanced Config panel, for users
  reporting overthrows on the AI-suggested values. It scales the computed
  coarse/fine stop thresholds outward before the existing Kp cap is
  applied, so it composes correctly with — rather than fights — every
  other safety mechanism already in that calculation. It is purely
  additive/conservative: it can only make the controller stop earlier
  than it otherwise would, never later, so raising it cannot itself cause
  an overthrow. New `safety_margin_pct` field (`/rest/ai_tuning_config`),
  EEPROM revision bumped to accommodate it.
- **Real logo on the iOS/Android Home Screen icon** — the Home Screen
  icon shown when the web portal is saved to a phone's home screen (and
  the browser favicon) now uses the actual TP Custom Rifle Parts logo
  instead of the placeholder icon.
- **Reverse Tube (anti-dribble)** — after a tube reaches its own genuine
  final stop for a charge (not on an abort), it can be briefly reversed to
  pull back the last bit of powder sitting in the tube/gate, reducing
  post-stop dribble. Configurable independently for the coarse and fine
  tubes (enable + number of motor revolutions, 0–10, typical 0.05–0.3 rev
  coarse / 0.02–0.15 rev fine), each reusing that tube's own configured
  minimum flow speed rather than needing a separate reverse-speed setting.
  New settings page section under Reverse Tube, next to Fine Trickler Stop
  Threshold.
- **Background colour theming** — Appearance settings previously only let
  you recolour buttons/accents. Background colour is now independently
  customizable with its own colour picker and six presets (Warm Cream, Cool
  Grey, Slate Blue, Sage, Pure White, Stone), plus a "Reset appearance to
  defaults" button. Both button accent and background persist across
  sessions and are derived consistently in light and dark mode.
- **Controller labeling and documentation link** — the Controller dropdown
  now reads "PID - Original OpenTrickler Firmware" / "Adaptive - Based on
  Opentrickler_ML Firmware", with a link to the
  [Opentrickler_ML repository](https://github.com/WhoKilledBambiLabs/Opentrickler_ML)
  so it's clear which controller you're choosing and where the adaptive
  logic comes from.
- **Manual Finish** — for large-kernel powders (e.g. N565/N570-class
  extruded stick powder, ~0.08 gn/kernel) where your Accepted Charge
  Tolerance is tighter than one whole kernel, the trickler stops short of
  guessing, blinks the under-charge LED, and tells you how many hand-cut
  partial kernels to add. Configurable cut-kernel weight; verifies and
  reports pass/fail exactly like an automatic finish.
- **Accepted Charge Tolerance** — the ± grains band that decides a "good"
  charge is now a per-profile setting (previously a hardcoded 0.0205 gn).
  Statistics can optionally include out-of-tolerance rejects for
  diagnosing the trickler itself rather than judging load consistency.
- **Per-profile charge mode settings** — charge-mode configuration (stop
  thresholds, stabilization, pulse mode, Accepted Charge Tolerance, Manual
  Finish, Reverse Tube, LED colours, etc.) moved from one global config to
  one set per profile, with matching REST and web-portal support. Switch
  profiles and your tuning switches with it.
- **Kernel-weight-aware AI tuning** — the model estimates single-granule
  powder weight and uses it as a resolution floor when finishing a charge,
  with a manual override (measured kernel weight) available via
  `/rest/ai_kernel_weight` and the UI, for when you've weighed a known
  count of kernels yourself.
- **Accuracy vs Speed bias** — per-profile slider (0.0 = prioritise speed,
  1.0 = prioritise accuracy, 0.5 = balanced) that scales how much finishing
  margin and settle patience the fine phase gets.
- **Branding** — TP Custom Rifle Parts logo in the web portal and as the
  boot-splash bitmap on the 128x64 display, plus a modernized UI with light
  and dark mode.
- **Check for Updates from the GUI (direct from GitHub, real TLS)** —
  Settings → Firmware Update now has a "Check for Updates" section. Set it
  to your GitHub owner/repo once (via the new **GitHub Owner / Org** /
  **GitHub Repository** fields), and the device checks
  `github.com/<owner>/<repo>/releases/latest/download/manifest.json` and
  `.../app.bin` directly over a real, certificate-verified HTTPS
  connection — no server of your own to run, no maintainer secrets, nothing
  to keep online. A minimal TLS 1.2 client (mbedTLS layered over the
  existing lwIP sockets stack, `src/ota_tls.c`) verifies GitHub's
  certificate chain against a small curated bundle of well-known root CAs
  embedded in the firmware (`src/ota_tls_roots.c`, sourced from Mozilla's
  root program via `certifi`), the same way a browser would, before
  trusting anything it downloads. The device shows the latest version and
  your notes/changelog text, and — with "Update Now" — downloads `app.bin`
  and stages it in the existing OTA staging area used by
  `tools/ota_upload.py`, all over WiFi with no laptop involved. Both
  checking and applying are explicit button presses — nothing runs on a
  timer or happens without you pressing a button, since this firmware
  controls reloading equipment. New REST endpoints: `/rest/update_config`,
  `/rest/update_check`, `/rest/update_status`, `/rest/update_apply`. New
  EEPROM region for the owner/repo settings (`EEPROM_UPDATE_CONFIG_BASE_ADDR`,
  15K) — this is a brand-new region, so it doesn't affect or reset any
  existing settings. `.github/workflows/publish-update-manifest.yml`
  automates the publishing side: on every published GitHub release it
  builds the firmware (same steps as `cmake.yml`), runs
  `tools/publish_update_manifest.py`, and attaches `manifest.json` +
  `app.bin` straight to that same release using GitHub's own automatically
  provided token — **zero repository secrets to create or manage**. See
  QUICKSTART.md section 11.

### Removed

- **"Suggested PID Baseline" panel on AI Tuning** — removed the
  auto-suggested PID/threshold table (and its "Save Suggested PIDs" /
  "Save Both" buttons and "Learn From Real Throws" toggle) now that PID
  Tuning has its own dedicated, purpose-built characterization page (see
  Added, above). AI Tuning now focuses on the adaptive/AI controller
  itself: characterization, calibration, and the learned model — not on
  producing PID numbers for the separate PID controller. The Extra Safety
  Margin slider that lived in this panel was kept and moved into Advanced
  Config, since it's a characterization run parameter rather than a PID
  suggestion.

### Changed

- **LCD status line tidied up** — the on-device status line previously
  showed both the target weight and the remaining amount, duplicating the
  title bar's own "Target: X.XXX" line. It now shows a short phase label
  (e.g. coarse/fine/settling/recovering) alongside the remaining amount
  instead, so the same information isn't shown twice and the phase is
  visible at a glance.
- **LCD decimal places default to 2** — the main screen weight readout's
  default display precision. `Decimal Places` remains a per-profile
  setting (Settings → Profile), adjustable from 0–3 independent of this
  default at any time.
- **Export Profile / Export Config now capture everything needed to
  restore a profile** — previously only the base profile configuration was
  exported. Export now also includes each profile's charge-mode settings
  (stop thresholds, Manual Finish, Reverse Tube, tolerances, etc.) and its
  kernel weight setting, for both single-profile export and full
  8-profile config export, so an exported file round-trips back into a
  complete, working profile rather than a partial one.
- **Coarse stop authority** — when the adaptive controller is in use, it
  can no longer silently override your configured coarse stop threshold;
  the clamp to that threshold is applied after all of the controller's
  internal floors, not before.
- **Undercharge salvage fix** — the top-up routine no longer gives up on
  shortfalls over a flat 0.30 gn, which previously could prompt "remove
  cup" while the charge was still well under target.

### Fixed

- **"Remove Cup" could get stuck permanently after the cup was returned**
  — the loop waiting for the cup to come back used the same "large
  negative excursion" formula that detects the cup being *lifted off* the
  scale, copy-pasted in where the opposite "near zero and settled" test
  belonged. With the default zero-band margin, that condition could only
  ever be true while the cup stayed off the scale, so returning it (the
  expected way to finish) left "Remove Cup" showing red indefinitely even
  once the scale read zero.
- **"Remove Cup" LED showed a stale colour while the cup was actually
  off the scale** — while the cup was being lifted or was genuinely
  removed, the last charge result's red/green/etc. stayed on screen
  instead of the "not ready" blue, so putting the cup back briefly showed
  the old colour before catching up. The LED now shows "not ready" for
  the whole time the cup is off/moving, matching the colour shown before
  a charge starts.
- **LCD "Target:" line now always shows 2 decimal places**, independent
  of the Decimal Places setting, which continues to control only the live
  weight readout below it. Previously both lines followed the same
  setting, making the header cramped at 3 decimal places for no benefit
  (the target is a fixed number you set once, not a live reading).
- **iOS Home Screen (installed as a web app) status bar overlap** — with
  `apple-mobile-web-app-status-bar-style: black-translucent`, iOS draws
  the status bar transparently over the page instead of reserving space
  for it. The Settings page's top bar, the settings drawer's slide-out
  side menu, and the Trickler page's own header all rendered underneath
  the clock/status icons as a result. All three now pad themselves clear
  of the status bar/notch using `env(safe-area-inset-top)`. Also fixes a
  missing comma in the `viewport` meta tag that was silently preventing
  `viewport-fit=cover` from taking effect at all.

- **AI-tuning overthrow on large trickler tubes, and on transferring AI
  suggestions to PID** — root-caused to two related issues:
  1. Settle detection used a shorter timeout window during AI
     characterization (1400 ms) than during normal charges (3000 ms), so
     characterization could record a sample before the scale had actually
     settled, especially with the higher-capacity large tube. Both paths
     now use a consistent ~3000 ms settle window, and stage-sample
     selection now prefers samples that were confirmed settled over ones
     that were cut off by the timeout.
  2. `/rest/ai_suggestions` (the "Suggested PID Baseline" AI tuning
     produces for you to review and apply) now checks each tube's
     currently-configured minimum flow speed and widens the suggested stop
     thresholds and Kp cap accordingly, instead of assuming
     characterization's own sample speed is always representative. This
     is what caused suggested PID values to overthrow by a large margin
     (reported up to ~20 gn) specifically on the large tube size after
     applying them.

  Together these should make characterization results, and the PID values
  suggested from them, track the actual tube size in use rather than
  assuming one flow profile for all three tube sizes. As before,
  characterization should still be re-run for each tube size — a model
  learned on one tube size is not assumed to transfer to another.

- **Unreadable "neutral" buttons (dark text on a dark button) with the new
  Background Colour picker** — the Appearance page's Background Colour
  feature was overriding `--nc` (the text colour used on every
  `.btn-neutral` button, including the Settings "Apply" button and the
  "Apply" / "Save to EEPROM" confirmation dialog) to a value derived from
  the picked page background. `.btn-neutral`'s own background (`--n`) is a
  fixed dark tone that never changes with that picker, so picking a light
  page background (including the default) paired dark button text with
  that still-dark button, making it unreadable. `--nc` is no longer tied
  to the Background Colour picker and keeps its correct, fixed contrast
  against `--n` in both light and dark mode.

- **Renaming a profile silently didn't save, even though a "Settings
  Applied" success message appeared.** The Profile settings page reads as
  one continuous screen, but it's actually two separate forms stacked in
  the same section: the Profile Name and PID gains (`/rest/profile_config`)
  are one form with its own "Apply" button partway down the page, and
  everything below that (LED colours, stop thresholds, Controller, Manual
  Finish, Reverse Tube, pulse/stabilization settings) is a second form
  (`/rest/charge_mode_config`) with its *own* "Apply" button at the very
  bottom. Editing the name, scrolling straight past the first form's Apply
  button, and clicking the second form's Apply button at the bottom
  submitted only the charge-mode fields — the name change was never sent,
  even though the dialog reported success. Clicking Apply or Save to
  EEPROM from *either* button on this page now saves both forms together,
  so it no longer matters which one you reach first.

### Notes for anyone updating from an earlier build of this fork

- The EEPROM charge-mode data revision has been bumped to accommodate the
  Reverse Tube fields. **Charge-mode settings (stop thresholds,
  tolerances, Manual Finish, Reverse Tube, LED colours, etc.) will reset
  to defaults on first boot after flashing.** Base motor/profile
  calibration and AI characterization history are not affected. Re-enter
  your tuned values (or re-run characterization) after updating.
- The PC simulator (`sim/`) from an earlier development pass is not
  actively maintained or distributed with this fork's releases; only the
  Pico 2 W / RP2350 firmware (`.uf2`) is built and shipped going forward.
