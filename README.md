# OpenTrickler Autotune

Firmware for the [OpenTrickler](https://github.com/eamars/OpenTrickler) powder
trickler on a **Raspberry Pi Pico 2 W**. It tunes itself to your powder and
machine to throw accurate charges quickly, with a web interface for setup,
tuning and charge history.

> **Safety first.** This firmware runs reloading equipment. Check every charge
> on a calibrated scale, stay with the machine while the motors are running,
> and keep a known-good firmware file so you can roll back.

## Contents

- [What you need](#what-you-need)
- [Installing](#installing)
- [Updating](#updating)
- [First setup](#first-setup)
- [Everyday use](#everyday-use)
- [Features](#features)
- [Troubleshooting](#troubleshooting)
- [Credits and license](#credits-and-license)

## What you need

- An assembled OpenTrickler (trickler, motors, scale interface, 128x64
  display with encoder), built and wired according to the
  [original OpenTrickler hardware documentation](https://github.com/eamars/OpenTrickler).
- A **Raspberry Pi Pico 2 W** (RP2350) as the controller. The older Pico W
  (RP2040) is **not supported**; it doesn't have enough memory for this
  firmware.
- A supported scale (for example A&D FX-i series, see [manuals/](manuals/)).
- A 2.4 GHz Wi-Fi network for the web interface (optional, but recommended).

**Recommended fine tube:** the
[Ultra Low Flow Fine Trickler Tube](https://github.com/eamars/OpenTrickler/blob/main/STL/OpenTrickler/Ultra%20Low%20Flow%20Fine%20Trickler%20Tube%20-%20Scaled%20to%2099.7%20Percent.stl)
from the original OpenTrickler repository. Its slow, even flow gives the
controller the most precision on the final approach. Other tubes work too,
but give each tube its own profile.

## Installing

1. Download the latest `OpenTrickler-Firmware-vX.Y.Z.uf2` from the
   [Releases page](https://github.com/Bleach3D/OpenTrickler-Autotune/releases/latest).
2. Disconnect motor power.
3. Hold the **BOOTSEL** button on the Pico 2 W and plug in the USB cable.
   Release the button once a USB drive named `RP2350` appears.
4. Copy the `.uf2` file onto that drive. The Pico restarts by itself once the
   copy is done.
5. Check the version under **Settings → Version** on the display.

## Updating

Once the firmware is installed, you can update over Wi-Fi:

**Web interface → Settings → Firmware Update → Check for Updates**

The controller downloads the newest release directly from GitHub, checks it
and installs it. If an update over Wi-Fi fails, install the `.uf2` from the
Releases page over USB as described above.

What changed in each version is listed in [CHANGELOG.md](CHANGELOG.md).
Read it before updating: some versions reset certain settings, and the
changelog says so.

## First setup

[QUICKSTART.md](QUICKSTART.md) walks through every step in detail. In short:

1. **Connect to Wi-Fi.** On first start the controller opens its own setup
   hotspot (`opentrickler-…`). Connect to it with your phone or laptop and
   enter your Wi-Fi details. Up to 5 networks can be saved; the controller
   joins the strongest one in range. Then open the controller's address
   (shown on the display) in a browser.
2. **Create a profile.** Keep **one profile per powder / tube / scale
   combination**, for example `Varget - ultra low flow tube`. A profile tuned
   for one setup doesn't carry over to another. Choose the controller at the
   top of the profile: **PID** (recommended) or the older **Adaptive**
   controller.
3. **Tune the profile** under Settings → AI / PID Tuning, or from the menu
   on the device:
   - **PID Tuning** (PID controller) runs a series of test throws, measures
     flow rate and scale delay, and sets the handoff and landing speeds.
   - **AI Tuning** (Adaptive controller) runs a characterization of how your
     powder flows, followed by a machine calibration of scale delay,
     settling time and powder that falls after the motor stops.
4. **Throw a few supervised test charges** and check them before relying on
   the profile.

Re-run the tuning after changing powder, tube, scale or the target weight
range.

## Everyday use

1. Select the profile that matches your current setup.
2. Enter the target weight.
3. Put the cup on the scale and wait for zero.
4. Let the controller throw and settle the charge. If Manual Finish is
   active, follow its prompts.
5. Remove the cup **when prompted**. The LED colour shows the result:
   green = in tolerance, yellow = under, red = over.
6. Put the empty cup back for the next charge.

The web interface shows each charge as a weight curve (Charge Timeline) and
keeps a throw history with mean and spread.

If the controller starts to be repeatedly under or over, re-run the tuning
rather than adjusting individual settings by hand.

## Features

- **Two controllers per profile**: the classic OpenTrickler PID controller
  (recommended) with automatic **PID Tuning**, or the Adaptive controller
  that learns a model of your powder's flow through characterization and
  machine calibration and refines it with normal charges.
- **Pre-charge** (needs a servo gate): drops the start of the next charge
  onto the closed gate while you handle the current cup.
- **Manual Finish** for coarse powders whose kernels are as large as your
  tolerance: the controller stops short and tells you how many kernels to
  add.
- **Reverse Tube**: briefly reverses a tube after it stops to prevent
  dribbling.
- **Adjustable tolerance** and an **accuracy vs. speed** slider per profile.
- **Web interface** with a Basic and an Advanced mode (Basic hides expert
  settings), Charge Timeline, throw history, and settings for motors, scale,
  LEDs and servo gate.
- **On-device menu**: start PID Tuning and AI Tuning from the encoder,
  save or discard the result on the device. The reset button goes back one
  page.
- **Profile export/import** as a JSON file for backups and for moving
  profiles between controllers.
- **Up to 5 Wi-Fi networks**, with a setup hotspot as a fallback.
- **Updates over Wi-Fi** directly from GitHub.

## Troubleshooting

- **Web interface not reachable:** check the address on the display. If no
  saved network is in range, the controller opens its setup hotspot
  (`opentrickler-…`) again.
- **Charges are consistently under or over:** make sure the right profile is
  selected and the scale is stable and zeroed, then re-run the tuning.
- **Something went wrong after an update:** install the previous version
  from the [Releases page](https://github.com/Bleach3D/OpenTrickler-Autotune/releases)
  over USB (see [Installing](#installing)).
- More help on scales and Wi-Fi: [manuals/](manuals/).

Bugs and questions:
[GitHub Issues](https://github.com/Bleach3D/OpenTrickler-Autotune/issues).

## Credits and license

This firmware builds on the work of:

- [Eamars/OpenTrickler](https://github.com/eamars/OpenTrickler): the original
  OpenTrickler hardware and firmware
- [WhoKilledBambiLabs/Opentrickler_ML](https://github.com/WhoKilledBambiLabs/Opentrickler_ML):
  AI characterization, machine calibration and runtime learning
  ([AI tuning reference](https://whokilledbambilabs.github.io/Opentrickler_ML/ai-tuning-reference.html))
- [thomaspember1990/OpenTrickler-Firmware](https://github.com/thomaspember1990/OpenTrickler-Firmware)
  (TP Custom Rifle Parts): per-profile settings, PID controller, Manual
  Finish, Reverse Tube

Licensed under GPL-3.0 like the original, see [LICENSE](LICENSE).
