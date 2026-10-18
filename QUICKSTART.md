# Quick Start — OpenTrickler Autotune

This guide takes a Pico 2 W OpenTrickler from installing the firmware to your
first charges. It assumes the hardware (trickler, motors, scale interface,
128x64 display with encoder) is already built and wired according to the
[original OpenTrickler documentation](https://github.com/eamars/OpenTrickler).

> This firmware runs reloading equipment. Check every charge on a calibrated
> scale, stay with the machine while the motors are running, and keep a
> known-good `.uf2` file so you can roll back.

## 1. Install the firmware

1. Download the latest `OpenTrickler-Firmware-vX.Y.Z.uf2` from the
   [Releases page](https://github.com/Bleach3D/OpenTrickler-Autotune/releases/latest).
2. Disconnect motor power.
3. Hold the **BOOTSEL** button on the Pico 2 W and plug in the USB cable.
   Release the button once a USB drive named `RP2350` appears.
4. Copy the `.uf2` file onto that drive. The Pico restarts by itself.
5. The display shows the boot animation (any button skips it) and then the
   main menu.

Updating from an older version? Check [CHANGELOG.md](CHANGELOG.md) first.
If a version resets settings, the changelog says so.

## 2. Connect to Wi-Fi

1. On first start the controller opens its own setup hotspot named
   `opentrickler-…`. Connect to it with your phone or laptop.
2. Open the web interface and enter your 2.4 GHz Wi-Fi name and password
   under **Settings → Wireless**. You can save up to 5 networks; the controller
   joins the strongest one in range and falls back to the setup hotspot if
   none is available.
3. After it has joined your network, open its address (shown on the display,
   or in your router's client list) in a browser.

## 3. Create a profile

Keep **one profile per powder / tube / scale combination**. A profile tuned
on one tube or powder does not carry over to another, and the controller
cannot detect which tube is installed.

1. Go to **Settings → Profile** and pick an empty slot.
2. Name it for the setup, for example `Varget - ultra low flow tube`.
3. Choose the **Controller** at the top of the profile:
   - **PID - Original OpenTrickler Firmware** (recommended): stops at the
     thresholds you set in the profile; tuned with PID Tuning (step 4).
   - **Adaptive - Based on Opentrickler_ML Firmware**: older controller that
     calculates its own stop margins from the AI model; tuned with AI Tuning
     (step 4). Not recommended for normal use.
4. Set the **Accepted Charge Tolerance**: the ± band that counts as a good
   charge (green). Typical 0.02–0.04 gn.
5. Save the profile.

## 4. Tune the profile

The tuning menu entry is called **AI / PID Tuning** and opens the tool that
matches the profile's controller. You can also start it from the menu on
the device and save or discard the result there.

**PID Tuning** (PID controller): runs a series of coarse and fine test
throws, measures flow rate and scale delay, and sets the handoff and landing
speeds. Enter your target weight and cup capacity, keep the cup on the scale
and don't touch the machine while it runs. Save the result when it is done.

**AI Tuning** (Adaptive controller):

1. **Characterization** measures how your powder flows through the coarse
   and fine tubes. Select the profile, enter your normal target weight, make
   sure the scale is stable and zeroed, and start it. Follow the remove /
   return cup prompts until it reports `ready_to_save`, then save it.
2. **Machine calibration** (afterwards) measures scale delay, settling time
   and the powder that still falls after a motor stops. Run it with the same
   profile and target weight and save the result.

Don't keep a tuning result if the scale was unstable, powder bridged in the
tube, the cup prompts were interrupted, or the wrong profile was selected.

Re-run the tuning after changing powder, tube, scale or target range.

Then throw a few supervised test charges and check them before you rely on
the profile.

## 5. Everyday use

1. Select the profile for your current setup and enter the target weight.
2. Put the cup on the scale and wait for zero.
3. Let the controller throw and settle the charge.
4. Remove the cup **when prompted**. The LED colour shows the result:
   green = in tolerance, yellow = under, red = over.
5. Put the empty cup back for the next charge.

The web interface shows each charge as a weight curve (**Charge Timeline**)
and keeps a **Throw History** with mean and spread.

If charges become repeatedly under or over, re-run the tuning rather than
adjusting individual settings by hand.

## 6. Optional features

All of these are in the profile's charge settings.

**Pre-charge** (needs a servo gate): while you handle the finished cup, the
coarse tube drops the start of the next charge onto the closed gate. The next
charge starts with that powder already in place. With pre-charge enabled,
a slight undercharge is not topped up automatically (that would empty the
gate into the cup); it is flagged yellow instead.

**Manual Finish**: for coarse stick powders whose single kernel is about as
heavy as your tolerance. Enter the weight of a cut kernel (for example
0.04 gn). The controller stops short, tells you how many cut kernels to add,
and checks the result.

**Reverse Tube**: if a little powder keeps falling after a motor stops,
let that tube reverse briefly after its final stop. Start small (about
0.05–0.3 revolutions coarse, 0.02–0.15 fine) and only increase if the
dribble continues. Too much can starve the start of the next charge.

**Accuracy vs. speed**: a slider per profile that trades charge time for
accuracy.

## 7. Back up your profiles

**Export Profile** on the Profile page (one profile) or **Export Config** in the settings menu (all slots)
downloads a JSON file with the profile settings and kernel weight. Use
**Import** to restore it after a reset or move it to another controller.
Worth doing once you have a profile you trust.

## 8. Update the firmware

**Settings → Firmware Update → Check for Updates** shows your version and
the newest release. Click **Update Now** to download and install it over
Wi-Fi. Nothing updates automatically, so the firmware never changes while
you're working.

If an update over Wi-Fi fails, install the `.uf2` from the Releases page over
USB as in step 1. That also works for going back to an older version.

## Getting help

- What changed in each version: [CHANGELOG.md](CHANGELOG.md)
- Scale and Wi-Fi setup notes: [manuals/](manuals/)
- Details of the AI tuning:
  [AI tuning reference](https://whokilledbambilabs.github.io/Opentrickler_ML/ai-tuning-reference.html)
- Bugs and questions:
  [GitHub Issues](https://github.com/Bleach3D/OpenTrickler-Autotune/issues)
