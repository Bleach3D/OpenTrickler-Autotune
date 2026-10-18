# Building and releasing

Notes for maintainers. Users: see [README.md](README.md) and
[QUICKSTART.md](QUICKSTART.md).

## Build

Requirements: Git, CMake, Ninja, the ARM GCC toolchain (`arm-none-eabi`),
Python 3, and a native C/C++ compiler (the Pico SDK builds `pioasm` and
`picotool` for the host).

```sh
git clone --recursive https://github.com/Bleach3D/OpenTrickler-Autotune.git
cd OpenTrickler-Autotune
cmake -B build-pico2w-release -G Ninja -DCMAKE_BUILD_TYPE=Release -DPICO_BOARD=pico2_w
cmake --build build-pico2w-release
```

In an existing clone, fetch the submodules with
`git submodule update --init --recursive`.

Outputs:

- `build-pico2w-release/app.uf2`: install over USB (BOOTSEL drive `RP2350`)
- `build-pico2w-release/app.bin`: OTA image, e.g.
  `python tools/ota_upload.py --host <device-ip> --bin build-pico2w-release/app.bin --apply`
- `build-pico2w-release/app.elf`: debug symbols

Only the Pico 2 W (RP2350) is supported; configuring with
`-DPICO_BOARD=pico_w` is rejected on purpose because the firmware does not
fit the RP2040's RAM.

The web interface is `src/html/web_portal.html`; the build turns it into
`src/generated/web_portal.html.h`.

## Release

Versions follow [Semantic Versioning](https://semver.org/): `MAJOR` for
incompatible changes (e.g. an EEPROM layout change that resets settings),
`MINOR` for new features, `PATCH` for bug fixes.

1. Move the `[Unreleased]` entries in [CHANGELOG.md](CHANGELOG.md) under a
   new version heading.
2. Put the same version (e.g. `v1.3.0`) into `RELEASE_VERSION` and commit.
3. Tag the commit and push branch and tag:
   ```sh
   git tag -a v1.3.0 -m "v1.3.0"
   git push origin main
   git push origin v1.3.0
   ```
4. On GitHub, create a release from that tag, named **exactly** like the
   tag, and publish it.

Publishing triggers the *publish-update-manifest* workflow
(`.github/workflows/publish-update-manifest.yml`). It builds the firmware and
attaches to the release:

- `OpenTrickler-Firmware-<tag>.uf2` for USB installs
- `app.bin` and `manifest.json`, which devices fetch from
  `releases/latest/download/` when the user clicks *Check for Updates*

The workflow stops if the tag and `RELEASE_VERSION` differ, because devices
compare the version strings literally. Pushing a tag alone does not publish
an update; only a published release does.
