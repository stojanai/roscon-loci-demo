# Building the demo locally

The repo intentionally does **not** contain `ardupilot/`, `toolchain/`, or
`venv/` (they're huge / vendored — see `.gitignore`). This guide recreates them.
Tested on macOS 15 (Apple Silicon) with GCC-ARM 15.3; Linux notes inline.

You do **not** need to build to run LOCI — the pre-built firmware is already in
`artifacts/` (see [Just run LOCI](#just-run-loci-no-build)). Build only if you
want to regenerate the ELFs or run the SITL flight demo.

## 0. Prerequisites

- `git`, `python3` (3.9+), and a shell
- **LOCI**: the `loci` CLI plus `jq` and `uv` on PATH
  (`brew install jq uv` on macOS)
- ~5 GB free disk (ArduPilot checkout + toolchain)

## 1. Clone ArduPilot (Copter-4.6.2) into the repo

```bash
cd roscon-loci-demo
git clone --depth 1 --branch Copter-4.6.2 https://github.com/ArduPilot/ardupilot.git
cd ardupilot && git submodule update --init --recursive --depth 1 && cd ..
```

## 2. Python build/SITL environment

```bash
python3 -m venv venv
./venv/bin/pip install --upgrade pip
./venv/bin/pip install empy==3.3.4 pexpect future pymavlink MAVProxy dronecan \
    gnureadline wxPython pillow matplotlib opencv-python
```

(`gnureadline wxPython pillow matplotlib opencv-python` are only needed for the
SITL MAVProxy console + map; skip them if you only build firmware.)

## 3. ARM bare-metal toolchain (no sudo)

The CubeOrange target needs `arm-none-eabi-gcc`. Download ARM's official tarball
and extract into `./toolchain/` (Homebrew's cask needs an interactive sudo, so
the tarball is easier for scripts):

```bash
# macOS Apple Silicon:
URL="https://gitlab.arm.com/api/v4/projects/tooling%2Fgnu-toolchains-for-arm/packages/generic/gnu-toolchain/15.3.rel1/arm-gnu-toolchain-15.3.rel1-darwin-arm64-arm-none-eabi.tar.xz"
# Linux x86_64: swap darwin-arm64 -> x86_64-arm  (…-15.3.rel1-x86_64-arm-none-eabi.tar.xz)
curl -sL -o toolchain.tar.xz "$URL"
mkdir -p toolchain && tar -xf toolchain.tar.xz -C toolchain --strip-components 1 && rm toolchain.tar.xz
./toolchain/bin/arm-none-eabi-gcc --version   # sanity check
```

## 4. Build the firmware

`scripts/build_fw.sh` handles configure + recompile + copies the ELF into
`artifacts/`. It reads the toolchain from `./toolchain/bin` and the venv.

```bash
./scripts/build_fw.sh baseline   # clean sprayer  -> artifacts/arducopter-baseline.elf
./scripts/build_fw.sh case1      # stack bug      -> artifacts/arducopter-bugged.elf
./scripts/build_fw.sh case2      # timing bug     -> artifacts/arducopter-case2-drift.elf
```

Each run leaves the source tree clean (it applies the patch, builds, reverts).
**Never run git commands on `ardupilot/` while a build is in flight** — waf reads
the working copy as it compiles.

## 5. (Optional) Run the SITL flight demo

```bash
./scripts/run_sitl.sh
```

MAVProxy console + map windows open (behind the terminal on macOS — Cmd+Tab).
Then at the `MAV>` prompt:

```
mode guided
arm throttle
takeoff 15
long DO_SPRAYER 1
graph SERVO_OUTPUT_RAW.servo10_raw
```

Left-click a distant point on the map and `guided 15`. Baseline: pump PWM
(servo10) tracks ground speed. Case-1 build: pump stays pinned regardless of speed.

## Just run LOCI (no build)

The pre-built ELFs ship in `artifacts/`. Open a Claude Code session **inside
`ardupilot/`** (LOCI activates per project dir) and ask for analysis, or drive
the CLI directly — e.g. stack + timing on the stack-bug build:

```bash
loci elf stack --elf artifacts/arducopter-bugged.elf --arch armv7e-m \
    --entry-functions _ZN10AC_Sprayer6updateEv --out-dir .loci-build/elf/stk
loci elf asm --elf artifacts/arducopter-case2-drift.elf \
    --functions "_ZNK10AC_Sprayer25drift_compensation_factorEf" --arch armv7e-m
```

CubeOrange = STM32H7 (Cortex-M7) → LOCI target `armv7e-m`. See `README.md` for
the measured results and the demo narrative.
