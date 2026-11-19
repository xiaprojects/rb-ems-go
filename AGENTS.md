# AGENTS.md — rb-ems-go (RB-04 Engine Monitoring System)

Guidance for AI coding agents (and humans) working in this repository. This is the
single source of truth; `CLAUDE.md` just points here.

## Project overview

`github.com/xiaprojects/rb-ems-go` is the **RB-04 Engine Monitoring System (EMS)**
component of the open-source **RB avionics** suite. It runs on a **Raspberry Pi 5
(ARM64)** with an 8" HDMI display and consists of several small Go daemons that poll
engine and aircraft sensors over **SPI / I2C / GPIO** and push the readings as JSON to
the **RB-01** display unit through a REST endpoint (default `/setEMS`).

Functions covered: engine RPM, temperatures (EGT/CHT/OAT), discrete GPIO state, and
analog (ADC) channels such as oil pressure/temperature and battery voltage. A system
monitor reports Raspberry Pi health.

> ⚠️ **Safety-critical avionics.** Be conservative. Do not change sensor decoding math,
> scaling factors, or SPI/I2C/GPIO framing without explicit confirmation from the user.

## Repository layout

This is a **multi-module monorepo**: each subdirectory is its own `main` package with
its **own `go.mod`**, and all of them declare the *same* module path
`github.com/xiaprojects/rb-ems-go`. Build and run each component from its own directory.

| Dir | Binary | Sensor tech | Entry point | Purpose |
|-----|--------|-------------|-------------|---------|
| `main/` | `ems-chtegt` | SPI thermocouple / RTD (MAX31855, MAX31856, MAX31865) | `main/main.go` → `ems_egt_cht_sample_spi()` in `main/egt.go` | EGT / CHT / OAT temperatures |
| `rpm/` | `ems-rpm` | GPIO pulse counting (`go-gpiocdev`, rising-edge IRQ + EMA smoothing) | `rpm/rpm.go` `main()` | Engine RPM |
| `adc/` | `ems-adc` | I2C ADS1115 ADC | `adc/main.go` → `ems_adc_sample_i2c()` in `adc/adc.go` | Oil pressure/temperature, battery voltage |
| `gpio/` | `ems-gpio` | Discrete GPIO read (`go-gpiocdev`) | `gpio/main.go` → `ems_gpio()` in `gpio/gpio.go` | Gear position switches |
| `sysmon/` | `rb-sysmon` | Shell commands (`kind:"executable"`) | `sysmon/sysmon.go` `main()` | Pi health: CPU/GPU temp, PMIC voltages, network, RAM |

## Architecture & shared pattern

Every component follows the same shape:

1. Read a JSON config file (`url` + `sensors[]` array).
2. Run an infinite poll loop.
3. Dispatch per-sensor on the `kind` field to the matching driver.
4. Apply linear scaling: `value = raw * r0 + r0Offset` (where present).
5. Change-detection — only POST when a reading changes (or after a max gap).
6. `http.POST` a JSON map to `url`, the RB-01 `/setEMS` REST API.

Each sensor's `name` becomes the **JSON payload key** sent to RB-01. The POST helper is
named `RBEMSPostData` and is duplicated in each module (intentionally — modules are
independent).

## Build / run / deploy

Target platform is **Linux / ARM64 on a Raspberry Pi**. From a Windows/macOS dev box,
cross-compile:

```sh
GOOS=linux GOARCH=arm64 go build
```

**`ems-chtegt` (root Makefile):**

```sh
make                 # go build -o ems-chtegt ./main/
make install         # deploy to /opt/stratux/bin, install systemd unit, copy ems.version
make clean
```

`make install` copies the binary to `/opt/stratux/bin`, installs
`debian/ems-chtegt.service`, enables it via systemd, and copies `ota/ems.version`.

**Other components** are built per-directory and each ships its own `*.service` systemd
unit (runs as user `pi`, `Restart=always`, logs to syslog):

```sh
cd rpm    && go build      # ems-rpm    (+ rpm/ems-rpm.service)
cd adc    && go build      # ems-adc    (+ adc/ems-adc.service)
cd gpio   && go build      # ems-gpio   (+ gpio/ems-gpio.service)
cd sysmon && go build      # rb-sysmon  (+ sysmon/rb-sysmon.service)
```

**Run locally on the Pi** (each takes its config path):

```sh
./ems-chtegt ems-chtegt.json
./ems-rpm    rpm/ems-rpm.json          # also accepts flags: -ppr -window -alpha ...
./ems-adc    adc/ems-adc.json
./ems-gpio   gpio/ems-gpio.json
./rb-sysmon  -config sysmon/rb-sysmon.json -wait 10 -state ""
```

Config files live beside each module: `main` uses the root `ems-chtegt.json`; the others
use `<dir>/ems-*.json` / `rb-sysmon.json`. The Stratux deploy layout puts configs under
`/opt/stratux/www/settings/`.

## Build & test automation

The root `Makefile` drives every module (CI runs `make ci` in `.github/workflows/ci.yml`):

```sh
make test         # unit tests of every module (-race)
make vet          # go vet, native and linux/arm64
make fmt-check    # gofmt
make check-dup    # main/, adc/, gpio/ alert.go + alert_test.go must stay identical
make build-arm64  # all daemons for the Pi 5 -> dist/linux-arm64/
make package      # + dist/rb-ems-<ota/ems.version>-linux-arm64.tgz (bin, systemd, settings)
make ci           # fmt-check check-dup vet test build-arm64
```

Tests are hardware-free (`*_test.go` beside each module). SPI/I2C are replaced by fakes
of `spi.Conn` / `i2c.Bus` so the real driver functions run unchanged; RB-01 is an
`httptest` server. Decoder expectations come from the datasheet tables (MAX31855,
MAX31856, MAX31865, ADS1115), so a change in decoding math or register framing fails a
test — update the test only when the datasheet agrees. `TestShippedConfig` in each
module checks the JSON config files beside it only use values the driver accepts.
When you edit `alert.go` or `alert_test.go`, copy the file to all three modules.

## Conventions & constraints

- **Hardware-only execution.** `periph.io` (SPI/I2C) and `go-gpiocdev` (GPIO) only
  function on the Pi. On a dev machine you can build, vet and run the unit tests, but you
  **cannot run** the daemons or test against real sensors.
- **Preserve license headers.** Code is **AGPL-3.0**, © XIAPROJECTS SRL, with dual
  commercial licensing offered. Keep the existing header block on source files.
- Some code comments are in **Italian** — that is expected, not a bug.

## Probe failure alerts (RB-01 `POST /alert`)

`main/`, `adc/` and `gpio/` push an `EVENT_TYPE_EMS_GENERIC_FAILURE` (type `13`) alert to
RB-01 when a probe fails to read `alertAfterFailures` times in a row (bus/CS error, chip
not responding, thermocouple fault, I2C error, conversion timeout, GPIO line error). The
alert title is `"<title> probe failure"`. On the first good read the alert is ended with
`{"Id": n, "Active": false}`. No keep-alive is sent while the fault lasts. The logic
lives in `alert.go`, duplicated in each module like `RBEMSPostData`.

Config fields (top level): `alertAfterFailures` (0 or missing = alerts disabled),
`alertUrl` (missing = `url` with the path replaced by `/alert`). Per sensor: `title`
(the name shown in the alert, missing = `name`). `rpm/` and `sysmon/` raise no alerts.

## Sensor `kind` reference

Per-component supported `kind` values and key config fields.

**`main/` (SPI temperatures)** — fields: `name`, `dev` (e.g. `/dev/spidev0.0`), `kind`,
`gpio` (>0 = manual chip-select on `GPIO{n}`, 0 = kernel CS), `spiMode` (0–3),
`r0`, `r0Offset`; top-level `busSpeed` (Hz). Supported `kind`:
- `MAX31855` — K-type thermocouple
- `MAX31856` / `MAX31856K` — K-type · `MAX31856J` — J-type · `MAX31856T` — T-type
- `MAX31865` — RTD (PT100)

**`rpm/`** — `kind: "gpiopulse"`; fields: `name`, `dev` (gpiochip, e.g.
`/dev/gpiochip0`), `gpio`. CLI flags tune `-ppr`, `-window`, `-alpha` (EMA), `-mindelta`,
`-maxgap`, `-debounce`.

**`adc/`** — `kind: "ADS1115"`; fields: `name`, `dev` (i2c bus, `0` or `i2c-1`), `addr`
(e.g. `0x48`), `channel` (0–3), `channel_neg` (optional; set it for a differential
`channel`−`channel_neg` reading, pairs 0-1/0-3/1-3/2-3 only; differential readings are
signed and not clamped at 0), `gain_mv` (6144/4096/2048/1024/512/256),
`dr_sps` (8/16/32/64/128/250/475/860), `r0`, `r0Offset`, `value_type` (`float`/`int`),
`average_level` (0 = none).

**`gpio/`** — `kind: "GPIO"`; fields: `name`, `dev`, `channel` (GPIO line offset). Emits
`0`/`1`.

**`sysmon/`** — `kind: "executable"`; fields: `name`, `dev` (a shell command whose stdout
is the value), `value_type` (`int`/`float`/`string`). Used for CPU/GPU temp, PMIC
voltages, network state, RAM, uptime, etc.
