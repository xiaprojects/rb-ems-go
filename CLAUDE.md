# CLAUDE.md

Full project guidance lives in **[AGENTS.md](AGENTS.md)** — read it first.

Highest-value reminders:

- **Safety-critical avionics (RB-04 EMS).** Be conservative; don't alter sensor decoding
  math, scaling, or SPI/I2C/GPIO framing without explicit confirmation.
- **Hardware-only execution.** Targets Raspberry Pi 5 / ARM64. On a dev box you can
  `go build` / `go vet` but cannot run the daemons (no SPI/I2C/GPIO, no test suite).
  Cross-compile with `GOOS=linux GOARCH=arm64 go build`.
- **Multi-module monorepo.** Each of `main/`, `rpm/`, `adc/`, `gpio/`, `sysmon/` has its
  own `go.mod` — build each from its own directory (root `make` builds only `ems-chtegt`).
- **Preserve the AGPL-3.0 license headers** (© XIAPROJECTS SRL).
