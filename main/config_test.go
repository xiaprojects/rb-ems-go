/*
This file is part of RB.

# Copyright (C) 2023 XIAPROJECTS SRL

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as published
by the Free Software Foundation, version 3.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program. If not, see <https://www.gnu.org/licenses/>.

This source is part of the project RB:
01 -> Display with Synthetic vision, Autopilot and ADSB
02 -> Display with SixPack
03 -> Display with Autopilot, ADSB, Radio, Flight Computer
04 -> Display with EMS: Engine monitoring system
05 -> Display with Stratux BLE Traffic
06 -> Display with Android 6.25" 7" 8" 10" 10.2"
07 -> Display with Stratux BLE Traffic composed by RB-05 + RB-03 in the same box

Community edition will be free for all builders and personal use as defined by the licensing model
Dual licensing for commercial agreement is available
Please join Discord community
*/
package main

import (
	"encoding/json"
	"net/url"
	"os"
	"strconv"
	"strings"
	"testing"
)

// The shipped ems-chtegt.json shall only use values the driver accepts
func TestShippedConfig(t *testing.T) {
	var cfg struct {
		Url                string `json:"url"`
		AlertUrl           string `json:"alertUrl"`
		AlertAfterFailures int    `json:"alertAfterFailures"`
		Speed              int64  `json:"busSpeed"`
		Sensors            []struct {
			Name    string  `json:"name"`
			Dev     string  `json:"dev"`
			Kind    string  `json:"kind"`
			Gpio    int     `json:"gpio"`
			SpiMode int     `json:"spiMode"`
			R0      float64 `json:"r0"`
		} `json:"sensors"`
	}
	data, err := os.ReadFile("../ems-chtegt.json")
	if err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(data, &cfg); err != nil {
		t.Fatalf("ems-chtegt.json: %v", err)
	}
	for _, u := range []string{cfg.Url, cfg.AlertUrl} {
		if u == "" {
			continue
		}
		if p, err := url.Parse(u); err != nil || p.Host == "" {
			t.Errorf("invalid url %q", u)
		}
	}
	if cfg.Url == "" || len(cfg.Sensors) == 0 {
		t.Fatal("missing url or sensors")
	}
	if cfg.AlertAfterFailures < 0 || cfg.Speed < 0 {
		t.Errorf("alertAfterFailures=%d busSpeed=%d", cfg.AlertAfterFailures, cfg.Speed)
	}
	kinds := map[string]bool{"MAX31855": true, "MAX31856": true, "MAX31856K": true,
		"MAX31856J": true, "MAX31856T": true, "MAX31865": true}
	names := map[string]bool{}
	csPins := map[string]string{}
	for _, s := range cfg.Sensors {
		if s.Name == "" || names[s.Name] {
			t.Errorf("sensor name %q empty or duplicated", s.Name)
		}
		names[s.Name] = true
		if !kinds[s.Kind] {
			t.Errorf("%s: unsupported kind %q", s.Name, s.Kind)
		}
		if !strings.HasPrefix(s.Dev, "/dev/spidev") {
			t.Errorf("%s: dev %q is not a spidev node", s.Name, s.Dev)
		}
		if s.SpiMode < 0 || s.SpiMode > 3 {
			t.Errorf("%s: spiMode %d", s.Name, s.SpiMode)
		}
		if s.Gpio < 0 || s.Gpio > 27 {
			t.Errorf("%s: gpio %d outside the 40 pin header (0..27)", s.Name, s.Gpio)
		}
		if s.R0 == 0 {
			t.Errorf("%s: r0 = 0 would always report 0", s.Name)
		}
		// Two probes on the same manual chip select would talk over each other
		if s.Gpio > 0 {
			key := strconv.Itoa(s.Gpio) // one line drives every chip wired to it, whatever the bus
			if other, dup := csPins[key]; dup {
				t.Errorf("%s and %s share chip select GPIO%d", other, s.Name, s.Gpio)
			}
			csPins[key] = s.Name
		}
	}
}
