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
	"net/http"
	"net/http/httptest"
	"os"
	"testing"
)

// The shipped ems-gpio.json shall only use values the driver accepts
func TestShippedConfig(t *testing.T) {
	var cfg struct {
		Url                string `json:"url"`
		AlertAfterFailures int    `json:"alertAfterFailures"`
		Sensors            []struct {
			Name    string `json:"name"`
			Kind    string `json:"kind"`
			Channel int    `json:"channel"`
		} `json:"sensors"`
	}
	data, err := os.ReadFile("ems-gpio.json")
	if err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(data, &cfg); err != nil {
		t.Fatalf("ems-gpio.json: %v", err)
	}
	if cfg.Url == "" || len(cfg.Sensors) == 0 {
		t.Fatal("missing url or sensors")
	}
	names := map[string]bool{}
	lines := map[int]string{}
	for _, s := range cfg.Sensors {
		if s.Name == "" || names[s.Name] {
			t.Errorf("sensor name %q empty or duplicated", s.Name)
		}
		names[s.Name] = true
		if s.Kind != "GPIO" {
			t.Errorf("%s: kind %q", s.Name, s.Kind)
		}
		if s.Channel < 0 || s.Channel > 27 {
			t.Errorf("%s: channel %d outside the 40 pin header (0..27)", s.Name, s.Channel)
		}
		if other, dup := lines[s.Channel]; dup {
			t.Errorf("%s and %s share GPIO line %d", other, s.Name, s.Channel)
		}
		lines[s.Channel] = s.Name
	}
}

func TestRBEMSPostData(t *testing.T) {
	var got map[string]int
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		_ = json.NewDecoder(r.Body).Decode(&got)
	}))
	defer srv.Close()
	if rc := RBEMSPostData(srv.URL, map[string]int{"gearnose": 1}); rc != 0 || got["gearnose"] != 1 {
		t.Errorf("rc=%d payload=%v", rc, got)
	}
	srv.Close()
	if rc := RBEMSPostData(srv.URL, map[string]int{"x": 1}); rc != 2 {
		t.Errorf("rc with server down = %d, want 2", rc)
	}
}
