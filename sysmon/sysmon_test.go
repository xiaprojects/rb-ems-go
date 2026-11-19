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

/*
Tests for rb-sysmon. Commands are run with bash like on the Pi, but only harmless
echo/exit commands are used: the shipped config is validated, never executed
(it needs vcgencmd and other Pi only tools).
*/

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"sync"
	"testing"
)

func TestParseValue(t *testing.T) {
	cases := []struct {
		in, typ string
		want    interface{}
		wantErr bool
	}{
		{"42", "int", int64(42), false},
		{"-3", "int", int64(-3), false},
		{"4.2", "int", nil, true},
		{"", "int", nil, true},
		{"51.5", "float", 51.5, false},
		{"0.8625V", "float", nil, true},
		{"up", "string", "up", false},
		{"raw", "", "raw", false},
		{"raw", "unknown", "raw", false},
	}
	for _, c := range cases {
		got, err := parseValue(c.in, c.typ)
		if (err != nil) != c.wantErr {
			t.Errorf("parseValue(%q, %q) err = %v, wantErr %v", c.in, c.typ, err, c.wantErr)
			continue
		}
		if !c.wantErr && got != c.want {
			t.Errorf("parseValue(%q, %q) = %#v, want %#v", c.in, c.typ, got, c.want)
		}
	}
}

func TestValuesEqual(t *testing.T) {
	cases := []struct {
		a, b interface{}
		want bool
	}{
		{nil, nil, true},
		{nil, int64(0), false},
		{int64(1), int64(1), true},
		{int64(1), int64(2), false},
		{float64(42), int64(42), true}, // value reloaded from the JSON state file
		{51.5, 51.5, true},
		{"a", "a", true},
	}
	for _, c := range cases {
		if got := valuesEqual(c.a, c.b); got != c.want {
			t.Errorf("valuesEqual(%#v, %#v) = %v, want %v", c.a, c.b, got, c.want)
		}
	}
}

func TestProcessSensor(t *testing.T) {
	st := &SensorState{}
	s := Sensor{Name: "cpu", Dev: "echo ' 42 '", Kind: "executable", ValueType: "int"}

	v, changed, err := processSensor(s, st)
	if err != nil || !changed || v != int64(42) {
		t.Fatalf("first read = %v %v %v, want 42 true nil", v, changed, err)
	}
	v, changed, err = processSensor(s, st)
	if err != nil || changed || v != nil {
		t.Errorf("unchanged read = %v %v %v, want nil false nil", v, changed, err)
	}
	s.Dev = "echo 43"
	if v, changed, _ = processSensor(s, st); !changed || v != int64(43) {
		t.Errorf("changed read = %v %v, want 43 true", v, changed)
	}

	for name, bad := range map[string]Sensor{
		"failing command": {Dev: "exit 1", Kind: "executable", ValueType: "int"},
		"unparsable":      {Dev: "echo abc", Kind: "executable", ValueType: "float"},
		"unknown kind":    {Dev: "echo 1", Kind: "spi", ValueType: "int"},
	} {
		st := &SensorState{}
		if _, changed, err := processSensor(bad, st); err == nil || changed || st.Valid {
			t.Errorf("%s: err=%v changed=%v valid=%v", name, err, changed, st.Valid)
		}
	}
}

type sysmonRecorder struct {
	mu     sync.Mutex
	posts  []map[string]interface{}
	status int
}

func (r *sysmonRecorder) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	r.mu.Lock()
	defer r.mu.Unlock()
	var body map[string]interface{}
	_ = json.NewDecoder(req.Body).Decode(&body)
	r.posts = append(r.posts, body)
	if r.status != 0 {
		w.WriteHeader(r.status)
	}
}

func TestExecuteSensorsPostsOnlyChanges(t *testing.T) {
	rec := &sysmonRecorder{}
	srv := httptest.NewServer(rec)
	defer srv.Close()

	stateFile := filepath.Join(t.TempDir(), "state.json")
	cfg := &Config{URL: srv.URL, Sensors: []Sensor{
		{Name: "a", Dev: "echo 1", Kind: "executable", ValueType: "int"},
		{Name: "b", Dev: "echo up", Kind: "executable", ValueType: "string"},
		{Name: "broken", Dev: "exit 3", Kind: "executable", ValueType: "int"},
	}}
	states := initSensorStates(cfg.Sensors)

	if err := executeSensors(cfg, states, stateFile); err != nil {
		t.Fatal(err)
	}
	if err := executeSensors(cfg, states, stateFile); err != nil { // nothing changed
		t.Fatal(err)
	}
	cfg.Sensors[1].Dev = "echo down"
	if err := executeSensors(cfg, states, stateFile); err != nil {
		t.Fatal(err)
	}

	if len(rec.posts) != 2 {
		t.Fatalf("posts = %v, want 2", rec.posts)
	}
	if len(rec.posts[0]) != 2 || rec.posts[0]["a"] != float64(1) || rec.posts[0]["b"] != "up" {
		t.Errorf("first post = %v", rec.posts[0])
	}
	if len(rec.posts[1]) != 1 || rec.posts[1]["b"] != "down" {
		t.Errorf("second post = %v, want only b=down", rec.posts[1])
	}

	// After a restart the persisted state suppresses the duplicate post
	restarted := initSensorStates(cfg.Sensors)
	loadSensorStates(restarted, stateFile)
	if !restarted["a"].Valid || restarted["broken"].Valid {
		t.Errorf("reloaded state a=%v broken=%v", restarted["a"].Valid, restarted["broken"].Valid)
	}
	if err := executeSensors(cfg, restarted, stateFile); err != nil {
		t.Fatal(err)
	}
	if len(rec.posts) != 2 {
		t.Errorf("post after restart with unchanged values: %v", rec.posts[2:])
	}
}

func TestPostDataErrors(t *testing.T) {
	rec := &sysmonRecorder{status: http.StatusBadRequest}
	srv := httptest.NewServer(rec)
	if err := postData(srv.URL, SensorData{"x": 1}); err == nil {
		t.Error("no error on HTTP 400")
	}
	srv.Close()
	if err := postData(srv.URL, SensorData{"x": 1}); err == nil {
		t.Error("no error with the server down")
	}
}

func TestLoadSensorStatesMissingOrCorrupt(t *testing.T) {
	dir := t.TempDir()
	states := initSensorStates([]Sensor{{Name: "a"}})
	loadSensorStates(states, filepath.Join(dir, "missing.json"))
	bad := filepath.Join(dir, "bad.json")
	_ = os.WriteFile(bad, []byte("{not json"), 0o644)
	loadSensorStates(states, bad)
	if states["a"].Valid {
		t.Error("state marked valid from a missing or corrupt file")
	}
}

func TestShippedConfig(t *testing.T) {
	cfg, err := loadConfig("rb-sysmon.json")
	if err != nil {
		t.Fatal(err)
	}
	if cfg.URL == "" || len(cfg.Sensors) == 0 {
		t.Fatal("missing url or sensors")
	}
	names := map[string]bool{}
	for _, s := range cfg.Sensors {
		if s.Name == "" || names[s.Name] {
			t.Errorf("sensor name %q empty or duplicated", s.Name)
		}
		names[s.Name] = true
		if s.Kind != "executable" {
			t.Errorf("%s: kind %q", s.Name, s.Kind)
		}
		if s.Dev == "" {
			t.Errorf("%s: empty command", s.Name)
		}
		switch s.ValueType {
		case "int", "float", "string":
		default:
			t.Errorf("%s: value_type %q", s.Name, s.ValueType)
		}
	}
	if _, err := loadConfig("does-not-exist.json"); err == nil {
		t.Error("loadConfig accepted a missing file")
	}
}
