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
08 -> Voice Recognition Box with LLM and Natural speaking and Voice Recorder

Community edition will be free for all builders and personal use as defined by the licensing model
Dual licensing for commercial agreement is available
Please join Discord community
*/
package main

/*
Hardware-free tests for the ADS1115 driver.
Register values come from the ADS1115 datasheet (Table 8, Config register);
the I2C bus is replaced by fakeADS1115 so readADS1115 runs unchanged.
*/

import (
	"encoding/json"
	"errors"
	"math"
	"net/http"
	"net/http/httptest"
	"os"
	"strconv"
	"testing"
	"time"

	"periph.io/x/conn/v3/i2c"
)

func TestADS1115GainTable(t *testing.T) {
	cases := []struct {
		mV   int
		bits uint16
		lsb  float64 // V per count = FSR / 32768
	}{
		{6144, 0x0000, 6.144 / 32768},
		{4096, 0x0200, 4.096 / 32768},
		{2048, 0x0400, 2.048 / 32768},
		{1024, 0x0600, 1.024 / 32768},
		{512, 0x0800, 0.512 / 32768},
		{256, 0x0A00, 0.256 / 32768},
	}
	for _, c := range cases {
		bits, lsb, err := ads1115Gain(c.mV)
		if err != nil || bits != c.bits || math.Abs(lsb-c.lsb) > 1e-12 {
			t.Errorf("ads1115Gain(%d) = 0x%04X, %g, %v; want 0x%04X, %g", c.mV, bits, lsb, err, c.bits, c.lsb)
		}
	}
	for _, bad := range []int{0, 5000, 128, -1} {
		if _, _, err := ads1115Gain(bad); err == nil {
			t.Errorf("ads1115Gain(%d) accepted", bad)
		}
	}
}

func TestADS1115DataRateTable(t *testing.T) {
	cases := []struct {
		sps  int
		bits uint16
	}{
		{8, 0x0000}, {16, 0x0020}, {32, 0x0040}, {64, 0x0060},
		{128, 0x0080}, {250, 0x00A0}, {475, 0x00C0}, {860, 0x00E0},
	}
	for _, c := range cases {
		bits, conv, err := ads1115DataRate(c.sps)
		if err != nil || bits != c.bits {
			t.Errorf("ads1115DataRate(%d) = 0x%04X, %v; want 0x%04X", c.sps, bits, err, c.bits)
		}
		// Wait shall cover one conversion period (rounded to the ms)
		period := time.Second / time.Duration(c.sps)
		if conv < period-time.Millisecond {
			t.Errorf("ads1115DataRate(%d) wait %v shorter than conversion period %v", c.sps, conv, period)
		}
	}
	for _, bad := range []int{0, 100, 1000} {
		if _, _, err := ads1115DataRate(bad); err == nil {
			t.Errorf("ads1115DataRate(%d) accepted", bad)
		}
	}
}

func TestADS1115Mux(t *testing.T) {
	for ch, want := range map[int]uint16{0: 0x4000, 1: 0x5000, 2: 0x6000, 3: 0x7000} {
		if got, err := ads1115MuxSingleEnded(ch); err != nil || got != want {
			t.Errorf("single-ended %d = 0x%04X, %v; want 0x%04X", ch, got, err, want)
		}
	}
	for _, bad := range []int{-1, 4} {
		if _, err := ads1115MuxSingleEnded(bad); err == nil {
			t.Errorf("single-ended %d accepted", bad)
		}
	}
	diff := map[[2]int]uint16{{0, 1}: 0x0000, {0, 3}: 0x1000, {1, 3}: 0x2000, {2, 3}: 0x3000}
	for p, want := range diff {
		if got, err := ads1115MuxDifferential(p[0], p[1]); err != nil || got != want {
			t.Errorf("differential %d-%d = 0x%04X, %v; want 0x%04X", p[0], p[1], got, err, want)
		}
	}
	for _, bad := range [][2]int{{1, 0}, {0, 2}, {1, 2}, {3, 2}, {0, 0}} {
		if _, err := ads1115MuxDifferential(bad[0], bad[1]); err == nil {
			t.Errorf("differential %d-%d accepted", bad[0], bad[1])
		}
	}
}

/*************** fake I2C ****************/

type fakeADS1115 struct {
	i2c.Bus
	addr      uint16
	config    uint16
	conv      int16
	neverDone bool
	failTx    bool
	writes    [][]byte
}

func (f *fakeADS1115) Tx(addr uint16, w, r []byte) error {
	if f.failTx {
		return errors.New("i2c: remote I/O error")
	}
	if addr != f.addr {
		return errors.New("i2c: no ACK")
	}
	if len(w) == 3 { // register write
		f.writes = append(f.writes, append([]byte(nil), w...))
		if w[0] == 0x01 {
			f.config = uint16(w[1])<<8 | uint16(w[2])
		}
		return nil
	}
	var v uint16
	switch w[0] {
	case 0x00:
		v = uint16(f.conv)
	case 0x01:
		v = f.config &^ 0x8000 // OS reads 0 while converting
		if !f.neverDone {
			v |= 0x8000
		}
	}
	r[0], r[1] = byte(v>>8), byte(v)
	return nil
}

func intPtr(v int) *int { return &v }

func TestReadADS1115(t *testing.T) {
	cases := []struct {
		name    string
		opt     ADS1115Opts
		conv    int16
		wantCfg uint16
		want    float64
	}{
		{"AIN0 4.096V full scale", ADS1115Opts{Addr: 0x48, Channel: 0, GainFSR: 4096, DRSPS: 860, Clamp0: true},
			0x7FFF, 0xC3E3, 32767 * 125e-6},
		{"AIN3 0.512V mid", ADS1115Opts{Addr: 0x48, Channel: 3, GainFSR: 512, DRSPS: 475, Clamp0: true},
			16384, 0xF9C3, 0.256},
		{"single-ended negative clamped", ADS1115Opts{Addr: 0x48, Channel: 1, GainFSR: 6144, DRSPS: 860, Clamp0: true},
			-10, 0xD1E3, 0},
		{"single-ended negative not clamped", ADS1115Opts{Addr: 0x48, Channel: 1, GainFSR: 6144, DRSPS: 860},
			-16, 0xD1E3, -16 * 187.5e-6},
		{"differential 0-1 signed", ADS1115Opts{Addr: 0x49, Channel: 0, ChannelNeg: intPtr(1), GainFSR: 2048, DRSPS: 860},
			-32768, 0x85E3, -2.048},
		{"differential 2-3", ADS1115Opts{Addr: 0x49, Channel: 2, ChannelNeg: intPtr(3), GainFSR: 256, DRSPS: 860},
			100, 0xBBE3, 100 * 7.8125e-6},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			bus := &fakeADS1115{addr: uint16(c.opt.Addr), conv: c.conv}
			got, err := readADS1115(bus, c.opt)
			if err != nil {
				t.Fatal(err)
			}
			if bus.config != c.wantCfg {
				t.Errorf("config register = 0x%04X, want 0x%04X", bus.config, c.wantCfg)
			}
			if math.Abs(got-c.want) > 1e-9 {
				t.Errorf("value = %v, want %v", got, c.want)
			}
		})
	}
}

func TestReadADS1115Errors(t *testing.T) {
	good := ADS1115Opts{Addr: 0x48, Channel: 0, GainFSR: 4096, DRSPS: 860}

	if _, err := readADS1115(&fakeADS1115{addr: 0x48, neverDone: true}, good); err == nil {
		t.Error("no timeout error when the conversion never completes")
	}
	if _, err := readADS1115(&fakeADS1115{addr: 0x48, failTx: true}, good); err == nil {
		t.Error("no error on I2C failure")
	}
	if _, err := readADS1115(&fakeADS1115{addr: 0x4A}, good); err == nil {
		t.Error("no error when nothing answers at the address")
	}

	bad := []ADS1115Opts{
		{Addr: 0x48, Channel: 4, GainFSR: 4096, DRSPS: 860},
		{Addr: 0x48, Channel: 1, ChannelNeg: intPtr(2), GainFSR: 4096, DRSPS: 860},
		{Addr: 0x48, Channel: 0, GainFSR: 3000, DRSPS: 860},
		{Addr: 0x48, Channel: 0, GainFSR: 4096, DRSPS: 100},
	}
	for _, o := range bad {
		bus := &fakeADS1115{addr: 0x48}
		if _, err := readADS1115(bus, o); err == nil {
			t.Errorf("invalid options %+v accepted", o)
		}
		if len(bus.writes) != 0 {
			t.Errorf("invalid options %+v wrote to the chip", o)
		}
	}
}

func TestRBEMSPostData(t *testing.T) {
	var got map[string]float32
	srv := httptest.NewServer(http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.Method != http.MethodPost || r.Header.Get("Content-Type") != "application/json" {
			t.Errorf("unexpected request %s %s", r.Method, r.Header.Get("Content-Type"))
		}
		_ = json.NewDecoder(r.Body).Decode(&got)
	}))
	defer srv.Close()

	if rc := RBEMSPostData(srv.URL+"/setEMS", map[string]float32{"oilpressure": 3.5}); rc != 0 {
		t.Errorf("rc = %d, want 0", rc)
	}
	if got["oilpressure"] != 3.5 {
		t.Errorf("payload = %v", got)
	}
	srv.Close()
	if rc := RBEMSPostData(srv.URL+"/setEMS", map[string]float32{"x": 1}); rc != 2 {
		t.Errorf("rc with server down = %d, want 2", rc)
	}
}

// The shipped configuration shall only use values the driver accepts
func TestShippedConfig(t *testing.T) {
	var cfg struct {
		Url                string `json:"url"`
		AlertAfterFailures int    `json:"alertAfterFailures"`
		Sensors            []struct {
			Name         string `json:"name"`
			Kind         string `json:"kind"`
			Dev          string `json:"dev"`
			Addr         string `json:"addr"`
			Channel      int    `json:"channel"`
			ChannelNeg   *int   `json:"channel_neg"`
			GainmV       int    `json:"gain_mv"`
			DRSPS        int    `json:"dr_sps"`
			ValueType    string `json:"value_type"`
			AverageLevel int    `json:"average_level"`
		} `json:"sensors"`
	}
	data, err := os.ReadFile("ems-adc.json")
	if err != nil {
		t.Fatal(err)
	}
	if err := json.Unmarshal(data, &cfg); err != nil {
		t.Fatalf("ems-adc.json: %v", err)
	}
	if cfg.Url == "" || len(cfg.Sensors) == 0 {
		t.Fatal("ems-adc.json: missing url or sensors")
	}
	if cfg.AlertAfterFailures < 0 {
		t.Errorf("alertAfterFailures = %d", cfg.AlertAfterFailures)
	}
	names := map[string]bool{}
	for _, s := range cfg.Sensors {
		if s.Name == "" || names[s.Name] {
			t.Errorf("sensor name %q empty or duplicated", s.Name)
		}
		names[s.Name] = true
		if s.Kind != "ADS1115" {
			t.Errorf("%s: kind %q", s.Name, s.Kind)
		}
		if _, _, err := ads1115Gain(s.GainmV); err != nil {
			t.Errorf("%s: %v", s.Name, err)
		}
		if _, _, err := ads1115DataRate(s.DRSPS); err != nil {
			t.Errorf("%s: %v", s.Name, err)
		}
		if s.ChannelNeg != nil {
			_, err = ads1115MuxDifferential(s.Channel, *s.ChannelNeg)
		} else {
			_, err = ads1115MuxSingleEnded(s.Channel)
		}
		if err != nil {
			t.Errorf("%s: %v", s.Name, err)
		}
		if s.Addr != "" {
			if a, err := strconv.ParseUint(s.Addr, 0, 8); err != nil || a < 0x48 || a > 0x4B {
				t.Errorf("%s: addr %q not 0x48..0x4B", s.Name, s.Addr)
			}
		}
		if s.ValueType != "" && s.ValueType != "float" && s.ValueType != "int" {
			t.Errorf("%s: value_type %q", s.Name, s.ValueType)
		}
		if s.AverageLevel < 0 {
			t.Errorf("%s: average_level %d", s.Name, s.AverageLevel)
		}
	}
}
