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
Hardware-free tests for the RPM daemon: pulse to RPM conversion, EMA smoothing,
change-detection on send and the shipped configuration. The GPIO interrupt is
replaced by writing the pulse counter directly.
*/

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"os"
	"sync"
	"sync/atomic"
	"testing"
	"time"
)

func TestRpmReadLoopConversionAndEMA(t *testing.T) {
	cfg := config{ppr: 4, window: 50 * time.Millisecond, alpha: 0.2, chanBuf: 8}
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	var pulses uint64
	// 10 pulses in 50 ms at 4 pulses/rev = 200 pulses/s = 3000 RPM
	atomic.StoreUint64(&pulses, 10)
	out := make(chan rpmSample, cfg.chanBuf)
	go rpmReadLoop(ctx, &pulses, cfg, out)

	first := <-out
	if first.Pulses != 10 || first.RawRPM != 3000 || first.RPM != 3000 {
		t.Errorf("first sample = %+v, want 10 pulses, 3000 RPM", first)
	}
	// No pulses in the next window: EMA 0.2*0 + 0.8*3000
	second := <-out
	if second.Pulses != 0 || second.RawRPM != 0 || second.RPM != 2400 {
		t.Errorf("second sample = %+v, want 0 pulses, raw 0, smoothed 2400", second)
	}
	cancel()
	for range out { // the reader closes the channel on exit
	}
}

func TestRpmReadLoopAlphaClamped(t *testing.T) {
	for _, alpha := range []float64{-1, 5} {
		cfg := config{ppr: 1, window: 20 * time.Millisecond, alpha: alpha, chanBuf: 8}
		ctx, cancel := context.WithCancel(context.Background())
		var pulses uint64 = 1 // 1 pulse in 20 ms at 1 pulse/rev = 3000 RPM
		out := make(chan rpmSample, cfg.chanBuf)
		go rpmReadLoop(ctx, &pulses, cfg, out)
		<-out
		s := <-out
		want := 0 // alpha 1: follows the raw value
		if alpha < 0 {
			want = 3000 // alpha 0: frozen on the first value
		}
		if s.RPM != want {
			t.Errorf("alpha %v: second RPM = %d, want %d", alpha, s.RPM, want)
		}
		cancel()
		for range out {
		}
	}
}

type rpmRecorder struct {
	mu       sync.Mutex
	got      []int
	failNext bool
}

func (r *rpmRecorder) ServeHTTP(w http.ResponseWriter, req *http.Request) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.failNext {
		r.failNext = false
		w.WriteHeader(http.StatusServiceUnavailable)
		return
	}
	var body map[string]int
	_ = json.NewDecoder(req.Body).Decode(&body)
	r.got = append(r.got, body["enginerpm"])
}

func withAPI(t *testing.T, h http.Handler) {
	srv := httptest.NewServer(h)
	oldURL, oldKey := APIURL, payloadKey
	APIURL, payloadKey = srv.URL+"/setEMS", "enginerpm"
	t.Cleanup(func() { srv.Close(); APIURL, payloadKey = oldURL, oldKey })
}

func TestRpmSendLoopChangeDetection(t *testing.T) {
	rec := &rpmRecorder{}
	withAPI(t, rec)
	cfg := config{minDelta: 5, maxSendGap: time.Hour}
	in := make(chan rpmSample, 16)
	for _, rpm := range []int{1000, 1002, 1004, 1005, 1001, 2000, 0} {
		in <- rpmSample{RPM: rpm}
	}
	close(in)
	rpmSendLoop(context.Background(), cfg, in)
	want := []int{1000, 1005, 2000, 0}
	if len(rec.got) != len(want) {
		t.Fatalf("sent %v, want %v", rec.got, want)
	}
	for i := range want {
		if rec.got[i] != want[i] {
			t.Fatalf("sent %v, want %v", rec.got, want)
		}
	}
}

func TestRpmSendLoopMaxGap(t *testing.T) {
	rec := &rpmRecorder{}
	withAPI(t, rec)
	cfg := config{minDelta: 100, maxSendGap: 300 * time.Millisecond}
	in := make(chan rpmSample)
	done := make(chan struct{})
	go func() { rpmSendLoop(context.Background(), cfg, in); close(done) }()
	in <- rpmSample{RPM: 2400}
	in <- rpmSample{RPM: 2401} // inside the gap: skipped
	time.Sleep(400 * time.Millisecond)
	in <- rpmSample{RPM: 2402} // gap elapsed: forced send
	close(in)
	<-done
	if len(rec.got) != 2 || rec.got[1] != 2402 {
		t.Errorf("sent %v, want [2400 2402]", rec.got)
	}
}

func TestRpmSendLoopRetriesAfterPostError(t *testing.T) {
	rec := &rpmRecorder{failNext: true}
	withAPI(t, rec)
	cfg := config{minDelta: 5, maxSendGap: time.Hour}
	in := make(chan rpmSample, 4)
	in <- rpmSample{RPM: 1500} // 503: not counted as sent
	in <- rpmSample{RPM: 1500} // retried even though unchanged
	close(in)
	rpmSendLoop(context.Background(), cfg, in)
	if len(rec.got) != 1 || rec.got[0] != 1500 {
		t.Errorf("sent %v, want [1500]", rec.got)
	}
}

func TestPostRPMStatus(t *testing.T) {
	withAPI(t, http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.Header.Get("Content-Type") != "application/json" {
			t.Errorf("content type %q", r.Header.Get("Content-Type"))
		}
		w.WriteHeader(http.StatusInternalServerError)
	}))
	if err := postRPM(http.DefaultClient, "enginerpm", 1); err == nil {
		t.Error("no error on HTTP 500")
	}
}

func TestShippedConfig(t *testing.T) {
	data, err := os.ReadFile("ems-rpm.json")
	if err != nil {
		t.Fatal(err)
	}
	var cfg configFile
	if err := json.Unmarshal(data, &cfg); err != nil {
		t.Fatalf("ems-rpm.json: %v", err)
	}
	found := false
	for _, s := range cfg.Sensors {
		if s.Kind != "gpiopulse" {
			continue
		}
		found = true
		// same bounds main() enforces before opening the line
		if s.GPIO <= 0 || s.GPIO > 50 {
			t.Errorf("%s: gpio %d rejected by main()", s.Name, s.GPIO)
		}
	}
	if !found {
		t.Error("no gpiopulse sensor: ems-rpm would exit")
	}
}
