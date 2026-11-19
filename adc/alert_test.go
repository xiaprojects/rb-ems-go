/*
	This file is part of RB.

	Copyright (C) 2023 XIAPROJECTS SRL

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
Tests for the RB-01 probe failure alerts (alert.go).
Like alert.go, this file is duplicated in main/, adc/ and gpio/: keep the copies
identical (make check-dup verifies it).
*/

import (
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"sync"
	"testing"
)

// fakeRB01 is a stand-in for the RB-01 POST /alert endpoint
type fakeRB01 struct {
	mu     sync.Mutex
	bodies []map[string]interface{}
	// status answered to the next requests, 200 when empty
	statuses []int
	nextId   int
	noId     bool
}

func (f *fakeRB01) ServeHTTP(w http.ResponseWriter, r *http.Request) {
	f.mu.Lock()
	defer f.mu.Unlock()
	if r.Method != http.MethodPost || r.URL.Path != "/alert" {
		http.Error(w, "unexpected "+r.Method+" "+r.URL.Path, http.StatusTeapot)
		return
	}
	if ct := r.Header.Get("Content-Type"); ct != "application/json" {
		http.Error(w, "bad content type "+ct, http.StatusBadRequest)
		return
	}
	var body map[string]interface{}
	if err := json.NewDecoder(r.Body).Decode(&body); err != nil {
		http.Error(w, err.Error(), http.StatusBadRequest)
		return
	}
	f.bodies = append(f.bodies, body)
	status := http.StatusOK
	if len(f.statuses) > 0 {
		status, f.statuses = f.statuses[0], f.statuses[1:]
	}
	if status != http.StatusOK {
		w.WriteHeader(status)
		return
	}
	if f.noId {
		_, _ = w.Write([]byte(`{}`))
		return
	}
	id := f.nextId
	if v, ok := body["Id"]; ok {
		id = int(v.(float64))
	} else {
		f.nextId++
	}
	_ = json.NewEncoder(w).Encode(map[string]int{"Id": id})
}

func (f *fakeRB01) requests() []map[string]interface{} {
	f.mu.Lock()
	defer f.mu.Unlock()
	return append([]map[string]interface{}(nil), f.bodies...)
}

func newFakeRB01(t *testing.T) (*fakeRB01, *httptest.Server) {
	f := &fakeRB01{nextId: 7}
	srv := httptest.NewServer(f)
	t.Cleanup(srv.Close)
	return f, srv
}

func TestAlertURLDerivation(t *testing.T) {
	cases := []struct {
		alertUrl, dataUrl string
		threshold         int
		wantUrl           string
		wantThreshold     int
	}{
		{"", "http://localhost/setEMS", 3, "http://localhost/alert", 3},
		{"", "http://10.0.0.1:8080/setEMS?x=1#f", 3, "http://10.0.0.1:8080/alert", 3},
		{"http://rb01/custom", "http://localhost/setEMS", 3, "http://rb01/custom", 3},
		{"", "", 3, "", 0},                // nothing to derive from: disabled
		{"", "not a url", 3, "", 0},       // no host: disabled
		{"", "http://h/setEMS", 0, "", 0}, // threshold 0: disabled
		{"", "http://h/setEMS", -1, "", -1},
	}
	for _, c := range cases {
		tr := NewRBAlertTracker(c.alertUrl, c.dataUrl, c.threshold)
		if tr.url != c.wantUrl || tr.threshold != c.wantThreshold {
			t.Errorf("NewRBAlertTracker(%q, %q, %d) = url %q threshold %d; want %q %d",
				c.alertUrl, c.dataUrl, c.threshold, tr.url, tr.threshold, c.wantUrl, c.wantThreshold)
		}
	}
}

func TestAlertRaiseAndEnd(t *testing.T) {
	f, srv := newFakeRB01(t)
	tr := NewRBAlertTracker(srv.URL+"/alert", "", 3)

	tr.Failure("egt1", "EGT 1")
	tr.Failure("egt1", "EGT 1")
	if n := len(f.requests()); n != 0 {
		t.Fatalf("alert sent before the threshold: %d requests", n)
	}
	tr.Failure("egt1", "EGT 1")
	reqs := f.requests()
	if len(reqs) != 1 {
		t.Fatalf("got %d requests at the threshold, want 1", len(reqs))
	}
	raise := reqs[0]
	if raise["Type"] != float64(RB_EVENT_TYPE_EMS_GENERIC_FAILURE) || raise["Title"] != "EGT 1 probe failure" || raise["Active"] != true {
		t.Errorf("raise body = %v", raise)
	}
	if RB_EVENT_TYPE_EMS_GENERIC_FAILURE != 13 {
		t.Errorf("EVENT_TYPE_EMS_GENERIC_FAILURE = %d, RB-01 expects 13", RB_EVENT_TYPE_EMS_GENERIC_FAILURE)
	}

	// No keep-alive while the fault lasts
	for i := 0; i < 5; i++ {
		tr.Failure("egt1", "EGT 1")
	}
	if n := len(f.requests()); n != 1 {
		t.Fatalf("keep-alive sent while the fault lasts: %d requests", n)
	}

	tr.Success("egt1")
	reqs = f.requests()
	if len(reqs) != 2 {
		t.Fatalf("got %d requests after recovery, want 2", len(reqs))
	}
	end := reqs[1]
	if end["Id"] != float64(7) || end["Active"] != false || len(end) != 2 {
		t.Errorf("end body = %v, want {Id:7 Active:false}", end)
	}

	// Further good reads send nothing
	tr.Success("egt1")
	if n := len(f.requests()); n != 2 {
		t.Errorf("extra request on a healthy probe: %d", n)
	}
}

func TestAlertFailureCounterResets(t *testing.T) {
	f, srv := newFakeRB01(t)
	tr := NewRBAlertTracker(srv.URL+"/alert", "", 3)
	for i := 0; i < 10; i++ {
		tr.Failure("cht1", "")
		tr.Failure("cht1", "")
		tr.Success("cht1") // intermittent: never 3 in a row
	}
	if n := len(f.requests()); n != 0 {
		t.Errorf("alert raised on non consecutive failures: %d requests", n)
	}
}

func TestAlertTitleDefaultsToName(t *testing.T) {
	f, srv := newFakeRB01(t)
	tr := NewRBAlertTracker(srv.URL+"/alert", "", 1)
	tr.Failure("cht2", "")
	reqs := f.requests()
	if len(reqs) != 1 || reqs[0]["Title"] != "cht2 probe failure" {
		t.Errorf("requests = %v, want title %q", reqs, "cht2 probe failure")
	}
}

func TestAlertProbesAreIndependent(t *testing.T) {
	f, srv := newFakeRB01(t)
	tr := NewRBAlertTracker(srv.URL+"/alert", "", 2)
	tr.Failure("a", "A")
	tr.Failure("b", "B")
	tr.Failure("a", "A") // a raised, Id 7
	tr.Failure("b", "B") // b raised, Id 8
	tr.Success("b")
	reqs := f.requests()
	if len(reqs) != 3 {
		t.Fatalf("got %d requests, want 3", len(reqs))
	}
	if reqs[2]["Id"] != float64(8) {
		t.Errorf("ended Id = %v, want 8 (probe b)", reqs[2]["Id"])
	}
	if !tr.probes["a"].open || tr.probes["b"].open {
		t.Errorf("state a.open=%v b.open=%v, want true false", tr.probes["a"].open, tr.probes["b"].open)
	}
}

func TestAlertRaiseRetriedWhenRB01Fails(t *testing.T) {
	f, srv := newFakeRB01(t)
	f.statuses = []int{http.StatusInternalServerError}
	tr := NewRBAlertTracker(srv.URL+"/alert", "", 2)
	tr.Failure("oil", "Oil")
	tr.Failure("oil", "Oil") // 500: not raised
	if tr.probes["oil"].open {
		t.Fatal("alert marked open although RB-01 answered 500")
	}
	tr.Failure("oil", "Oil") // retried
	if n := len(f.requests()); n != 2 || !tr.probes["oil"].open {
		t.Errorf("requests=%d open=%v, want 2 true", n, tr.probes["oil"].open)
	}
}

func TestAlertRaiseWithoutIdIsRetried(t *testing.T) {
	f, srv := newFakeRB01(t)
	f.noId = true
	tr := NewRBAlertTracker(srv.URL+"/alert", "", 1)
	tr.Failure("oil", "Oil")
	if tr.probes["oil"].open {
		t.Error("alert marked open although RB-01 answered without Id")
	}
}

func TestAlertEndRetriedAndNotFound(t *testing.T) {
	f, srv := newFakeRB01(t)
	tr := NewRBAlertTracker(srv.URL+"/alert", "", 1)
	tr.Failure("gear", "Gear")

	f.mu.Lock()
	f.statuses = []int{http.StatusInternalServerError}
	f.mu.Unlock()
	tr.Success("gear") // 500: still open
	if !tr.probes["gear"].open {
		t.Fatal("alert closed although RB-01 answered 500")
	}

	f.mu.Lock()
	f.statuses = []int{http.StatusNotFound}
	f.mu.Unlock()
	tr.Success("gear") // 404: RB-01 forgot it, treat as ended
	if tr.probes["gear"].open {
		t.Error("alert still open after RB-01 answered 404")
	}
	if n := len(f.requests()); n != 3 {
		t.Errorf("got %d requests, want 3", n)
	}
}

func TestAlertRB01Unreachable(t *testing.T) {
	_, srv := newFakeRB01(t)
	u := srv.URL + "/alert"
	srv.Close()
	tr := NewRBAlertTracker(u, "", 1)
	tr.Failure("x", "X") // shall not panic nor block
	if tr.probes["x"].open {
		t.Error("alert marked open with RB-01 down")
	}
}

func TestAlertDisabledAndNilTracker(t *testing.T) {
	f, srv := newFakeRB01(t)
	tr := NewRBAlertTracker(srv.URL+"/alert", "", 0)
	tr.Failure("x", "X")
	tr.Success("x")
	if n := len(f.requests()); n != 0 {
		t.Errorf("disabled tracker sent %d requests", n)
	}
	var nilTracker *RBAlertTracker
	nilTracker.Failure("x", "X")
	nilTracker.Success("x")
}
