/*
This file is part of RB.

# Copyright (C) 2026 XIAPROJECTS SRL

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

Community edition will be free for all builders and personal use as defined by the licensing model
Dual licensing for commercial agreement is available
Please join Discord community
*/
package main

/*
Probe failure alerts pushed to RB-01 through POST /alert.

	{"Type": 13, "Title": "EGT 1 probe failure", "Active": true}  raises, RB-01 answers {"Id": n}
	{"Id": n, "Active": false}                                    ends it

A probe raises the alert after alertAfterFailures failed reads in a row and ends it on
the first good read. Nothing is sent while the fault lasts (no keep-alive).
The file is duplicated in each module on purpose: modules are independent.
*/

import (
	"bytes"
	"encoding/json"
	"errors"
	"log"
	"net/http"
	"net/url"
	"time"
)

// RB-01 EVENT_TYPE_EMS_GENERIC_FAILURE
const RB_EVENT_TYPE_EMS_GENERIC_FAILURE = 13

// A stuck RB-01 shall not stall the sensor loop
const RB_ALERT_HTTP_TIMEOUT = 1 * time.Second

type probeAlertState struct {
	failures int
	id       int
	open     bool
}

type RBAlertTracker struct {
	url       string
	threshold int
	client    *http.Client
	probes    map[string]*probeAlertState
}

/*
NewRBAlertTracker().

	threshold <= 0 disables the alerts: the tracker then does nothing.
	alertUrl empty is derived from dataUrl with the path replaced by /alert
*/
func NewRBAlertTracker(alertUrl string, dataUrl string, threshold int) *RBAlertTracker {
	t := &RBAlertTracker{
		threshold: threshold,
		client:    &http.Client{Timeout: RB_ALERT_HTTP_TIMEOUT},
		probes:    make(map[string]*probeAlertState),
	}
	if threshold <= 0 {
		log.Printf("Probe alerts disabled (alertAfterFailures=%d)", threshold)
		return t
	}
	t.url = alertUrl
	if t.url == "" {
		if u, err := url.Parse(dataUrl); err == nil && u.Host != "" {
			u.Path = "/alert"
			u.RawQuery = ""
			u.Fragment = ""
			t.url = u.String()
		}
	}
	if t.url == "" {
		log.Printf("Probe alerts disabled: no alertUrl and cannot derive it from url %q", dataUrl)
		t.threshold = 0
		return t
	}
	log.Printf("Probe alerts enabled: %s after %d failures in a row", t.url, threshold)
	return t
}

func (t *RBAlertTracker) probe(name string) *probeAlertState {
	p, ok := t.probes[name]
	if !ok {
		p = &probeAlertState{}
		t.probes[name] = p
	}
	return p
}

/*
Failure().

	One failed read of the probe. Raises the alert once the threshold is reached;
	if RB-01 could not take it, the next failure tries again
*/
func (t *RBAlertTracker) Failure(name string, title string) {
	if t == nil || t.threshold <= 0 {
		return
	}
	p := t.probe(name)
	if p.failures < t.threshold {
		p.failures++
	}
	if p.open || p.failures < t.threshold {
		return
	}
	if title == "" {
		title = name
	}
	title = title + " probe failure"
	id, status, err := t.post(map[string]interface{}{
		"Type":   RB_EVENT_TYPE_EMS_GENERIC_FAILURE,
		"Title":  title,
		"Active": true,
	})
	if err != nil || status != http.StatusOK {
		log.Printf("%s alert not raised (status %d): %v", name, status, err)
		return
	}
	p.open = true
	p.id = id
	log.Printf("%s alert raised: %q Id=%d", name, title, id)
}

/*
Success().

	One good read of the probe. Ends the open alert, if any; if RB-01 could not
	take it, the next good read tries again. 404: RB-01 no longer has the alert
*/
func (t *RBAlertTracker) Success(name string) {
	if t == nil || t.threshold <= 0 {
		return
	}
	p, ok := t.probes[name]
	if !ok {
		return
	}
	p.failures = 0
	if !p.open {
		return
	}
	_, status, err := t.post(map[string]interface{}{
		"Id":     p.id,
		"Active": false,
	})
	if err != nil || (status != http.StatusOK && status != http.StatusNotFound) {
		log.Printf("%s alert Id=%d not ended (status %d): %v", name, p.id, status, err)
		return
	}
	log.Printf("%s alert ended: Id=%d (status %d)", name, p.id, status)
	p.open = false
	p.id = 0
}

func (t *RBAlertTracker) post(body map[string]interface{}) (int, int, error) {
	data, err := json.Marshal(body)
	if err != nil {
		return 0, 0, err
	}
	resp, err := t.client.Post(t.url, "application/json", bytes.NewBuffer(data))
	if err != nil {
		return 0, 0, err
	}
	defer resp.Body.Close()
	if resp.StatusCode != http.StatusOK {
		return 0, resp.StatusCode, nil
	}
	var answer struct {
		Id *int
	}
	if err := json.NewDecoder(resp.Body).Decode(&answer); err != nil {
		return 0, resp.StatusCode, err
	}
	if answer.Id == nil {
		return 0, resp.StatusCode, errMissingId
	}
	return *answer.Id, resp.StatusCode, nil
}

var errMissingId = errors.New("RB-01 answer without Id")
