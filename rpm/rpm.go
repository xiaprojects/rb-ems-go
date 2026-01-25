/*
This file is part of RB.

# Copyright (C) 2025 XIAPROJECTS SRL

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
	"bytes"
	"encoding/json"
	"fmt"
	"log"
	"net/http"
	"os"
	"strconv"
	"sync/atomic"
	"time"

	"github.com/warthog618/go-gpiocdev"
)

const (
	ChipPath    = "/dev/gpiochip0"
	APIURL      = "http://127.0.0.1/setEMS"
	PPR         = 4                      // pulses per revolution
	Window      = 250 * time.Millisecond // measurement window
	MinDeltaRPM = 5                      // send only if changed by >= this amount
)

type rpmPayload struct {
	RPM int `json:"enginerpm"`
}

func main() {
	if len(os.Args) < 2 {
		log.Fatal("Missing GPIO Number")
		os.Exit(1)
	}

	i, err := strconv.Atoi(os.Args[1])
	if err != nil {
		log.Fatal("GPIO shall be a number")
		os.Exit(1)
	}
	if i > 50 || i == 0 {
		log.Fatal("GPIO shall be a number between 1-49")
		os.Exit(1)
	}

	ems_rpm_sample_gpio(i)
	os.Exit(0)
}

func ems_rpm_sample_gpio(Offset int) {
	var pulseCount uint64

	// Edge event handler: keep it short (just count), as recommended. [web:47]
	eh := func(evt gpiocdev.LineEvent) {
		atomic.AddUint64(&pulseCount, 1)
	}

	line, err := gpiocdev.RequestLine(
		ChipPath,
		Offset,
		gpiocdev.WithRisingEdge,
		gpiocdev.WithEventHandler(eh),
		gpiocdev.WithDebounce(2*time.Millisecond),
	)
	if err != nil {
		fmt.Printf("GPIO RequestLine failed: chip=%s offset=%d err=%v\n", ChipPath, Offset, err)
		fmt.Println("Hints: try running with sudo, or fix /dev/gpiochip permissions; also try removing WithDebounce if unsupported.")
		os.Exit(1)
		return
	}
	defer line.Close()
	client := &http.Client{Timeout: 2 * time.Second}

	lastSent := -1
	minDutySent := 0

	ticker := time.NewTicker(Window)
	defer ticker.Stop()
	alpha := 0.2 // prova 0.1..0.3
	emaInit := false
	var ema float64

	for range ticker.C {
		pulses := atomic.SwapUint64(&pulseCount, 0)
		rawRPM := (float64(pulses) / Window.Seconds()) * (60.0 / float64(PPR))

		if !emaInit {
			ema = rawRPM
			emaInit = true
		} else {
			ema = alpha*rawRPM + (1.0-alpha)*ema
		}

		smoothRPM := int(ema + 0.5) // arrotonda

		if minDutySent > 10 || abs(smoothRPM-lastSent) >= MinDeltaRPM {
			fmt.Println("RPM:", smoothRPM)
			if err := postRPM(client, smoothRPM); err != nil {
				fmt.Println("POST error:", err)
			} else {
				lastSent = smoothRPM
			}
			minDutySent = 0
		}
		minDutySent = minDutySent + 1
	}

}

func postRPM(client *http.Client, rpm int) error {
	body, err := json.Marshal(rpmPayload{RPM: rpm})
	if err != nil {
		return err
	}
	req, err := http.NewRequest(http.MethodPost, APIURL, bytes.NewReader(body))
	if err != nil {
		return err
	}
	req.Header.Set("Content-Type", "application/json")

	resp, err := client.Do(req)
	if err != nil {
		return err
	}
	defer resp.Body.Close()

	if resp.StatusCode < 200 || resp.StatusCode >= 300 {
		return fmt.Errorf("http status: %s", resp.Status)
	}
	return nil
}

func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}
