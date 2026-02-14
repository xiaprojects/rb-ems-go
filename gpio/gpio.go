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
	"bytes"
	"encoding/json"
	"io"
	"net/http"
	"os"
	"time"

	"github.com/warthog618/go-gpiocdev"
)

func RBEMSPostData(url string, payload map[string]float32) int {
	data, err := json.Marshal(payload)
	if err != nil {
		return 1
	}

	resp, err := http.Post(url, "application/json", bytes.NewBuffer(data))
	if err != nil {
		return 2
	}
	defer resp.Body.Close()
	return 0
}

func ems_gpio(configFilename string) {
	type sensor struct {
		Name    string `json:"name"`
		Kind    string `json:"kind"`
		Dev     string `json:"dev"`
		Channel int    `json:"channel"`
	}
	type config struct {
		Url     string   `json:"url"`
		Sensors []sensor `json:"sensors"`
	}

	f, err := os.Open(configFilename)
	if err != nil {
		panic(err)
	}
	defer f.Close()

	rawCfg, err := io.ReadAll(f)
	if err != nil {
		panic(err)
	}

	var configuration config
	if err := json.Unmarshal(rawCfg, &configuration); err != nil {
		panic(err)
	}

	// Initialize buffer to store previous sensor readings for change detection
	previousReadings := make(map[string]int)

	// Open GPIO chip
	chip, err := gpiocdev.NewChip("gpiochip0")
	if err != nil {
		panic(err)
	}
	defer chip.Close()

	// Main loop
	for {
		// Sleep for 100 ms
		time.Sleep(100 * time.Millisecond)

		// Create payload for this cycle
		payload := make(map[string]float32)
		changed := false

		// Cycle through sensors array
		for _, sensor := range configuration.Sensors {
			key := sensor.Name
			offset := sensor.Channel

			// Open GPIO line using channel as offset
			line, err := chip.RequestLine(offset)
			if err != nil {
				continue
			}

			// Read the GPIO value (0 or 1)
			value, err := line.Value()
			if err != nil {
				line.Close()
				continue
			}

			// Close the line
			line.Close()

			// Check if value changed
			if previousValue, exists := previousReadings[key]; !exists || previousValue != value {
				changed = true
				previousReadings[key] = value

				// Convert reading to float32 for payload
				payload[key] = float32(value)
			}
		}

		// At the end of the loop transmit if changed
		if changed && len(payload) > 0 {
			RBEMSPostData(configuration.Url, payload)
		}
	}

}
