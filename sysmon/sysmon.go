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
	"flag"
	"fmt"
	"io"
	"net/http"
	"os"
	"os/exec"
	"strconv"
	"strings"
	"sync"
	"time"
)

// Config represents the JSON configuration structure
type Config struct {
	URL     string   `json:"url"`
	Sensors []Sensor `json:"sensors"`
}

// Sensor represents individual sensor configuration
type Sensor struct {
	Name      string `json:"name"`
	Dev       string `json:"dev"`
	Kind      string `json:"kind"`
	ValueType string `json:"value_type"`
}

// SensorData represents the data to be sent to the server
type SensorData map[string]interface{}

// SensorState tracks previous values for change detection
type SensorState struct {
	PrevValue interface{}
	Valid     bool
	mu        sync.RWMutex
}

type SensorStates map[string]*SensorState

func main() {
	configPath := flag.String("config", "", "Path to JSON configuration file")
	waitTime := flag.Int("wait", 10, "Wait time between loops in seconds")
	stateFile := flag.String("state", "sensor_states.json", "Path to state storage file (empty to disable)")
	flag.Parse()

	if *configPath == "" {
		fmt.Println("Error: config path is required. Use -config <path>")
		return
	}

	// Read and parse config
	config, err := loadConfig(*configPath)
	if err != nil {
		fmt.Printf("Error loading config: %v\n", err)
		return
	}

	// Initialize sensor states for change detection
	states := initSensorStates(config.Sensors)

	// Load states only if state file is specified and exists
	if *stateFile != "" {
		loadSensorStates(states, *stateFile)
		fmt.Printf("Using state file: %s\n", *stateFile)
	} else {
		fmt.Println("State persistence disabled")
	}

	fmt.Printf("Loaded %d sensors, posting to %s every %d seconds\n",
		len(config.Sensors), config.URL, *waitTime)

	for {

		err := executeSensors(config, states, *stateFile)
		if err != nil {
			fmt.Printf("Error in execution loop: %v\n", err)
		}
		time.Sleep(10 * time.Second)
	}
}

func loadConfig(path string) (*Config, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, fmt.Errorf("failed to read config file: %w", err)
	}

	var config Config
	if err := json.Unmarshal(data, &config); err != nil {
		return nil, fmt.Errorf("failed to parse config: %w", err)
	}

	return &config, nil
}

func initSensorStates(sensors []Sensor) SensorStates {
	states := make(SensorStates)
	for _, sensor := range sensors {
		states[sensor.Name] = &SensorState{
			Valid: false,
		}
	}
	return states
}

func loadSensorStates(states SensorStates, stateFile string) {
	data, err := os.ReadFile(stateFile)
	if err != nil {
		return // File doesn't exist yet
	}

	var savedStates map[string]interface{}
	if err := json.Unmarshal(data, &savedStates); err != nil {
		return
	}

	for name, prevValue := range savedStates {
		if state, exists := states[name]; exists {
			state.mu.Lock()
			state.PrevValue = prevValue
			state.Valid = true
			state.mu.Unlock()
		}
	}
}

func saveSensorStates(states SensorStates, stateFile string) error {
	toSave := make(map[string]interface{})

	for name, state := range states {
		state.mu.RLock()
		if state.Valid {
			toSave[name] = state.PrevValue
		}
		state.mu.RUnlock()
	}

	data, err := json.MarshalIndent(toSave, "", "  ")
	if err != nil {
		return err
	}

	return os.WriteFile(stateFile, data, 0644)
}

func executeSensors(config *Config, states SensorStates, stateFile string) error {
	sensorData := make(SensorData)
	var mu sync.Mutex

	for _, sensor := range config.Sensors {
		time.Sleep(100 * time.Millisecond)
		value, changed, err := processSensor(sensor, states[sensor.Name])
		if err != nil {
			fmt.Printf("Sensor %s failed: %v\n", sensor.Name, err)
			continue // Skip failed sensors
		} else {
			fmt.Printf("Sensor %s %.2f\n", sensor.Name, value)
		}

		if !changed {
			continue // Skip unchanged sensors
		}

		mu.Lock()
		sensorData[sensor.Name] = value
		mu.Unlock()

	}

	// Save states only if state file is specified
	if stateFile != "" {
		if err := saveSensorStates(states, stateFile); err != nil {
			fmt.Printf("Warning: failed to save states to %s: %v\n", stateFile, err)
		}
	}

	if len(sensorData) == 0 {
		return nil
	}

	// POST only changed data
	return postData(config.URL, sensorData)
}

func processSensor(sensor Sensor, state *SensorState) (interface{}, bool, error) {
	if sensor.Kind != "executable" {
		return nil, false, fmt.Errorf("unsupported kind: %s", sensor.Kind)
	}

	cmd := exec.Command("bash", "-c", sensor.Dev)
	var stdout, stderr bytes.Buffer
	cmd.Stdout = &stdout
	cmd.Stderr = &stderr

	err := cmd.Run()
	if err != nil {
		return nil, false, fmt.Errorf("command '%s' failed: %w (stderr: %s)", sensor.Dev, err, stderr.String())
	}

	output := strings.TrimSpace(stdout.String())
	newValue, parseErr := parseValue(output, sensor.ValueType)
	if parseErr != nil {
		return nil, false, fmt.Errorf("parse error: %w", parseErr)
	}

	// Compare with previous value
	state.mu.Lock()
	defer state.mu.Unlock()

	if state.Valid && valuesEqual(state.PrevValue, newValue) {
		return nil, false, nil // No change
	}

	// Update state
	state.PrevValue = newValue
	state.Valid = true
	return newValue, true, nil
}

func parseValue(output, valueType string) (interface{}, error) {
	switch valueType {
	case "int":
		value, err := strconv.ParseInt(output, 10, 64)
		return value, err
	case "float":
		value, err := strconv.ParseFloat(output, 64)
		return value, err
	case "string":
		return output, nil
	default:
		return output, nil
	}
}

func valuesEqual(a, b interface{}) bool {
	// Handle nil cases
	if a == nil || b == nil {
		return a == b
	}

	// Convert to strings for comparison (handles numbers as strings too)
	aStr := fmt.Sprintf("%v", a)
	bStr := fmt.Sprintf("%v", b)
	return aStr == bStr
}

func postData(url string, data SensorData) error {
	jsonData, err := json.Marshal(data)
	if err != nil {
		return fmt.Errorf("failed to marshal data: %w", err)
	}

	resp, err := http.Post(url, "application/json", bytes.NewBuffer(jsonData))
	if err != nil {
		return fmt.Errorf("POST request failed: %w", err)
	}
	defer resp.Body.Close()

	body, _ := io.ReadAll(resp.Body)
	if resp.StatusCode >= 400 {
		return fmt.Errorf("server returned %s: %s", resp.Status, string(body))
	}

	fmt.Printf("Posted changes: %s\n", string(jsonData))
	return nil
}
