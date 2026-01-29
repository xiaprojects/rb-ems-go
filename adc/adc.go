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
	"fmt"
	"io"
	"log"
	"math"
	"net/http"
	"os"
	"strconv"
	"time"

	"periph.io/x/conn/v3/i2c"
	"periph.io/x/conn/v3/i2c/i2creg"
	"periph.io/x/host/v3"
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

type ADS1115Opts struct {
	Addr    byte
	Channel int // 0..3
	Bus     int // i2c-0, i2c-1, ...
	GainFSR int // 6144,4096,2048,1024,512,256 (mV full-scale)
	DRSPS   int // 8,16,32,64,128,250,475,860
	Clamp0  bool
}

func ads1115Gain(gainFSRmV int) (uint16, float64, error) {
	switch gainFSRmV {
	case 6144:
		return 0x0000, 187.5e-6, nil
	case 4096:
		return 0x0200, 125e-6, nil
	case 2048:
		return 0x0400, 62.5e-6, nil
	case 1024:
		return 0x0600, 31.25e-6, nil
	case 512:
		return 0x0800, 15.625e-6, nil
	case 256:
		return 0x0A00, 7.8125e-6, nil
	default:
		return 0, 0, fmt.Errorf("unsupported gain_fsr_mv=%d (use 6144/4096/2048/1024/512/256)", gainFSRmV)
	}
}

func ads1115DataRate(drSPS int) (uint16, time.Duration, error) {
	switch drSPS {
	case 8:
		return 0x0000, 125 * time.Millisecond, nil
	case 16:
		return 0x0020, 63 * time.Millisecond, nil
	case 32:
		return 0x0040, 32 * time.Millisecond, nil
	case 64:
		return 0x0060, 16 * time.Millisecond, nil
	case 128:
		return 0x0080, 8 * time.Millisecond, nil
	case 250:
		return 0x00A0, 4 * time.Millisecond, nil
	case 475:
		return 0x00C0, 3 * time.Millisecond, nil
	case 860:
		return 0x00E0, 2 * time.Millisecond, nil
	default:
		return 0, 0, fmt.Errorf("unsupported dr_sps=%d (use 8/16/32/64/128/250/475/860)", drSPS)
	}
}

func ads1115MuxSingleEnded(channel int) (uint16, error) {
	switch channel {
	case 0:
		return 0x4000, nil // AIN0 vs GND
	case 1:
		return 0x5000, nil // AIN1 vs GND
	case 2:
		return 0x6000, nil // AIN2 vs GND
	case 3:
		return 0x7000, nil // AIN3 vs GND
	default:
		return 0, fmt.Errorf("invalid channel %d (expected 0..3)", channel)
	}
}

func readADS1115(bus i2c.Bus, opt ADS1115Opts) (float64, error) {
	const (
		regConv   = 0x00
		regConfig = 0x01

		osSingleStart  = 0x8000
		modeSingleShot = 0x0100
		compDisable    = 0x0003

		osBitMask = 0x8000
	)

	dev := &i2c.Dev{Bus: bus, Addr: uint16(opt.Addr)}

	readReg16 := func(reg byte) (uint16, []byte, error) {
		r := make([]byte, 2)
		if err := dev.Tx([]byte{reg}, r); err != nil {
			return 0, nil, fmt.Errorf("ADS1115 Tx reg 0x%02X: %w", reg, err)
		}
		u := uint16(r[0])<<8 | uint16(r[1])
		return u, r, nil
	}

	writeReg16 := func(reg byte, val uint16) error {
		w := []byte{reg, byte(val >> 8), byte(val & 0xFF)}
		if err := dev.Tx(w, nil); err != nil {
			return fmt.Errorf("ADS1115 write reg 0x%02X: %w", reg, err)
		}
		return nil
	}

	mux, err := ads1115MuxSingleEnded(opt.Channel)
	if err != nil {
		return 0, err
	}
	gainBits, lsb, err := ads1115Gain(opt.GainFSR)
	if err != nil {
		return 0, err
	}
	drBits, convTime, err := ads1115DataRate(opt.DRSPS)
	if err != nil {
		return 0, err
	}

	cfgWant := uint16(osSingleStart | mux | gainBits | modeSingleShot | drBits | compDisable)
	//log.Printf("ADS1115 write cfgWant=0x%04X bytes=%02X %02X %02X", cfgWant, regConfig, byte(cfgWant>>8), byte(cfgWant&0xFF))

	if err := writeReg16(regConfig, cfgWant); err != nil {
		return 0, err
	}

	// Poll OS ready
	deadline := time.Now().Add(2*convTime + 10*time.Millisecond)
	for {
		u, _, err := readReg16(regConfig)
		if err != nil {
			return 0, err
		}
		ready := (u & osBitMask) != 0

		if ready {
			break
		}
		if time.Now().After(deadline) {
			return 0, fmt.Errorf("ADS1115 conversion not ready (timeout), last cfg=0x%04X", u)
		}
		time.Sleep(1 * time.Millisecond)
	}

	// Read conversion
	u, _, err := readReg16(regConv)
	if err != nil {
		return 0, err
	}
	raw := int16(u)
	v := float64(raw) * lsb

	if opt.Clamp0 && v < 0 {
		v = 0
	}
	return v, nil
}

// LUT basata su valori Westach 399-series pubblicati (°F vs ohm).
var westach399_R_ohm_vs_F = []struct {
	F float64
	R float64
}{
	{32, 9800},
	{70, 3570},
	{100, 1750},
	{212, 212},
}

func westach399S9_F_from_R(R float64) (float64, error) {
	if R <= 0 {
		return 0, fmt.Errorf("invalid resistance %.3f", R)
	}

	if R >= westach399_R_ohm_vs_F[0].R {
		return westach399_R_ohm_vs_F[0].F, nil
	}
	last := westach399_R_ohm_vs_F[len(westach399_R_ohm_vs_F)-1]
	if R <= last.R {
		return last.F, nil
	}

	for i := 0; i < len(westach399_R_ohm_vs_F)-1; i++ {
		a := westach399_R_ohm_vs_F[i]
		b := westach399_R_ohm_vs_F[i+1]
		if R <= a.R && R >= b.R {
			la := math.Log(a.R)
			lb := math.Log(b.R)
			lr := math.Log(R)
			t := (lr - la) / (lb - la)
			return a.F + t*(b.F-a.F), nil
		}
	}
	return 0, fmt.Errorf("resistance out of lut range: %.3f", R)
}

func ems_adc_sample_i2c(configFilename string) {
	type sensor struct {
		Name    string  `json:"name"`
		Kind    string  `json:"kind"`
		Dev     string  `json:"dev"`
		Addr    string  `json:"addr"`
		Channel int     `json:"channel"`
		R0      float64 `json:"r0"`
		GainmV  int     `json:"gain_mv"`
		DRSPS   int     `json:"dr_sps"`
		Probe   *string `json:"probe"` // null oppure "399S9"
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

	if _, err := host.Init(); err != nil {
		panic(err)
	}

	buses := map[int]i2c.BusCloser{}
	defer func() {
		for _, b := range buses {
			_ = b.Close()
		}
	}()

	getBus := func(idx int) (i2c.BusCloser, error) {
		if b, ok := buses[idx]; ok {
			return b, nil
		}
		b, err := i2creg.Open(strconv.Itoa(idx))
		if err != nil {
			return nil, err
		}
		buses[idx] = b
		return b, nil
	}

	parseBusIndex := func(dev string) (int, error) {
		// supporta "1" oppure "i2c-1"
		if len(dev) >= 4 && dev[:4] == "i2c-" {
			return strconv.Atoi(dev[4:])
		}
		return strconv.Atoi(dev)
	}

	parseAddr := func(s string) (byte, error) {
		if s == "" {
			return 0x48, nil
		}
		v, err := strconv.ParseUint(s, 0, 8)
		if err != nil {
			return 0, err
		}
		return byte(v), nil
	}

	for {
		payload := make(map[string]float32)

		for _, s := range configuration.Sensors {
			if s.Kind != "ADS1115" {
				//log.Printf("%s error: chip not supported: %s", s.Name, s.Kind)
				continue
			}

			busIdx, err := parseBusIndex(s.Dev)
			if err != nil {
				//log.Printf("%s error: invalid i2c bus: %s", s.Name, s.Dev)
				continue
			}
			addr, err := parseAddr(s.Addr)
			if err != nil {
				//log.Printf("%s error: invalid addr: %s", s.Name, s.Addr)
				continue
			}

			bus, err := getBus(busIdx)
			if err != nil {
				//log.Printf("%s error: open i2c bus %d: %v", s.Name, busIdx, err)
				continue
			}

			gain := s.GainmV
			if gain == 0 {
				gain = 6144
			}
			dr := s.DRSPS
			if dr == 0 {
				dr = 128
			}

			v, err := readADS1115(bus, ADS1115Opts{
				Addr:    addr,
				Channel: s.Channel,
				Bus:     busIdx,
				GainFSR: gain,
				DRSPS:   dr,
				Clamp0:  true,
			})
			if err != nil {
				//log.Printf("%s error: %v", s.Name, err)
				continue
			}

			out := v * s.R0
			if s.Probe != nil && *s.Probe == "399S9" {
				// Qui nel tuo codice originale sembra ci fosse una conversione V->R e poi R->F.
				// Mantengo la firma, ma se vuoi la conversione completa dimmi come calcoli R da V.
				out, _ = westach399S9_F_from_R(out)
			}

			log.Printf("%s: Vin=%.6fV value=%.6f", s.Name, v, out)
			payload[s.Name] = float32(out)
		}

		_ = RBEMSPostData(configuration.Url, payload)
		time.Sleep(1 * time.Second)
	}
}
