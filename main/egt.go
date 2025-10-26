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

	Community edition will be free for all builders and personal use as defined by the licensing model
	Dual licensing for commercial agreement is available
	Please join Discord community
*/

package main

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"errors"
	"fmt"
	"log"
	"net/http"
	"time"

	"periph.io/x/conn/v3/physic"
	"periph.io/x/conn/v3/spi"
	"periph.io/x/conn/v3/spi/spireg"
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
	if resp.StatusCode != http.StatusOK && resp.StatusCode != http.StatusCreated {

	}
	return 0
}

// decodeMAX31855 decodes the 32-bit raw reading from a MAX31855
func decodeMAX31855(raw uint32) (float64, float64, string) {
	signMask14 := uint16(0xC000) // runtime var, not const
	signMask12 := uint16(0xF000)
	// Fault detection bits
	fault := (raw & 0x00010000) != 0
	scv := (raw & 0x4) != 0 // short to VCC
	scg := (raw & 0x2) != 0 // short to GND
	oc := (raw & 0x1) != 0  // open circuit

	// ---- Thermocouple temperature (bits 31:18)
	tc := int16(raw >> 18)
	if tc&0x2000 != 0 { // sign bit (bit 13)
		tc |= int16(signMask14) // sign-extend negative values
	}
	thermoC := float64(tc) * 0.25

	// ---- Internal temperature (bits 17:4)
	internal := int16((raw >> 4) & 0xFFF)
	if internal&0x800 != 0 { // sign bit (bit 11)
		internal |= int16(signMask12)
	}
	internalC := float64(internal) * 0.0625

	status := "OK"
	if fault {
		status = "FAULT"
		if scv {
			status += " (Short to VCC)"
		}
		if scg {
			status += " (Short to GND)"
		}
		if oc {
			status += " (Open Circuit)"
		}
	}

	//fmt.Printf("%X %s\n", raw, status)

	return thermoC, internalC, status
}

// --- MAX31855 ---
func readMAX31855(conn spi.Conn) (float64, error) {
	buf := make([]byte, 4)
	if err := conn.Tx(nil, buf); err != nil {
		return 0, err
	}
	raw := binary.BigEndian.Uint32(buf)

	if raw == 0 {
		return -100, nil
	}

	tc, _, _ := decodeMAX31855(raw)
	return tc, nil
}

/*******************************************************************************************/

// MAX31865 registers
const (
	regConfig    = 0x00
	regRTDmsb    = 0x01
	regRTDlsb    = 0x02
	regFaultStat = 0x07
)

// config bits
const (
	cfgVbias      = 0x80 // VBIAS on (bit 7)
	cfgConversion = 0x20 // single-shot conversion (bit 5)
	cfgOneShot    = 0x20
	cfg50HzFilter = 0x01 // example: depends on desired filter / wires
)

// parameters for conversion (change rRef to your board)
const (
	rRef  = 430.0   // reference resistor on board (ohms)
	r0    = 100.0   // PT100
	alpha = 0.00385 // PT100 alpha
)

// readRegister reads count bytes from a MAX31865 over SPI.
// It writes the register address with MSB=0 for read (per datasheet: addr with R/W = 0 for write, 1 for read is used on breakout boards — below we use addr | 0x00/0x80 depending on style).
func readRegister(conn spi.Conn, reg byte, count int) ([]byte, error) {
	// Many MAX31865 designs expect you to send the register address with the MSB = 1 for read.
	// Use 0x00 + reg for write pointer, then read by clocking bytes.
	// First set register pointer (write single byte)
	if err := conn.Tx([]byte{reg & 0x7F}, nil); err != nil {
		return nil, err
	}
	out := make([]byte, count)
	if err := conn.Tx(nil, out); err != nil {
		return nil, err
	}
	return out, nil
}

// writeRegister writes one byte value to a register
func writeRegister(conn spi.Conn, reg byte, val byte) error {
	buf := []byte{reg | 0x80, val} // some breakouts use MSB set to indicate write; if yours uses other scheme remove 0x80
	return conn.Tx(buf, nil)
}

// decodeRTD converts raw RTD MSB/LSB into resistance and temperature
func decodeRTD(msb, lsb byte) (float64, float64) {
	combined := uint16(msb)<<8 | uint16(lsb)
	rtdCounts := combined >> 1 // bit0 is fault
	rtdOhms := float64(rtdCounts) * rRef / 32768.0
	temp := (rtdOhms - r0) / (r0 * alpha) // linear approx
	return rtdOhms, temp
}

// readRTDAndCheck reads RTD, checks fault register, returns temp or error
func readRTDAndCheck(conn spi.Conn) (float64, float64, error) {
	// Enable VBIAS and trigger single conversion
	// Build config: VBIAS=1, One-shot conversion=1, 3-wire bit maybe set if needed
	// For many boards you must follow the exact bit map – adjust as needed for 2/3-wire:
	config := byte(0x80) // VBIAS on
	// set appropriate filter/wire settings here if needed
	if err := writeRegister(conn, regConfig, config); err != nil {
		return 0, 0, err
	}

	// start conversion
	if err := writeRegister(conn, regConfig, config|cfgOneShot); err != nil {
		return 0, 0, err
	}

	// wait conversion (a few ms)
	time.Sleep(65 * time.Millisecond)

	// read RTD MSB/LSB
	b, err := readRegister(conn, regRTDmsb, 2)
	if err != nil {
		return 0, 0, err
	}

	// read fault status
	f, err := readRegister(conn, regFaultStat, 1)
	if err != nil {
		return 0, 0, err
	}
	fault := f[0]

	// evaluate fault bits: bit 0 = high, bit1 = low, bit2 = in-RTD short etc (check datasheet mapping)
	// For MAX31865: faults bits include OC (open circuit), etc. If any fault bit set -> error
	if fault != 0 {
		return 0, 0, fmt.Errorf("MAX31865 fault register: 0x%02X", fault)
	}

	rtdOhms, temp := decodeRTD(b[0], b[1])

	fmt.Printf("%02X%02X %fOhm %.1f°C\n", b[0], b[1], rtdOhms, temp)

	// Additional heuristic: detect absurdly large or small resistance that indicates open-circuit
	if rtdOhms > (rRef*2.5) || rtdOhms < 0.1 {
		return rtdOhms, temp, errors.New("RTD reading out of range, likely open circuit")
	}

	return rtdOhms, temp, nil
}

/*
// --- MAX31865 (PT100/RTD) ---

	func readMAX31865(conn spi.Conn) (float64, error) {
		// Configuration register: 0x80 = write, 0x00 = config register
		// Continuous conversion, 60Hz filter, 3-wire disabled (bit pattern 0xC2)
		tx := []byte{0x80, 0xC2}
		if err := conn.Tx(tx, nil); err != nil {
			return 0, err
		}
		time.Sleep(100 * time.Millisecond)

		// Read 2 bytes starting at address 0x01 (MSB first)
		tx = []byte{0x01, 0x00, 0x00}
		rx := make([]byte, 3)
		if err := conn.Tx(tx, rx); err != nil {
			return 0, err
		}

		fmt.Printf("%02X%02X%02X\n", rx[0], rx[1], rx[2])
		rt := uint16(rx[1])<<8 | uint16(rx[2])
		// Simplified conversion: RTD to temperature (PT100, α=0.00385)
		temp := (float64(rt)/32768.0*430.0 - 100.0) // rough estimate

		return temp, nil
	}
*/
func ems_egt_cht_sample_spi() {
	if _, err := host.Init(); err != nil {
		log.Fatal(err)
	}

	type sensor struct {
		name    string
		dev     string
		kind    string
		gpio    int
		spiMode int
	}

	/*
		In this example we are using the Raspberry Pi SPI interfaces to access to the EGT-CHT
		You can have up to 2 SPI with 2+3 CS => 5
		If you want moure, you can use GPIO interfaces
	*/

	sensors := []sensor{
		{"egt1", "/dev/spidev0.0", "MAX31855", 0, 0},
		{"egt2", "/dev/spidev0.1", "MAX31855", 0, 0},
		{"cht1", "/dev/spidev1.0", "MAX31865", 0, 1},
		{"cht2", "/dev/spidev1.1", "MAX31865", 0, 1},
		{"cht3", "/dev/spidev1.2", "MAX31865", 0, 1},
	}

	// infinite loop as service
	for {
		payload := make(map[string]float32)

		for _, s := range sensors {
			spiDev, err := spireg.Open(s.dev)
			if err != nil {
				log.Printf("[%s] open error: %v\n", s.name, err)
				continue
			}
			conn, err := spiDev.Connect(1*physic.MegaHertz, spi.Mode(s.spiMode), 8)
			if err != nil {
				log.Printf("[%s] connect error: %v\n", s.name, err)
				spiDev.Close()
				continue
			}

			var temp float64
			switch s.kind {
			case "MAX31855":
				temp, err = readMAX31855(conn)
			case "MAX31865":
				//temp, err = readMAX31865(conn)
				_, temp, err = readRTDAndCheck(conn)
			}

			if err != nil {
				fmt.Printf("%s error: %v\n", s.name, err)
			} else {
				if temp < -10 {

				} else {
					fmt.Printf("%s: %.2f °C\n", s.name, temp)
					payload[s.name] = float32(temp)
				}
			}
			spiDev.Close()
		}

		RBEMSPostData("http://localhost/setEMS", payload)

		time.Sleep(1 * time.Second)
	}
}
