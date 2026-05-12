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

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"io/ioutil"
	"log"
	"net/http"
	"os"
	"sync"
	"time"

	"periph.io/x/conn/v3/gpio"
	"periph.io/x/conn/v3/gpio/gpioreg"
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
	return 0
}

/*************** BUS + CS MODE ****************/

type SpiBus struct {
	conn  spi.Conn
	close func() error
	mu    sync.Mutex
}

func (b *SpiBus) Close() {
	if b.close != nil {
		_ = b.close()
	}
}

func (b *SpiBus) doKernel(fn func() error) error {
	// Anche in kernel-CS serializzo: evita interleaving quando più sensori condividono lo stesso /dev/spidevX.Y
	b.mu.Lock()
	defer b.mu.Unlock()
	return fn()
}

func (b *SpiBus) doManual(cs gpio.PinOut, fn func() error) error {
	b.mu.Lock()
	defer b.mu.Unlock()

	_ = cs.Out(gpio.High) // safety
	if err := cs.Out(gpio.Low); err != nil {
		return err
	}
	// CS deve restare LOW durante tutto il frame SPI [web:10]
	err := fn()
	_ = cs.Out(gpio.High)
	return err
}

func (b *SpiBus) Do(manual bool, cs gpio.PinOut, fn func() error) error {
	if manual {
		return b.doManual(cs, fn)
	}
	return b.doKernel(fn)
}

/*************** MAX31855 ****************/

func decodeMAX31855(raw uint32) (float64, float64, string) {
	signMask14 := uint16(0xC000)
	signMask12 := uint16(0xF000)

	fault := (raw & 0x00010000) != 0
	scv := (raw & 0x4) != 0
	scg := (raw & 0x2) != 0
	oc := (raw & 0x1) != 0

	tc := int16(raw >> 18)
	if tc&0x2000 != 0 {
		tc |= int16(signMask14)
	}
	thermoC := float64(tc) * 0.25

	internal := int16((raw >> 4) & 0xFFF)
	if internal&0x800 != 0 {
		internal |= int16(signMask12)
	}
	internalC := float64(internal) * 0.0625

	status := "OK"
	if fault {
		status = "FAULT"
		thermoC = -500
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
	return thermoC, internalC, status
}

func readMAX31855(b *SpiBus, manual bool, cs gpio.PinOut) (float64, error) {
	var buf [4]byte
	err := b.Do(manual, cs, func() error {
		return b.conn.Tx(nil, buf[:])
	})
	if err != nil {
		return -500, err
	}

	raw := binary.BigEndian.Uint32(buf[:])
	// Check for disconnected/not-connected chip: raw == 0 or all 0xFF (no response)
	if raw == 0 {
		err = fmt.Errorf("MAX31855 error: chip not responding (raw: 0x00000000)")
		return -500, err
	}
	if raw == 0xFFFFFFFF {
		err = fmt.Errorf("MAX31855 error: no response detected (raw: 0xFFFFFFFF)")
		return -500, err
	}
	tc, _, status := decodeMAX31855(raw)
	// Log fault status for debugging
	if status != "OK" {
		err = fmt.Errorf("MAX31855 fault: %s (raw: 0x%08X)", status, raw)
	}
	return tc, err
}

/*************** MAX31856 ****************/

const (
	max31856CR0   = 0x00
	max31856CR1   = 0x01
	max31856TCMSB = 0x0C
)

// Framing MAX31856: bit7=0 read, bit7=1 write [web:10][web:45]
func max31856ReadReg(b *SpiBus, manual bool, cs gpio.PinOut, reg byte, n int) ([]byte, error) {
	w := make([]byte, 1+n)
	r := make([]byte, 1+n)
	w[0] = reg & 0x7F

	err := b.Do(manual, cs, func() error {
		return b.conn.Tx(w, r)
	})
	if err != nil {
		return nil, err
	}
	return r[1:], nil
}

func max31856WriteReg(b *SpiBus, manual bool, cs gpio.PinOut, reg byte, data ...byte) error {
	w := append([]byte{reg | 0x80}, data...) // write bit7=1 [web:10]
	return b.Do(manual, cs, func() error {
		return b.conn.Tx(w, nil)
	})
}

func readMAX31856(b *SpiBus, manual bool, cs gpio.PinOut, thermocoupleType byte) (float64, error) {
	// Enable continuous conversion (CR0 bit7) [web:10]
	if err := max31856WriteReg(b, manual, cs, max31856CR0, 0x80); err != nil {
		return -500, err
	}

	// Set thermocouple type in CR1 (low nibble)
	cr1, err := max31856ReadReg(b, manual, cs, max31856CR1, 1)
	if err != nil {
		return -500, err
	}
	newCR1 := (cr1[0] &^ 0x0F) | (thermocoupleType & 0x0F)
	if err := max31856WriteReg(b, manual, cs, max31856CR1, newCR1); err != nil {
		return -500, err
	}

	time.Sleep(200 * time.Millisecond)

	d, err := max31856ReadReg(b, manual, cs, max31856TCMSB, 3)
	if err != nil {
		return -500, err
	}
	raw := int32(d[0])<<16 | int32(d[1])<<8 | int32(d[2])

	// Check for disconnected/not-connected chip: all 1s (0x7FFFFF after shift) or all 0s
	if raw == 0 {
		err = fmt.Errorf("MAX31856 error: chip not responding (raw: 0x000000)")
		return -500, err
	}
	if raw == 0x7FFFFF {
		err = fmt.Errorf("MAX31856 error: no response detected (raw: 0x7FFFFF)")
		return -500, err
	}

	// 19-bit signed in [23:5], LSB=2^-7=0.0078125°C [web:10]
	code19 := raw >> 5
	if (code19 & (1 << 18)) != 0 {
		code19 -= 1 << 19
	}
	return float64(code19) * 0.0078125, nil
}

/*************** MAX31865 ****************/

const (
	regConfig = 0x00
	regRTDmsb = 0x01
	regRTDlsb = 0x02
)

const (
	rRef  = 430.0
	r0    = 100.0
	alpha = 0.00385
)

func readRegisterMAX31865(b *SpiBus, manual bool, cs gpio.PinOut, reg byte, count int) ([]byte, error) {
	tx := make([]byte, count+1)
	rx := make([]byte, count+1)
	tx[0] = reg & 0x7F

	err := b.Do(manual, cs, func() error {
		return b.conn.Tx(tx, rx)
	})
	if err != nil {
		return nil, err
	}
	return rx[1:], nil
}

func writeRegisterMAX31865(b *SpiBus, manual bool, cs gpio.PinOut, reg byte, val byte) error {
	buf := []byte{reg | 0x80, val}
	return b.Do(manual, cs, func() error {
		return b.conn.Tx(buf, nil)
	})
}

func decodeRTD(msb, lsb byte, r0In float64) (float64, float64) {
	if r0In < 1 {
		r0In = r0
	}
	combined := uint16(msb)<<8 | uint16(lsb)
	rtdCounts := combined >> 1
	rtdOhms := float64(rtdCounts) * rRef / 32768.0
	temp := (rtdOhms - r0In) / (r0In * alpha)
	return rtdOhms, temp
}

func readRTDAndCheck(b *SpiBus, manual bool, cs gpio.PinOut, r0In float64) (float64, float64, error) {
	config := byte(0x80 | 0x10 | 0x20 | 0x02)
	if err := writeRegisterMAX31865(b, manual, cs, regConfig, config); err != nil {
		return 0, 0, err
	}
	time.Sleep(65 * time.Millisecond)

	d, err := readRegisterMAX31865(b, manual, cs, regRTDmsb, 2)
	if err != nil {
		return 0, 0, err
	}
	rtdOhms, temp := decodeRTD(d[0], d[1], r0In)
	return rtdOhms, temp, nil
}

/*************** MAIN SERVICE ****************/

func ems_egt_cht_sample_spi(configFilename string) {
	if _, err := host.Init(); err != nil {
		log.Fatal(err)
	}

	type sensor struct {
		Name     string  `json:"name"`
		Dev      string  `json:"dev"`
		Kind     string  `json:"kind"`
		Gpio     int     `json:"gpio"`    // >0 => CS manuale su GPIO{n}, 0 => CS kernel [web:136]
		SpiMode  int     `json:"spiMode"` // spi.Mode(0..3)
		R0       float64 `json:"r0"`
		R0Offset float64 `json:"r0Offset"`
	}

	type config struct {
		Url     string   `json:"url"`
		Speed   int      `json:"busSpeed"` // Hz, 0 = default 1MHz
		Sensors []sensor `json:"sensors"`
	}

	file, err := os.Open(configFilename)
	if err != nil {
		panic(err)
	}
	defer file.Close()

	data, _ := ioutil.ReadAll(file)
	var configuration config
	if err := json.Unmarshal(data, &configuration); err != nil {
		panic(err)
	}
	sensors := configuration.Sensors

	// cache bus per /dev/spidevX.Y
	buses := map[string]*SpiBus{}

	getBus := func(dev string, mode spi.Mode, speed int) (*SpiBus, error) {
		if b, ok := buses[dev]; ok {
			return b, nil
		}

		if speed <= 0 {
			speed = 1 * int(physic.MegaHertz)
		}

		p, err := spireg.Open(dev) // spidev node: CS kernel associato al dev [web:136]
		if err != nil {
			return nil, err
		}

		c, err := p.Connect(physic.Frequency(speed), mode, 8)
		if err != nil {
			_ = p.Close()
			return nil, err
		}

		b := &SpiBus{
			conn:  c,
			close: p.Close,
		}
		buses[dev] = b
		return b, nil
	}

	for {
		payload := make(map[string]float32)

		for _, s := range sensors {
			b, err := getBus(s.Dev, spi.Mode(s.SpiMode), configuration.Speed)
			if err != nil {
				log.Printf("[%s] open/connect error: %v\n", s.Name, err)
				continue
			}

			manual := s.Gpio > 0
			var cs gpio.PinOut
			if manual {
				cs = gpioreg.ByName(fmt.Sprintf("GPIO%d", s.Gpio))
				if cs == nil {
					log.Printf("[%s] GPIO%d non trovato\n", s.Name, s.Gpio)
					continue
				}
				err = cs.Out(gpio.High) // deselect
				if err != nil {
					log.Printf("%s error gpio.High: %v\n", s.Name, err)
				}
			}

			time.Sleep(100 * time.Millisecond)

			var temp float64

			switch s.Kind {
			case "MAX31855":
				temp, err = readMAX31855(b, manual, cs)

			case "MAX31865":
				_, temp, err = readRTDAndCheck(b, manual, cs, 100)

			case "MAX31856J":
				temp, err = readMAX31856(b, manual, cs, 0x02) // J
			case "MAX31856K":
				temp, err = readMAX31856(b, manual, cs, 0x03) // K
			case "MAX31856":
				temp, err = readMAX31856(b, manual, cs, 0x03) // default K
			case "MAX31856T":
				temp, err = readMAX31856(b, manual, cs, 0x07) // T

			default:
				err = fmt.Errorf("kind non supportato: %s", s.Kind)
				log.Printf("[%s] unsupported sensor type: %s", s.Name, s.Kind)
			}

			if err != nil {
				log.Printf("%s error: %v\n", s.Name, err)
				continue
			}

			temp = temp*s.R0 + s.R0Offset

			if temp > -500 {
				log.Printf("%s: %.2f °C\n", s.Name, temp)
				payload[s.Name] = float32(temp)
			}
		}

		_ = RBEMSPostData(configuration.Url, payload)
		time.Sleep(1 * time.Second)
	}
}
