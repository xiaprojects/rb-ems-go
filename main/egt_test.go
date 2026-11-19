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
Hardware-free tests for the SPI temperature drivers.

Expected values come from the datasheet example tables (MAX31855 Table 2/3,
MAX31856 Table 5, MAX31865 RTD transfer function), not from the code itself,
so a change in the decoding math shows up here.
The SPI bus is replaced by fakeSpiConn: the real read functions run unchanged.
*/

import (
	"errors"
	"math"
	"strings"
	"testing"

	"periph.io/x/conn/v3/gpio"
	"periph.io/x/conn/v3/spi"
)

// MAX31855 frame: D31..D18 thermocouple (14 bit, 0.25 °C), D16 fault,
// D15..D4 internal (12 bit, 0.0625 °C), D2 SCV, D1 SCG, D0 OC
func max31855Frame(tc14 uint32, internal12 uint32, faultBits uint32) uint32 {
	return (tc14&0x3FFF)<<18 | (internal12&0xFFF)<<4 | faultBits
}

func TestDecodeMAX31855Datasheet(t *testing.T) {
	cases := []struct {
		name       string
		tc14       uint32
		internal12 uint32
		wantTC     float64
		wantInt    float64
	}{
		{"+1600.00", 0x1900, 0x000, 1600.00, 0},
		{"+1000.00", 0x0FA0, 0x000, 1000.00, 0},
		{"+100.75", 0x0193, 0x000, 100.75, 0},
		{"+25.00", 0x0064, 0x000, 25.00, 0},
		{"0.00", 0x0000, 0x000, 0.00, 0},
		{"-0.25", 0x3FFF, 0x000, -0.25, 0},
		{"-1.00", 0x3FFC, 0x000, -1.00, 0},
		{"-250.00", 0x3C18, 0x000, -250.00, 0},
		{"internal +127.0000", 0x0000, 0x7F0, 0, 127.0},
		{"internal +100.5625", 0x0000, 0x649, 0, 100.5625},
		{"internal +25.0000", 0x0000, 0x190, 0, 25.0},
		{"internal -0.0625", 0x0000, 0xFFF, 0, -0.0625},
		{"internal -1.0000", 0x0000, 0xFF0, 0, -1.0},
		{"internal -20.0000", 0x0000, 0xEC0, 0, -20.0},
		{"internal -55.0000", 0x0000, 0xC90, 0, -55.0},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			tc, internal, status := decodeMAX31855(max31855Frame(c.tc14, c.internal12, 0))
			if tc != c.wantTC {
				t.Errorf("thermocouple = %v, want %v", tc, c.wantTC)
			}
			if internal != c.wantInt {
				t.Errorf("internal = %v, want %v", internal, c.wantInt)
			}
			if status != "OK" {
				t.Errorf("status = %q, want OK", status)
			}
		})
	}
}

func TestDecodeMAX31855Faults(t *testing.T) {
	cases := []struct {
		name string
		bits uint32
		want string
	}{
		{"open circuit", 0x1, "FAULT (Open Circuit)"},
		{"short to GND", 0x2, "FAULT (Short to GND)"},
		{"short to VCC", 0x4, "FAULT (Short to VCC)"},
		{"fault flag only", 0x0, "FAULT"},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			raw := max31855Frame(0x0064, 0x190, 0x00010000|c.bits)
			tc, _, status := decodeMAX31855(raw)
			if status != c.want {
				t.Errorf("status = %q, want %q", status, c.want)
			}
			if tc != -500 {
				t.Errorf("thermocouple on fault = %v, want -500", tc)
			}
		})
	}
}

func TestDecodeRTD(t *testing.T) {
	// RTD code = R_RTD * 32768 / R_REF, sent left aligned with the fault bit in D0
	code := func(ohms float64) (byte, byte) {
		c := uint16(math.Round(ohms*32768/rRef)) << 1
		return byte(c >> 8), byte(c)
	}
	cases := []struct {
		name     string
		ohms     float64
		r0       float64
		wantTemp float64
	}{
		{"PT100 0 °C", 100.0, 100, 0},
		{"PT100 100 °C", 138.5, 100, 100},
		{"PT100 -40 °C", 84.6, 100, -40},
		{"r0 < 1 falls back to 100 Ω", 138.5, 0, 100},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			msb, lsb := code(c.ohms)
			ohms, temp := decodeRTD(msb, lsb, c.r0)
			if math.Abs(ohms-c.ohms) > 0.02 { // one LSB = 430/32768 = 0.013 Ω
				t.Errorf("ohms = %v, want %v", ohms, c.ohms)
			}
			if math.Abs(temp-c.wantTemp) > 0.1 {
				t.Errorf("temp = %v, want %v", temp, c.wantTemp)
			}
		})
	}
	// The fault bit (D0 of LSB) shall not change the reading
	_, a := decodeRTD(0x3B, 0x88, 100)
	_, b := decodeRTD(0x3B, 0x89, 100)
	if a != b {
		t.Errorf("fault bit changed the temperature: %v vs %v", a, b)
	}
}

/*************** fake SPI ****************/

// fakeSpiConn records every transaction and answers through respond.
// The embedded interface is nil: only Tx is used by the drivers.
type fakeSpiConn struct {
	spi.Conn
	writes  [][]byte
	respond func(w, r []byte) error
}

func (f *fakeSpiConn) Tx(w, r []byte) error {
	f.writes = append(f.writes, append([]byte(nil), w...))
	if f.respond != nil {
		return f.respond(w, r)
	}
	return nil
}

func newFakeBus(respond func(w, r []byte) error) (*SpiBus, *fakeSpiConn) {
	c := &fakeSpiConn{respond: respond}
	return &SpiBus{conn: c}, c
}

func TestReadMAX31855(t *testing.T) {
	frame := func(raw uint32) func(w, r []byte) error {
		return func(w, r []byte) error {
			r[0], r[1], r[2], r[3] = byte(raw>>24), byte(raw>>16), byte(raw>>8), byte(raw)
			return nil
		}
	}

	b, _ := newFakeBus(frame(max31855Frame(0x0193, 0x190, 0)))
	if tc, err := readMAX31855(b, false, nil); err != nil || tc != 100.75 {
		t.Errorf("good frame: got %v, %v; want 100.75, nil", tc, err)
	}

	for name, raw := range map[string]uint32{
		"all zero (chip missing)": 0x00000000,
		"all ones (bus floating)": 0xFFFFFFFF,
		"open circuit":            max31855Frame(0, 0x190, 0x00010001),
	} {
		b, _ := newFakeBus(frame(raw))
		if tc, err := readMAX31855(b, false, nil); err == nil || tc != -500 {
			t.Errorf("%s: got %v, %v; want -500 and an error", name, tc, err)
		}
	}

	b, _ = newFakeBus(func(w, r []byte) error { return errors.New("bus error") })
	if tc, err := readMAX31855(b, false, nil); err == nil || tc != -500 {
		t.Errorf("bus error: got %v, %v; want -500 and an error", tc, err)
	}
}

// fakeMAX31856 emulates the register file: CR0, CR1 and the 3 byte LTC result
func fakeMAX31856(ltc uint32) (*SpiBus, *fakeSpiConn, *[16]byte) {
	var regs [16]byte
	regs[max31856CR1] = 0x03 // power-on default: K type, 1 sample
	regs[max31856TCMSB], regs[max31856TCMSB+1], regs[max31856TCMSB+2] = byte(ltc>>16), byte(ltc>>8), byte(ltc)
	b, c := newFakeBus(func(w, r []byte) error {
		addr := w[0] & 0x7F
		if w[0]&0x80 != 0 { // write
			copy(regs[addr:], w[1:])
			return nil
		}
		copy(r[1:], regs[addr:])
		return nil
	})
	return b, c, &regs
}

func TestReadMAX31856Datasheet(t *testing.T) {
	// LTC code is 19 bit signed, left aligned in 24 bit, LSB 0.0078125 °C
	cases := []struct {
		name string
		ltc  uint32
		want float64
	}{
		{"+1600", 0x640000, 1600},
		{"+1000", 0x3E8000, 1000},
		{"+100.9375", 0x064F00, 100.9375},
		{"+25", 0x019000, 25},
		{"+0.0625", 0x000100, 0.0625},
		{"+0.0078125", 0x000020, 0.0078125},
		{"-0.0078125", 0xFFFFE0, -0.0078125},
		{"-0.25", 0xFFFC00, -0.25},
		{"-1", 0xFFF000, -1},
		{"-250", 0xF06000, -250},
	}
	for _, c := range cases {
		t.Run(c.name, func(t *testing.T) {
			b, _, _ := fakeMAX31856(c.ltc)
			got, err := readMAX31856(b, false, nil, 0x03)
			if err != nil {
				t.Fatalf("unexpected error: %v", err)
			}
			if got != c.want {
				t.Errorf("got %v, want %v", got, c.want)
			}
		})
	}
}

func TestReadMAX31856Framing(t *testing.T) {
	b, c, regs := fakeMAX31856(0x019000)
	if _, err := readMAX31856(b, false, nil, 0x07); err != nil { // T type
		t.Fatal(err)
	}
	if regs[max31856CR0] != 0x80 {
		t.Errorf("CR0 = 0x%02X, want 0x80 (continuous conversion)", regs[max31856CR0])
	}
	if regs[max31856CR1] != 0x07 {
		t.Errorf("CR1 = 0x%02X, want 0x07 (T type, averaging kept)", regs[max31856CR1])
	}
	// write CR0, read CR1, write CR1, read 3 bytes from LTCBH
	wantFirst := []byte{0x80, 0x01, 0x81, 0x0C}
	if len(c.writes) != len(wantFirst) {
		t.Fatalf("got %d transactions, want %d", len(c.writes), len(wantFirst))
	}
	for i, w := range c.writes {
		if w[0] != wantFirst[i] {
			t.Errorf("transaction %d address byte = 0x%02X, want 0x%02X", i, w[0], wantFirst[i])
		}
	}
	if n := len(c.writes[3]); n != 4 {
		t.Errorf("LTC read length = %d, want 4 (address + 3 bytes)", n)
	}

	for name, ltc := range map[string]uint32{"all zero": 0x000000, "0x7FFFFF": 0x7FFFFF} {
		b, _, _ := fakeMAX31856(ltc)
		if v, err := readMAX31856(b, false, nil, 0x03); err == nil || v != -500 {
			t.Errorf("%s: got %v, %v; want -500 and an error", name, v, err)
		}
	}
}

func TestReadRTDAndCheckFraming(t *testing.T) {
	var regs [8]byte
	regs[regRTDmsb], regs[regRTDlsb] = 0x3B, 0x88 // ~100 Ω
	b, c := newFakeBus(func(w, r []byte) error {
		addr := w[0] & 0x7F
		if w[0]&0x80 != 0 {
			copy(regs[addr:], w[1:])
			return nil
		}
		copy(r[1:], regs[addr:])
		return nil
	})
	ohms, temp, err := readRTDAndCheck(b, false, nil, 100)
	if err != nil {
		t.Fatal(err)
	}
	if regs[regConfig] != 0xB2 {
		t.Errorf("config = 0x%02X, want 0xB2 (VBIAS, 1-shot, 3-wire, fault clear)", regs[regConfig])
	}
	if c.writes[0][0] != 0x80 || c.writes[1][0] != 0x01 {
		t.Errorf("unexpected framing: %X", c.writes)
	}
	if math.Abs(ohms-100) > 0.02 || math.Abs(temp) > 0.1 {
		t.Errorf("got %v Ω %v °C, want ~100 Ω ~0 °C", ohms, temp)
	}
}

func TestSpiBusKernelCS(t *testing.T) {
	b, _ := newFakeBus(nil)
	called := false
	if err := b.Do(false, nil, func() error { called = true; return nil }); err != nil || !called {
		t.Errorf("Do(kernel) did not run fn: called=%v err=%v", called, err)
	}
	want := errors.New("boom")
	if err := b.Do(false, nil, func() error { return want }); !errors.Is(err, want) {
		t.Errorf("Do(kernel) error = %v, want %v", err, want)
	}
}

// fakePin records the levels driven on a manual chip select line
type fakePin struct {
	gpio.PinOut
	levels *[]string
}

func (p fakePin) Out(l gpio.Level) error {
	*p.levels = append(*p.levels, l.String())
	return nil
}

func TestSpiBusManualCS(t *testing.T) {
	var events []string
	cs := fakePin{levels: &events}
	b, _ := newFakeBus(nil)
	want := errors.New("boom")
	err := b.Do(true, cs, func() error {
		events = append(events, "Tx")
		return want
	})
	if !errors.Is(err, want) {
		t.Errorf("error = %v, want %v", err, want)
	}
	// CS shall be LOW for the whole frame and released even when Tx fails
	got := strings.Join(events, ",")
	if got != "High,Low,Tx,High" {
		t.Errorf("CS sequence = %s, want High,Low,Tx,High", got)
	}
}
