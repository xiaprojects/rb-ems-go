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
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"runtime"
	"sync"
	"sync/atomic"
	"syscall"
	"time"

	"github.com/warthog618/go-gpiocdev"
)

var (
	ChipPath   = "/dev/gpiochip0"
	APIURL     = "http://127.0.0.1/setEMS"
	payloadKey = "enginerpm"
)

type rpmPayload map[string]int

type rpmSample struct {
	RPM    int
	RawRPM float64
	Pulses uint64
	At     time.Time
}

type configFile struct {
	URL     string   `json:"url"`
	Sensors []sensor `json:"sensors"`
}

type sensor struct {
	Name string `json:"name"`
	Dev  string `json:"dev"`
	Kind string `json:"kind"`
	GPIO int    `json:"gpio"`
}

func main() {
	var (
		ppr        = flag.Int("ppr", 4, "Pulses per revolution")
		window     = flag.Duration("window", 250*time.Millisecond, "Measurement window")
		minDelta   = flag.Int("mindelta", 5, "Send only if changed by >= this amount")
		maxSendGap = flag.Duration("maxgap", 3*time.Second, "Force periodic send even if stable")
		evtBuf     = flag.Int("evbuf", 1024, "GPIO event buffer size")
		debounce   = flag.Duration("debounce", 0, "GPIO debounce (0 disables)")
		alpha      = flag.Float64("alpha", 0.2, "EMA alpha 0..1")
		chanBuf    = flag.Int("chanbuf", 64, "Sample channel buffer")
	)
	flag.Parse()

	if flag.NArg() < 1 {
		log.Fatal("Missing JSON configuration file path")
	}

	cfgPath := flag.Arg(0)
	data, err := os.ReadFile(cfgPath)
	if err != nil {
		log.Fatalf("cannot read config file %q: %v", cfgPath, err)
	}

	var fileCfg configFile
	if err := json.Unmarshal(data, &fileCfg); err != nil {
		log.Fatalf("invalid config file %q: %v", cfgPath, err)
	}

	var sensorCfg *sensor
	for i := range fileCfg.Sensors {
		if fileCfg.Sensors[i].Kind == "gpiopulse" {
			sensorCfg = &fileCfg.Sensors[i]
			break
		}
	}
	if sensorCfg == nil {
		log.Fatal("no gpiopulse sensor found in config")
	}
	if sensorCfg.GPIO <= 0 || sensorCfg.GPIO > 50 {
		log.Fatal("GPIO shall be a number between 1-49")
	}

	if sensorCfg.Dev != "" {
		ChipPath = sensorCfg.Dev
	}
	if fileCfg.URL != "" {
		APIURL = fileCfg.URL
	}
	if sensorCfg.Name != "" {
		payloadKey = sensorCfg.Name
	}

	offset := sensorCfg.GPIO

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	cfg := config{
		ppr:        *ppr,
		window:     *window,
		minDelta:   *minDelta,
		maxSendGap: *maxSendGap,
		evtBuf:     *evtBuf,
		debounce:   *debounce,
		alpha:      *alpha,
		chanBuf:    *chanBuf,
	}

	if err := run(ctx, offset, cfg); err != nil {
		log.Fatal(err)
	}
}

type config struct {
	ppr        int
	window     time.Duration
	minDelta   int
	maxSendGap time.Duration
	evtBuf     int
	debounce   time.Duration
	alpha      float64
	chanBuf    int
}

func run(ctx context.Context, offset int, cfg config) error {
	// Keep the event handler as short as possible: only an atomic increment.
	var pulseCount uint64
	eh := func(evt gpiocdev.LineEvent) {
		atomic.AddUint64(&pulseCount, 1)
	}

	opts := []gpiocdev.LineReqOption{
		gpiocdev.WithRisingEdge,
		gpiocdev.WithEventHandler(eh),
		gpiocdev.WithEventBufferSize(cfg.evtBuf),
	}
	if cfg.debounce > 0 {
		opts = append(opts, gpiocdev.WithDebounce(cfg.debounce))
	}

	line, err := gpiocdev.RequestLine(ChipPath, offset, opts...)
	if err != nil {
		return fmt.Errorf("GPIO RequestLine failed: chip=%s offset=%d err=%w", ChipPath, offset, err)
	}
	defer line.Close()

	// Channel between reader and sender. Buffer helps absorb short bursts.
	samples := make(chan rpmSample, cfg.chanBuf)

	var wg sync.WaitGroup
	wg.Add(2)
	go func() {
		defer wg.Done()
		rpmReadLoop(ctx, &pulseCount, cfg, samples)
	}()
	go func() {
		defer wg.Done()
		rpmSendLoop(ctx, cfg, samples)
	}()

	<-ctx.Done()
	wg.Wait()
	return nil
}

// Goroutine 1: lettura + calcolo RPM (producer)
func rpmReadLoop(ctx context.Context, pulseCount *uint64, cfg config, out chan<- rpmSample) {
	defer close(out)

	// Let the sender have CPU priority under load.
	runtime.Gosched()

	ticker := time.NewTicker(cfg.window)
	defer ticker.Stop()

	alpha := cfg.alpha
	if alpha < 0 {
		alpha = 0
	}
	if alpha > 1 {
		alpha = 1
	}

	emaInit := false
	var ema float64

	for {
		select {
		case <-ctx.Done():
			return
		case t := <-ticker.C:
			pulses := atomic.SwapUint64(pulseCount, 0)
			rawRPM := (float64(pulses) / cfg.window.Seconds()) * (60.0 / float64(cfg.ppr))

			if !emaInit {
				ema = rawRPM
				emaInit = true
			} else {
				ema = alpha*rawRPM + (1.0-alpha)*ema
			}

			smooth := int(ema + 0.5)
			s := rpmSample{RPM: smooth, RawRPM: rawRPM, Pulses: pulses, At: t}

			select {
			case out <- s:
			case <-ctx.Done():
				return
			default:
				// If sender is slow, drop the oldest sample behaviour is preferable.
				// Here we drop the new sample to keep reader lightweight.
			}
		}
	}
}

// Goroutine 2: invio HTTP (consumer)
func rpmSendLoop(ctx context.Context, cfg config, in <-chan rpmSample) {
	// Encourage the sender to run promptly.
	runtime.GOMAXPROCS(runtime.GOMAXPROCS(0))

	client := &http.Client{Timeout: 2 * time.Second}

	lastSent := -1
	lastSentAt := time.Time{}

	for {
		select {
		case <-ctx.Done():
			return
		case s, ok := <-in:
			if !ok {
				return
			}

			needSend := false
			if lastSent < 0 {
				needSend = true
			} else if abs(s.RPM-lastSent) >= cfg.minDelta {
				needSend = true
			} else if !lastSentAt.IsZero() && time.Since(lastSentAt) >= cfg.maxSendGap {
				needSend = true
			}

			if !needSend {
				continue
			}

			fmt.Printf("RPM: %d (raw=%.1f pulses=%d)\n", s.RPM, s.RawRPM, s.Pulses)
			if err := postRPM(client, payloadKey, s.RPM); err != nil {
				fmt.Println("POST error:", err)
				continue
			}
			lastSent = s.RPM
			lastSentAt = time.Now()
		}
	}
}

func postRPM(client *http.Client, field string, rpm int) error {
	body, err := json.Marshal(rpmPayload{field: rpm})
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
