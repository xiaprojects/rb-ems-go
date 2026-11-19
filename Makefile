export STRATUX_HOME := /opt/stratux

# Multi-module monorepo: main/ builds from the root go.mod, the others from their own.
#   dir:binary
COMPONENTS := main:ems-chtegt adc:ems-adc gpio:ems-gpio rpm:ems-rpm sysmon:rb-sysmon
# Modules holding their own go.mod (the root module covers main/)
MODULES    := . adc gpio rpm sysmon
# alert.go / alert_test.go are duplicated on purpose and must stay identical
DUP_FILES  := alert.go alert_test.go
DUP_DIRS   := main adc gpio

GO       ?= go
DIST     ?= dist
VERSION  := $(shell cat ota/ems.version 2>/dev/null || echo 0)
GOFLAGS_BUILD := -trimpath
TESTFLAGS ?= -race -count=1

# ./... stops at nested go.mod files, so in the root module it only covers main/
.PHONY: all build build-arm64 package test vet fmt fmt-check check-dup ci clean install help

all: ems-chtegt

ems-chtegt: main/*.go
	go build -o ems-chtegt ./main/

help:
	@echo "make              build ems-chtegt (native) - unchanged legacy target"
	@echo "make build        build every component for this machine into $(DIST)/native/"
	@echo "make build-arm64  cross-compile every component for the Pi 5 into $(DIST)/linux-arm64/"
	@echo "make package      build-arm64 + tarball with binaries, services, configs, version"
	@echo "make test         run the unit tests of every module (TESTFLAGS='$(TESTFLAGS)')"
	@echo "make vet          go vet every module (native and linux/arm64)"
	@echo "make fmt-check    fail if any file is not gofmt'ed"
	@echo "make check-dup    fail if the duplicated alert files drifted apart"
	@echo "make ci           fmt-check check-dup vet test build-arm64 (what CI runs)"
	@echo "make install      deploy ems-chtegt on the Pi (root)"

# --- build -----------------------------------------------------------------

build:
	@set -e; for c in $(COMPONENTS); do \
		dir=$${c%%:*}; bin=$${c##*:}; mod=$$dir; [ $$dir = main ] && mod=.; \
		echo "build  $$bin (native)"; \
		(cd $$mod && $(GO) build $(GOFLAGS_BUILD) -o $(CURDIR)/$(DIST)/native/$$bin ./...); \
	done

build-arm64:
	@set -e; for c in $(COMPONENTS); do \
		dir=$${c%%:*}; bin=$${c##*:}; mod=$$dir; [ $$dir = main ] && mod=.; \
		echo "build  $$bin (linux/arm64)"; \
		(cd $$mod && CGO_ENABLED=0 GOOS=linux GOARCH=arm64 \
			$(GO) build $(GOFLAGS_BUILD) -o $(CURDIR)/$(DIST)/linux-arm64/$$bin ./...); \
	done

package: build-arm64
	@set -e; stage=$(DIST)/pkg/rb-ems-$(VERSION); rm -rf $(DIST)/pkg; \
	mkdir -p $$stage/bin $$stage/systemd $$stage/settings; \
	cp $(DIST)/linux-arm64/* $$stage/bin/; \
	cp debian/ems-chtegt.service adc/ems-adc.service gpio/ems-gpio.service \
	   rpm/ems-rpm.service sysmon/rb-sysmon.service $$stage/systemd/; \
	cp ems-chtegt.json adc/ems-adc.json gpio/ems-gpio.json rpm/ems-rpm.json \
	   sysmon/rb-sysmon.json $$stage/settings/; \
	cp ota/ems.version $$stage/; \
	tar -C $(DIST)/pkg -czf $(DIST)/rb-ems-$(VERSION)-linux-arm64.tgz rb-ems-$(VERSION); \
	echo "package $(DIST)/rb-ems-$(VERSION)-linux-arm64.tgz"

# --- verification ----------------------------------------------------------

test:
	@set -e; for m in $(MODULES); do \
		echo "test   $$m"; (cd $$m && $(GO) test $(TESTFLAGS) ./...); \
	done

vet:
	@set -e; for m in $(MODULES); do \
		echo "vet    $$m"; \
		(cd $$m && $(GO) vet ./... && GOOS=linux GOARCH=arm64 $(GO) vet ./...); \
	done

fmt:
	gofmt -w $$(git ls-files '*.go' 2>/dev/null || find . -name '*.go')

fmt-check:
	@out=$$(gofmt -l $$(find . -name '*.go' -not -path './$(DIST)/*')); \
	if [ -n "$$out" ]; then echo "gofmt needed on:"; echo "$$out"; exit 1; fi; echo "fmt    ok"

check-dup:
	@set -e; for f in $(DUP_FILES); do for d in $(DUP_DIRS); do \
		cmp -s main/$$f $$d/$$f || { echo "$$d/$$f differs from main/$$f (keep the copies identical)"; exit 1; }; \
	done; done; echo "dup    ok"

ci: fmt-check check-dup vet test build-arm64

# --- deploy ----------------------------------------------------------------

install: ems-chtegt
	mkdir -p $(STRATUX_HOME)/bin
	mkdir -p $(STRATUX_HOME)/www/settings

	# binaries
	cp -f ems-chtegt $(STRATUX_HOME)/bin/
	chmod 755 $(STRATUX_HOME)/bin/ems-chtegt
	test -f $(STRATUX_HOME)/www/settings/ems-chtegt.json || cp -f ems-chtegt.json $(STRATUX_HOME)/www/settings
	cp debian/ems-chtegt.service /lib/systemd/system
	systemctl daemon-reload
	systemctl enable ems-chtegt.service
	# Version
	cp ota/ems.version $(STRATUX_HOME)

clean:
	rm -f ems-chtegt
	rm -rf $(DIST)
