export STRATUX_HOME := /opt/stratux

all: rb-sysmon

rb-sysmon: *.go
	go build -o rb-sysmon


install: rb-sysmon
	mkdir -p $(STRATUX_HOME)/bin
	mkdir -p $(STRATUX_HOME)/www/settings

	# binaries
	cp -f rb-sysmon $(STRATUX_HOME)/bin/
	chmod 755 $(STRATUX_HOME)/bin/rb-sysmon
	cp -f rb-sysmon.json $(STRATUX_HOME)/www/settings
	cp rb-sysmon.service /lib/systemd/system
	systemctl daemon-reload
	systemctl enable rb-sysmon.service

clean:
	rm -f rb-sysmon
