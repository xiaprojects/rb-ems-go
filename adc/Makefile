export STRATUX_HOME := /opt/stratux

all: ems-adc

ems-adc: *.go
	go build -o ems-adc


install: ems-adc
	mkdir -p $(STRATUX_HOME)/bin
	mkdir -p $(STRATUX_HOME)/www/settings

	# binaries
	cp -f ems-adc $(STRATUX_HOME)/bin/
	chmod 755 $(STRATUX_HOME)/bin/ems-adc
	test -f $(STRATUX_HOME)/www/settings/ems-adc.json || cp -f ems-adc.json $(STRATUX_HOME)/www/settings
	cp ems-adc.service /lib/systemd/system
	systemctl daemon-reload
	systemctl enable ems-adc.service

clean:
	rm -f ems-adc
