export STRATUX_HOME := /opt/stratux

all: ems-gpio

ems-gpio: *.go
	go build -o ems-gpio


install: ems-gpio
	mkdir -p $(STRATUX_HOME)/bin
	mkdir -p $(STRATUX_HOME)/www/settings

	# binaries
	cp -f ems-gpio $(STRATUX_HOME)/bin/
	chmod 755 $(STRATUX_HOME)/bin/ems-gpio
	cp -f ems-gpio.json $(STRATUX_HOME)/www/settings
	cp ems-gpio.service /lib/systemd/system
	systemctl daemon-reload
	systemctl enable ems-gpio.service

clean:
	rm -f ems-gpio
