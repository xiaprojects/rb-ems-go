export STRATUX_HOME := /opt/stratux

all: ems-rpm

ems-rpm: *.go
	go build -o ems-rpm


install: ems-rpm
	mkdir -p $(STRATUX_HOME)/bin
	mkdir -p $(STRATUX_HOME)/www/settings

	# binaries
	cp -f ems-rpm $(STRATUX_HOME)/bin/
	chmod 755 $(STRATUX_HOME)/bin/ems-rpm
	test -f $(STRATUX_HOME)/www/settings/ems-rpm.json || cp -f ems-rpm.json $(STRATUX_HOME)/www/settings
	cp ems-rpm.service /lib/systemd/system
	systemctl daemon-reload
	systemctl enable ems-rpm.service

clean:
	rm -f ems-rpm
