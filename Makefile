export STRATUX_HOME := /opt/stratux

all: ems

ems: main/*.go
	go build -o ems ./main/


install: ems
	mkdir -p $(STRATUX_HOME)/bin
	mkdir -p $(STRATUX_HOME)/www/settings

	# binaries
	cp -f ems $(STRATUX_HOME)/bin/
	chmod 755 $(STRATUX_HOME)/bin/ems
	cp -f ems-chtegt.json $(STRATUX_HOME)/www/settings
	cp debian/ems-chtegt.service /lib/systemd/system
	systemctl daemon-reload
	systemctl enable ems-chtegt.service

clean:
	rm -f ems
