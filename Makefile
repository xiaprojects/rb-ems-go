export STRATUX_HOME := /opt/stratux

all: ems-chtegt

ems-chtegt: main/*.go
	go build -o ems-chtegt ./main/


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
	cp ems.version $(STRATUX_HOME)

clean:
	rm -f ems-chtegt
