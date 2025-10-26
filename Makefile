export STRATUX_HOME := /opt/stratux/

all: ems

ems: main/*.go
	go build -o ems ./main/


optinstall: ems
	mkdir -p $(STRATUX_HOME)/bin

	# binaries
	cp -f ems $(STRATUX_HOME)/bin/
	chmod 755 $(STRATUX_HOME)/bin/ems

clean:
	rm -f ems
