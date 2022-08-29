filename = top
pcf_file = ./io.pcf

top.bin:
	#yosys -p 'synth_ice40 -top top -json $(filename).json' $(filename).v
	yosys -p 'synth_ice40 -noflatten -top top -json $(filename).json' $(filename).v
	nextpnr-ice40 --lp1k --package qn84 --json $(filename).json --pcf $(pcf_file) --asc $(filename).asc
	icepack $(filename).asc $(filename).bin

clean:
	rm -f $(filename).json $(filename).asc $(filename).bin
