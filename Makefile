DIRS = ev3drive ev3odometry ev3laser ev3control OUTPUT_DIR = bin

all: $(DIRS) ev3init

$(DIRS):
	$(MAKE) -C $@ && cp $@/$@ $(OUTPUT_DIR)/$@

ev3init:
	cp ev3init.sh $(OUTPUT_DIR)/ev3init.sh && chmod +x $(OUTPUT_DIR)/ev3init.sh

clean: 
	$(MAKE) -C ev3drive clean
	$(MAKE) -C ev3odometry clean
	$(MAKE) -C ev3laser clean
	$(MAKE) -C ev3control clean
	rm -rf bin/*
	
.PHONY: clean $(DIRS)
