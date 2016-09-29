DIRS = ev3drive ev3odometry ev3laser ev3control ev3motors
OUTPUT_DIR = bin

all: $(DIRS) ev3init TestingTheLIDAR

$(DIRS):
	$(MAKE) -C $@ && cp $@/$@ $(OUTPUT_DIR)/$@

ev3init:
	cp scripts/ev3init.sh $(OUTPUT_DIR)/ev3init.sh && chmod +x $(OUTPUT_DIR)/ev3init.sh
TestingTheLIDAR:
	cp scripts/TestingTheLIDAR.sh $(OUTPUT_DIR)/TestingTheLIDAR.sh && chmod +x $(OUTPUT_DIR)/TestingTheLIDAR.sh
	
clean: 
	$(MAKE) -C ev3drive clean
	$(MAKE) -C ev3odometry clean
	$(MAKE) -C ev3laser clean
	$(MAKE) -C ev3control clean
	$(MAKE) -C ev3motors clean	
	rm -rf bin/*
	
.PHONY: clean $(DIRS)
