DIRS = ev3drive ev3odometry ev3laser ev3control ev3dead-reconning ev3wifi
OUTPUT_DIR = bin

all: $(DIRS) ev3init TestingTheLIDAR TestingTheDriveWithDeadReconning

$(DIRS):
	$(MAKE) -C $@ && cp $@/$@ $(OUTPUT_DIR)/$@

ev3init:
	cp scripts/ev3init.sh $(OUTPUT_DIR)/ev3init.sh && chmod +x $(OUTPUT_DIR)/ev3init.sh
TestingTheLIDAR:
	cp scripts/TestingTheLIDAR.sh $(OUTPUT_DIR)/TestingTheLIDAR.sh && chmod +x $(OUTPUT_DIR)/TestingTheLIDAR.sh
TestingTheDriveWithDeadReconning:
	cp scripts/TestingTheDriveWithDeadReconning.sh $(OUTPUT_DIR)/TestingTheDriveWithDeadReconning.sh && chmod +x $(OUTPUT_DIR)/TestingTheDriveWithDeadReconning.sh
		
clean: 
	$(MAKE) -C ev3drive clean
	$(MAKE) -C ev3odometry clean
	$(MAKE) -C ev3laser clean
	$(MAKE) -C ev3control clean
	$(MAKE) -C ev3dead-reconning clean
	$(MAKE) -C ev3wifi clean
	rm -f $(addprefix $(OUTPUT_DIR)/, $(DIRS) ev3init.sh TestingTheLIDAR.sh TestingTheDriveWithDeadReconning.sh)	
		
.PHONY: clean $(DIRS)
