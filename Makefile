DIRS = ev3drive ev3odometry ev3laser ev3control ev3dead-reconning ev3wifi ccdrive
OUTPUT_DIR = bin

all: $(DIRS) ev3init TestingTheLIDAR TestingTheDriveWithDeadReconning ccinit audio video

$(DIRS):
	$(MAKE) -C $@ && cp $@/$@ $(OUTPUT_DIR)/$@

ev3init:
	cp scripts/ev3init.sh $(OUTPUT_DIR)/ev3init.sh && chmod +x $(OUTPUT_DIR)/ev3init.sh
TestingTheLIDAR:
	cp scripts/TestingTheLIDAR.sh $(OUTPUT_DIR)/TestingTheLIDAR.sh && chmod +x $(OUTPUT_DIR)/TestingTheLIDAR.sh
TestingTheDriveWithDeadReconning:
	cp scripts/TestingTheDriveWithDeadReconning.sh $(OUTPUT_DIR)/TestingTheDriveWithDeadReconning.sh && chmod +x $(OUTPUT_DIR)/TestingTheDriveWithDeadReconning.sh
ccinit:
	cp scripts/ccinit.sh $(OUTPUT_DIR)/ccinit.sh && chmod +x $(OUTPUT_DIR)/ccinit.sh
audio:
	cp scripts/audio.sh $(OUTPUT_DIR)/audio.sh && chmod +x $(OUTPUT_DIR)/audio.sh
video:
	cp scripts/video.sh $(OUTPUT_DIR)/video.sh && chmod +x $(OUTPUT_DIR)/video.sh

clean:
	$(MAKE) -C ev3drive clean
	$(MAKE) -C ev3odometry clean
	$(MAKE) -C ev3laser clean
	$(MAKE) -C ev3control clean
	$(MAKE) -C ev3dead-reconning clean
	$(MAKE) -C ev3wifi clean
	$(MAKE) -C ccdrive clean
	rm -f $(addprefix $(OUTPUT_DIR)/, $(DIRS) ev3init.sh TestingTheLIDAR.sh TestingTheDriveWithDeadReconning.sh ccinit.sh audio.sh video.sh)

.PHONY: clean $(DIRS)
