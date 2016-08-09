DIRS = ev3drive ev3odometry ev3laser ev3control

$(DIRS):
	$(MAKE) -C $@
all: $(DIRS)
clean: MAKE = make clean
clean: $(DIRS)
.PHONY: clean $(DIRS)