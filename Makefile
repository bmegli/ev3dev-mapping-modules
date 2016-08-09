
.PHONY: all
	$(MAKE) -C ev3drive && cp ev3drive/ev3drive .
	$(MAKE) -C ev3odometry && cp ev3odometry/ev3odometry .
	$(MAKE) -C ev3laser && cp ev3laser/ev3laser .		
	$(MAKE) -C ev3control && cp ev3control/ev3control
