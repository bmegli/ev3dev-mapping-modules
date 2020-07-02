# ev3dev-mapping-modules
A bunch of small programs feeding data to [ev3dev-mapping-ui]

For high-level project overview visit [ev3dev-mapping web page](http://www.ev3dev.org/projects/2016/08/07/Mapping/)

For project meta-repository visit [ev3dev-mapping](https://github.com/bmegli/ev3dev-mapping)

![Alt text](https://user-images.githubusercontent.com/9095769/28492434-20d8cfa8-6f04-11e7-8f9e-4415bc809063.png "ev3dev-mapping-modules screenshot")

## Requirements

### Software

ev3dev-mapping-modules works on [ev3dev](http://www.ev3dev.org/) operating system (jessie - yes, strech - no).

ev3dev-mapping-modules works with [ev3dev-mapping-ui].

### Hardware

ev3dev-mapping-modules works on [Lego Mindstorms EV3] with any combination of hardware in the following table. 

| Hardware                      | ev3dev-mapping-modules module| ev3dev-mapping-ui component | Purpose                            |
| ------------------------------|------------------------------|-----------------------------|------------------------------------|
| 2 x EV3 Large Servo Motor     | ev3drive, ev3odometry        | Drive, Odometry             | motor control, position estimation |
| above + [CruizCore] gyroscope | ev3drive, ev3dead-reconning  | Drive, DeadReconning        | motor control, position estimation |
| [WiFi dongle]                 | ev3wifi                      | WiFi                        | WiFi signal strengh monitoring     |
| [Neato XV11 Lidar]            | ev3laser                     | Laser                       | environment scanning                                   |

[Lego Mindstorms EV3]: https://www.lego.com/en-us/mindstorms/products/mindstorms-ev3-31313
[CruizCore]: http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-jessie/sensor_data.html#microinfinity-digital-gyroscope-and-accelerometer
[WiFi dongle]: http://www.ev3dev.org/docs/networking/#with-a-wi-fi-dongle
[Neato XV11 Lidar]: http://www.ev3dev.org/docs/tutorials/using-xv11-lidar/

## Building Instructions

The instructions here are for compiling on EV3.

### Compilers and make

``` bash
$ sudo apt-get update
$ sudo apt-get install build-essential
```

### Dependencies

ev3wifi module depends on Minimalistic Netlink Library - [libmnl](https://www.netfilter.org/projects/libmnl/)

``` bash
$ sudo apt-get install libmnl0 libmnl-dev
```

### Getting git

``` bash
sudo apt-get install git
```

### Cloning the repository

``` bash
git clone --recursive https://github.com/bmegli/ev3dev-mapping-modules
```

Don't forget `recursive` - the repository has submodules.

### Building the modules

``` bash
cd ev3dev-mapping-modules
make all
```

Arm in patience (20 minutes). The results will be in `bin` directory.

## Next steps

ev3dev-mapping-modules is rather useless without ev3dev-mapping-ui!

Head on to [ev3dev-mapping-ui] also for information when and how to run ev3dev-mapping-modules.

## Final Remakrs

### ev3control

You don't need to call the modules by yourself. 

ev3control enables/disables/monitors the modules as requested by ev3dev-mapping-ui.

### Init Scripts

After building the project `bin` directory contains initialization scripts.
Those scripts help with things like loading the drivers, setting port modes and hardware warm-up.
The scripts are very simple and intended for *one shot* running after each boot.
If called multiple times they will not work.

### Running

ev3dev-mapping-modules is usually started by calling init script followed by `ev3control`.

Example:

``` bash
cd ev3dev-mapping-modules/bin
sudo ./ev3init.sh     #load the CruizCore driver, prepare laser ports, warm up lasers
./ev3control 8004 500 #make ev3control listen on TCP/IP port 8004 with 500 ms keepalive 
```

### Security

Note that ev3control is insecure at this stage so you should only use it in trusted networks (e.g. private) and as non-root user.


[ev3dev-mapping-ui]: https://github.com/bmegli/ev3dev-mapping-ui
