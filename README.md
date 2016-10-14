# ev3dev-mapping-modules
A bunch of small programs feeding data to [ev3dev-mapping-ui](https://github.com/bmegli/ev3dev-mapping-ui)

For high-level project overview visit [ev3dev-mapping web page](http://www.ev3dev.org/projects/2016/08/07/Mapping/)

For project meta-repository visit [ev3dev-mapping](https://github.com/bmegli/ev3dev-mapping)

To do - add information on hardware requirements

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

Don't forget `recursive` - the repository has two submodules.

### Building the modules

``` bash
cd ev3dev-mapping-modules
make all
```

Arm in patience (20 minutes). The results will be in `bin` directory.

## Next steps

### Getting Started - LIDAR

Follow [Using the XV11 LIDAR](http://www.ev3dev.org/docs/tutorials/using-xv11-lidar/)

### Mapping 

To do! Totally preliminary!

Assuming the same hardware like me you would cd to bin directory, call sudo ev3init, and call ev3control.

ev3control would take care of the rest enabling/disabling modules on request from ev3dev-mapping-ui.

Note that ev3control is insecure at this stage so you should only use it in trusted networks (e.g. private)
