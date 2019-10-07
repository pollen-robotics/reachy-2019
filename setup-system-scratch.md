# Setup your Raspberry-Pi from Scratch

Reachy's software is built around a Raspberry-Pi. We are currently using the Raspbian Buster OS.

The first step is to download and burn an SD card (>=16Go) with the image file: [Raspbian Buster with desktop](https://www.raspberrypi.org/downloads/raspbian/). You can use [etcher](https://www.balena.io/etcher/) for that.

## Connect to your Raspberry-Pi 4

Next steps are installing extra required software on your Raspberry-Pi. So, make sure you can access it, either add ssh file to the *boot* partition or plug a keyboard, mouse and screen to the board.
You may also need to add a wifi configuration *wpa_supplicant.conf* file directly on the *boot* partition:

```bash
country=fr
update_config=1
ctrl_interface=/var/run/wpa_supplicant

network={
    ssid="mon-reseau-wifi"
    psk="password"
}
```

### Raspi-Config

Then, run ```sudo raspi-config```:

1. Change default user password
2. Change hostname

If you want to be able to remotely access the GUI (via VNC):
3. Enable VNC (Interfacing --> VNC)
4. Auto graphical login (Boot --> Desktop/CLI --> Dekstop Autologin)
5. Setup Screen Resolution (Advanced --> Resolution --> 1080p)


### Run a full update

* ```sudo apt update```
* ```sudo apt upgrade```

### Setup Python

We use the Python3 coming with the system. It should work with any other recent Python (>= 3.6) and with virtual environment but this is out of scope of this readme.

1. Install ipython 
```
pip3 install ipython

```
