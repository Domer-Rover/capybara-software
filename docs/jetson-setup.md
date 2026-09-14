# Jetson Setup

Host: `jetsonson.dhcp.nd.edu`. Admin (only sudo): `jetsonson`. Only one person can use the rover hardware at a time.

## Add a developer (admin)

```bash
cd ~/capybara-software
sudo scripts/add_dev_user.sh <username> "<Full Name>" <git-email>   # username must be lowercase
sudo scripts/add_dev_user.sh --regen-key <username>                 # replace their GitHub key
sudo pkill -u <username>; sudo userdel -r <username>                # remove a developer
```

The script creates the user, adds hardware groups (`dialout video render plugdev i2c gpio zed jtop adm`), sets git author, clones the three repos into `~/domerrover/`, and prints a GitHub SSH key. Add that key at https://github.com/settings/keys.

Developers have no sudo. System packages: ask the admin (`sudo apt install`). Personal Python packages: `python3 -m venv .venv`.

## First login (developer, from your laptop)

```bash
ssh-keygen -t ed25519                              # skip if you already have a key
ssh-copy-id <username>@jetsonson.dhcp.nd.edu       # once; last time you type the password
ssh <username>@jetsonson.dhcp.nd.edu
ssh -T git@github.com                              # confirms GitHub key works
```

## Serial devices

`ttyUSB` numbers swap between boots, so configs use stable names from `udev/99-rover.rules`. Install once (admin), and again after adding or replacing an adapter:

```bash
sudo cp udev/99-rover.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/rover_*
```

| Name | Device | USB adapter |
|---|---|---|
| `/dev/rover_roboclaw` | 3 RoboClaws (addresses 128–130) | FTDI FT232R `BG00HO5R` |
| `/dev/rover_lidar` | LD19 LIDAR | Silicon Labs CP2102 |
| `/dev/rover_gps` | u-blox GPS | u-blox GNSS receiver |

Check what's plugged in: `ls -l /dev/serial/by-id/`
