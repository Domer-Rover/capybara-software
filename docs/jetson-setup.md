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

| Device | Port | by-id |
|---|---|---|
| RoboClaw | `/dev/ttyUSB0` | `usb-FTDI_FT232R_USB_UART_BG00HO5R` |
| LD19 LIDAR | `/dev/ttyUSB1` | `usb-Silicon_Labs_CP2102_...` |
| u-blox GPS | `/dev/ttyACM0` | `usb-u-blox_AG_...` |

`ttyUSB` numbers can swap after reboot. Check with `ls -l /dev/serial/by-id/`.
