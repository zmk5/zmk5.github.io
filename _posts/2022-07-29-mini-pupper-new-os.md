---
layout: single
title: Run a Custom Ubuntu OS for Mini Pupper
categories: [general, guide]
tags: [ros, ubuntu, mini-pupper]
fullview: true
comments: false
classes: wide
---

## Intoduction

## Install the OS and Raspberry Pi Dependencies

- Get Raspberry Pi image of Ubuntu Mate (20.04)
    - [`20.04/arm64`](https://releases.ubuntu-mate.org/20.04/arm64/) of Ubuntu Mate
        - Make sure to download `ubuntu-mate-20.04.1-desktop-arm64+raspi.img.xz`

- Use `balenaEtcher` to write the image to and SD Card.
    - If you are trying to write this image using Ubuntu 22.04 on your HOST computer, then you will need to run `balenaEtcher` with a CLI and the following flags since they are using an old version of Chromium according to [this](https://github.com/balena-io/etcher/issues/3761#issuecomment-1141339382) GitHub issue:
        - `./balenaEtcher-1.7.9-x64.AppImage --no-sandbox --disable-gpu-sandbox --disable-seccomp-filter-sandbox`

- Plug into Raspberry Pi and follow installation instructions
    - **IMPORTANT**: Make sure to make your username `ubuntu`, if you use any other username none of the `QuadrupedRobot` libraries will work! You can give any name for the computer name.

- Update the OS inside a terminal window

```bash
~$ sudo apt update && sudo apt upgrade -y
```

- Install OpenSSH Server

```bash
~$ sudo apt install openssh-server
```

- Open firewall (*if needed*)

```bash
~$ sudo ufw allow 22
~$ sudo ufw reload
```

- Install `git`

```bash
~$ sudo apt install git
```

- **Optional** Installation of `sshguard`

```bash
~$ sudo apt install sshguard
```

## Install `QuadrupedRobot` libraries

- Create an empty `.hw_version` file.

```bash
~$ cd ~/
~$ touch .hw_version
```

- Create a `Robotics` directory in your `home` directory and `cd` into it.
    - This *needs* to be done or many of the installation scripts for the Mini Pupper will fail.

```bash
~$ mkdir Robotics
~$ cd Robotics
```

- Clone the `QuadrupedRobot` repository

```bash
~$ git clone https://github.com/mangdangroboticsclub/QuadrupedRobot.git
```

- `cd` into the `Legacy` directory and install legacy components.

```bash
~$ cd QuadrupedRobot/Legacy
~$ bash pre_install.sh
```

- Install `MangDang` components

```bash
~$ cd /home/ubuntu/Robotics/QuadrupedRobot
~$ bash install.sh
```

Finally, restart your system and you'll notice that your Mini Pupper's eyes will light up greating you with a successful installation!
