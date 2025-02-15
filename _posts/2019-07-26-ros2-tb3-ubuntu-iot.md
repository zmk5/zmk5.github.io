---
layout: single
title: Setting-up the Turtlebot3 with ROS 2 on Ubuntu Server IoT 18.04
categories: [general, guide]
tags: [ros2, ubuntu]
fullview: true
comments: false
classes: wide
---


One of the cool things about ROS 2 is that ROS Master is finally gone. The new DDS approach allows for interesting ways of controlling multiple autonomous agents without having to rely on a centralized ROS Master running the show. For this guide, I'll show you how to set up ROS 2 on a Turtlebot3 burger or waffle using Ubuntu Server IoT 18.04. Why use this instead of Ubuntu Mate 18.04 like the Turtlebot guide suggests? Well, I've run into a lot of problems during the installation and its just kind of bloated. I don't Firefox, VLC, Thunderbird, Libreoffice etc. All I need is just a bash shell because I'm going to be writing most of the code for it on a different computer anyways. So let's start!

NOTE: If you are not comfortable with just using a terminal, I suggest installing Ubuntu Mate 18.04 instead. Make sure to download and install the `armhf` version!

&nbsp;

### Step 1: Download and Install Ubuntu Server IoT 18.04

You can get it from this link [here](https://ubuntu.com/download/iot/raspberry-pi-2-3). Make sure to download the Raspberry Pi 3 version!

As an aside, the Ubuntu Mate image for the Raspberry Pi 3 version that is 32-bit, but that won’t work with this distro. Only the Raspberry Pi 2 version of Ubuntu Server IoT is 32-bit. I tried that version with the Raspberry Pi 3 and it won’t boot past the color screen.

&nbsp;

### Step 2: Start the OS on the Turtlebot3

Insert the microSD card into the slot on the Raspberry Pi 3 and begin the boot up process. It will go through the whole configuration and you will reach the login screen which is just the bash shell. The login is `ubuntu` and the password is `ubuntu`. Once entered you will be greeted with an option to change the password. Do it since it is much more secure this way.

&nbsp;

### Step 3: Change the Device's Hostname

This may not be necessary if you plan on just using one Turtlebot3, but if you want to use multiple ones, I would suggest changing the device's `hostname` using:

```bash
~$ hostnamectl set-hostname <the-new-hostname-you-want>
```

NOTE: When I use `< >`, in a command, it typically means you **do not** use them the command. So the above command would look like this:

```bash
~$ hostnamectl set-hostname my-new-tb3-hostname
```

&nbsp;

### Step 4: Create a New Username

Just like the step above, this is optional if you only plan on using one turtlebot3. If not, I would suggest either changing the current one, or generating a new one and giving permissions. I decided with just creating a new one. You can do that this way:

```bash
~$ sudo adduser <username>
```

Once the command is executed, it will ask for information about the user such as First Name, Last Name, etc. You can just keep them blank and press `ENTER` to continue to the next options. Next, we will add group permissions to the new user once back to the command prompt:

```bash
~$ sudo usermod -aG sudo <the-new-username-you-made>
```

The above command gives the new username `sudoer` permissions by adding them to the `sudo` group. To have the same permissions as the `ubuntu` user you logged on with, you will need to repeat this command and add the user to the following permission groups:

```bash
~$ sudo usermod -aG adm <the-new-username-you-made>
~$ sudo usermod -aG dialout <the-new-username-you-made>
~$ sudo usermod -aG cdrom <the-new-username-you-made>
~$ sudo usermod -aG floppy <the-new-username-you-made>
~$ sudo usermod -aG audio <the-new-username-you-made>
~$ sudo usermod -aG dip <the-new-username-you-made>
~$ sudo usermod -aG video <the-new-username-you-made>
~$ sudo usermod -aG plugdev <the-new-username-you-made>
~$ sudo usermod -aG lxd <the-new-username-you-made>
~$ sudo usermod -aG netdev <the-new-username-you-made>
```

Not all of these groups are useful or needed (I have yet to see a turtlebot with a `floppy` drive lol), but I added them just to stay on the safe side.

Once this is done, reboot using the `reboot` command and login using the new username.

&nbsp;

### Step 5: Configure Static IP

This is a step for people that want a static IP for their turtlebots. In our lab, this is super useful because we can only connect to the robots using a wifi router separated from the universities internet. To install packages and such on the Raspberry Pi requires the use of an ethernet cable. Therefore, the following configuration is a wifi connection with no internet and static IP and an ethernet connection with a dynamic IP that lets us access the internet.

The newest versions of Ubuntu have a really neat systemd tool called `netplan`. Instead of configuring those old `/etc/network/interfaces` and `/etc/dhcpcd.conf` files, we just have to modify one `yaml` file located in the `/etc/netplan/` folder. We will begin by changing directory into the `/etc/netplan/` folder:

```bash
~$ cd /etc/netplan/
```

In this folder, you will find a file which we can ignore, and  we will create a new file called `01-netcfg.yaml`:

```bash
~$ sudo touch 01-netcfg.yaml
```

Now we will modify it using `nano`:

```bash
~$ sudo nano 01-netcfg.yaml
```

```yaml
# My Turtlebot3 Network Configuration
network:
    version: 2
    renderer: networkd
    ethernets:
      eth0:
        dhcp4: yes
        dhcp6: yes
        optional: true
    wifis:
      wlan0:
        dhcp4: no
        dhcp6: yes
        addresses: [xxx.xxx.xxx.xxx/24]
        access-points:
          "my-wifi-connection-name":
            password: "my-connection-password"
```

NOTE: For the `access-points` section, the connection name and password should be in quotes like above.

Finally, apply the new network configuration using this command:

```bash
~$ sudo netplan apply
```

&nbsp;

### Step 6: Tweak `systemd`

For some reason, the OS will attempt to configure a network IP at startup. This can delay boot-up by up to 5 minutes. So to prevent this, we are going to mask this `systemd` process using the following command:

```bash
~$ systemctl mask systemd-networkd-wait-online.service
```

Now boot-up should be super quick!

&nbsp;

### Step 7: Install Turtlebot3 ROS 2 Packages

Next, we are going to install the Turtlebot3 ROS 2 packages. Instructions are found [here](http://emanual.robotis.com/docs/en/popup/turtlebot3_ros2_sbc_setting/). These are not downloadable through the ROS 2 repositories yet, so you will need to compile many of these. Just a little heads up, the Micro-XRCE-DDS-Agent installation takes the longest because it will compile third party packages too. Everything afterwards is pretty quick.

NOTE: The **Build LIDAR Client** installation is wrong. The link for the `turtlebot3_lidar.tar.bz2` driver is broken and according to [this](https://github.com/ROBOTIS-GIT/turtlebot3/issues/446#issuecomment-508293951) issue, they are attempting to fix the problem. In the meantime, you can use the Crystal Clemmys driver. The command to download that is this:

```bash
~$ wget https://github.com/ROBOTIS-GIT/turtlebot3/raw/24d14d772520508e409b80c43859b7020d76bb82/turtlebot3_lidar/turtlebot3_lidar.tar.bz2
```

&nbsp;

### Step 8: Install Update to OpenCR Board

Next, you will have to update the OpenCR board to use ROS 2. The guide to do that is provided [here](http://emanual.robotis.com/docs/en/platform/turtlebot3/ros2/#opencr-setup) The issue with this is that Ubuntu Server IoT 18.04 is 64-bit arm OS. The drivers provided by ROBOTIS can only be applied by a 32-bit arm OS. I raised an issue [here](https://github.com/ROBOTIS-GIT/turtlebot3/issues/455) on there GitHub page, so they might provide one in the future.

For now, the best thing you can do is get a spare Ubuntu Mate `armhf` microSD card and use that to install the new updated OpenCR Drivers. They also provide `x86` drivers, so you can connect your laptop to the OpenCR board and install the drivers with that too.

&nbsp;

### Step 9: Install ROS 2

Follow the binary installation guide found [here](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Install-Debians/).

&nbsp;

### Step 10: Edit .bashrc File

Since I use the turtlebots for a multi-agent configuration, a nice way of having multiple robots interact in the network easier is to have all the devices use the same `ROS_DOMAIN_ID`. You can add a value to it on startup by adding it to your `.bashrc` file like this:

```bash
# ROS 2 source information
source /opt/ros/dashing/setup.bash
export ROS_DOMAIN_ID=42
```

The number can be anything you want, just make sure all the devices you wish to use together have the same one.

Thats it! If you have any feedback or suggestions please let me know!
