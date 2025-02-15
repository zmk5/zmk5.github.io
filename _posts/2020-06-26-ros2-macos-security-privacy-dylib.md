---
layout: single
title: dylib Authorization with a ROS 2 Foxy Binary Install in Newer Versions of macOS
categories: [general, guide]
tags: [ros2, macOS]
fullview: true
comments: false
classes: wide
---

### Intoduction

One of the issues I've run into using the binary installation of ROS 2 Foxy on macOS (Catalina and newer) is you have to allow all the `dylib` files one...by...one...because they are not by *"identified developers"*. Since the OSR group are not distributing signed binaries, macOS will try and be extra careful and not allow these `dylib`s to run. This is smart and secure behavior by the OS but it'is kind of annoying especially when you are a developer and you know exactly what you are running. If you installed ROS 2 Foxy using a source installation, this behavior doesn't happen since **YOU** are the developer because you compiled the binary on your own system.

To get around this, you have to allow every `dylib` used by the program binary, which can be confusing to newcomers since there is no option to override this behavior. The following guide will show you how to override this.

&nbsp;

**WARNING**: If you do not know for certain who provided the binaries you wish to use, do not give it permission to run on your system!

&nbsp;

### Step 1

For example, when trying to open `rviz2` through the command line, you will run into this a lot pop-up window below a lot. You don't want to move the binary to the trash or just close. Our fist step in getting around this is to open up the *"Security & Privacy"* option in the *"Settings"* app. Press the Lock symbol at the bottom left of that window to allow you to change permissions.

![Step 1](/assets/media/2020-06-26/step1.png)

&nbsp;

### Step 2

The option to allow the `dylib` will only show up when you close the small pop-up that had the warning (press *"Cancel"*). You will then see an option to allow the `dylib` in the *"Security & Privacy"* window as shown in the picture below. Now press *"Allow Anyways"*.

![Step 2](/assets/media/2020-06-26/step2.png)

&nbsp;

### Step 3

Run the command in your terminal for your program again (in my case `rviz2`) and the new pop up warning will give you an *"Open"* option as shown below. Press *"Open"* and continue with what you were trying to open.

Some executables require lots of `dylib`s, so you will be repeating this a lot...

![Step 3](/assets/media/2020-06-26/step3.png)

&nbsp;

Hope this helps! Just remember to be care to not use this on binaries that you do not know about!
