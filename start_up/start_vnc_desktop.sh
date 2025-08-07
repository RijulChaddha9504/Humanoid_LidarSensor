#!/bin/bash

# Step 1: Kill any old Xvfb, VNC, or LXDE processes
pkill -f Xvfb
pkill -f x11vnc
pkill -f startlxde

# Step 2: Clean up lock files and old sockets
rm -f /tmp/.X1-lock
rm -rf /tmp/.X11-unix/X1
rm -f /tmp/.Xauthority

# Step 3: Create a new Xauthority file
touch /tmp/.Xauthority
export XAUTHORITY=/tmp/.Xauthority

# Step 4: Generate a magic cookie and authorize display :1
xauth add :1 . $(mcookie)

# Step 5: Start Xvfb (Virtual Framebuffer) on display :1
Xvfb :1 -screen 0 1280x1024x24 -auth /tmp/.Xauthority -ac &

# Step 6: Set DISPLAY environment variable for the session
export DISPLAY=:1

# Step 7: Start x11vnc server (VNC access)
x11vnc -display :1 -auth /tmp/.Xauthority -nopw -forever -shared &

# Step 8: Start lightweight desktop environment
startlxde &

# Step 9: Wait a few seconds to allow the environment to initialize
sleep 5

# Step 10: Optional test - launch xclock
xclock &
