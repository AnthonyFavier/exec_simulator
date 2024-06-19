#!/bin/bash
ffmpeg -r 30 -i /tmp/camera_save_tutorial/frame_%00d.jpg -c:v libx264 video.mp4