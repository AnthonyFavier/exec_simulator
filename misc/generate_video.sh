#!/bin/bash
# ffmpeg -r 5 -pattern_type glob -i '/tmp/camera_save_tutorial/frame_*.jpg' -c:v libx264 video.mp4
ffmpeg -r 20 -i /tmp/camera_save_tutorial/frame_%00d.jpg -c:v libx264 video.mp4