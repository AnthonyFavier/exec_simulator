#!/bin/bash
ffmpeg -r 30 -pattern_type glob -i '/tmp/camera_save_tutorial/default_bird_camera_link_my_camera*.jpg' -c:v libx264 my_camera.mp4