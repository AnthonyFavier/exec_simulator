#!/bin/bash
ffmpeg -r 30 -i /tmp/save_frames_from_simulator/frame_%00d.jpg -c:v libx264 video.mp4