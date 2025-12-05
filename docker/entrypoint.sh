#!/bin/bash
source "$HOME/.bashrc"
{
    echo "Container is Running"
    source /opt/ros/noetic/setup.bash
    exec /bin/bash
} || {
    echo "Container failed..."
    exit 1
}