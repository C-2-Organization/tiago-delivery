#!/bin/bash

# Build Docker image
docker build -t tiago-delivery-ros2 -f docker/Dockerfile.ros2-humble .
