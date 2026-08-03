#!/bin/bash

# Allow local connection to X11 display for docker containers
xhost +local:docker

# Build and run the simulation container
docker compose up --build
