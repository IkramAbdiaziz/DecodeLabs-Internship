#!/bin/bash

# Grant display socket permissions to the container
xhost +local:root

# Stop any running containers
docker compose down

# Build and start the simulation
docker compose up --build
