#!/bin/bash
# Universal Docker Setup Script
# Creates project directory structure and handles X11 forwarding for GUI applications in Docker
# Clear the terminal

clear


GREEN='\033[0;32m'

RED='\033[0;31m'

YELLOW='\033[0;33m'

BLUE='\033[0;34m'

CYAN='\033[0;36m'

BOLD='\033[1m'

PURPLE='\033[0;35m'

NC='\033[0m' # No Color

echo -e "${BLUE}"

echo "                          ██████╗ ██████╗ ██████╗ ██████╗ ███████╗ ██████╗   "

echo "                         ██╔═══██╗██╔══██╗██╔══██╗██╔══██╗██╔════╝██╔════╝   "

echo "                         ██║   ██║██████╔╝██████╔╝██████╔╝█████╗  ██║        "

echo "                         ██║   ██║██╔══██╗██╔══██╗██╔══██╗██╔══╝  ██║        "

echo "                         ╚██████╔╝██║  ██║██████╔╝██████╔╝███████╗╚██████╗   "

echo "                          ╚═════╝ ╚═╝  ╚═╝╚═════╝ ╚═════╝ ╚══════╝ ╚═════╝   "

echo -e "${WHITE}                                  Let all terminals see the world${NC}"

echo

sleep 2

# Create project directory structure
echo "Creating project directory structure..."

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check X environment variables
echo "Checking X environment variables..."
echo "XAUTHORITY=$XAUTHORITY"
echo "XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR"

# Only configure X11 if XAUTHORITY or XDG_RUNTIME_DIR is not set
if [ -z "$XAUTHORITY" ] || [ -z "$XDG_RUNTIME_DIR" ]; then
    echo "Setting up X11 forwarding..."
    
    # Try to set XAUTHORITY if not defined
    if [ -z "$XAUTHORITY" ]; then
        XAUTH_PATH=$(xauth info 2>/dev/null | grep "Authority file" | awk '{print $3}')
        if [ -n "$XAUTH_PATH" ]; then
            export XAUTHORITY=$XAUTH_PATH
            echo "XAUTHORITY set to $XAUTHORITY"
        fi
    fi
    
    # Try to set XDG_RUNTIME_DIR if not defined
    if [ -z "$XDG_RUNTIME_DIR" ]; then
        export XDG_RUNTIME_DIR=/run/user/$(id -u)
        echo "XDG_RUNTIME_DIR set to $XDG_RUNTIME_DIR"
    fi
    
    # Configure X server access
    if command_exists xhost; then
        echo "Configuring xhost access..."
        xhost +local:docker
        
        # Create .docker.xauth file
        echo "Creating X authentication file..."
        touch /tmp/.docker.xauth
        xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -
        chmod 777 /tmp/.docker.xauth
    else
        echo "Warning: xhost command not found. X11 forwarding may not work properly."
    fi
else
    echo "X environment variables already set, skipping X11 setup."
fi

# Start Docker containers
echo "Starting Docker containers..."
if command_exists docker-compose; then
    echo "Using docker-compose command..."
    docker-compose up -d
elif command_exists docker && command_exists compose; then
    echo "Using docker compose command..."
    docker compose up -d
else
    echo "Error: Neither docker-compose nor docker compose commands are available."
    exit 1
fi

# Connect to container
echo "Connecting to container..."
docker exec -it orbbec-camera-demo bash





