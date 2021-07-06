#!/bin/bash
set -e

# setup ros environment
source "/dev_ws/install/setup.bash"
exec "$@"
