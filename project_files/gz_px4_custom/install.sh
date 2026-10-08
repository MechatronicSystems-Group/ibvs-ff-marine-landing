#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

if [ -d "$HOME/PX4-Autopilot" ]; then
	echo
	echo "~/PX4-Autopilot directory exists, installing custom content..."
	echo

	cp "$SCRIPT_DIR/custom_worlds/"* "$HOME/PX4-Autopilot/Tools/simulation/gz/worlds"
	cp -r "$SCRIPT_DIR/custom_models/"* "$HOME/PX4-Autopilot/Tools/simulation/gz/models"

	# Jank but can also copy plugins directly - one at a time, unfortunately. Requires make px4_sitl to run first.
	cp "$SCRIPT_DIR/gz_plugins/barge_controller/build/libBargeController.so" "$HOME/PX4-Autopilot/build/px4_sitl_default/src/modules/simulation/gz_plugins"

	# Add gz plugins to search directory
	export GZ_SIM_SYSTEM_PLUGIN_PATH=$(find "$SCRIPT_DIR/gz_plugins" -type d -name build | paste -sd:)

else
	echo "No ~/PX4-Autopilot found. Install in home directory, or edit install.sh to target correct install directory."
fi
