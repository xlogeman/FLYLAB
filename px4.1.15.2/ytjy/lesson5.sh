#!/bin/sh

export PX4_SIM_MODEL=none_iris
../build/px4_sitl_default/bin/px4 -d -i 1 &
../build/px4_sitl_default/bin/px4 -d -i 2 &
../build/px4_sitl_default/bin/px4 -d -i 3 &
../build/px4_sitl_default/bin/px4 -d -i 4
