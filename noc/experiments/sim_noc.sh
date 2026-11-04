#!/bin/sh

middleware=pocolibs
gz_world=/shared-workspace/gazebo/worlds/noc-wall-world.world

# noc copy of src/06a-physical-interaction/simulation.sh (only the world differs):
# noc-wall-world uses model://noc-tilthex (EE on link ee, ee_joint force_torque sensor -> /noc/ee_ft).
# Prepend our custom models path so Gazebo loads /shared-workspace/gazebo/models/noc-tilthex
# (mrsim-rotor still comes from /opt/openrobots/share/gazebo/models via the existing path)
export GZ_SIM_RESOURCE_PATH=/shared-workspace/gazebo/models:$GZ_SIM_RESOURCE_PATH

# Genom3 components for full Gazebo simulation
# Gazebo handles dynamics + contact via mrsim-gazebo plugin
# rotorcraft = motor interface, pom = state estimation, optitrack = motion capture
# uavpos/uavatt = fully-actuated controller, maneuver = trajectory, phynt = AF + WO
components="
  uavpos
  uavatt
  maneuver
  phynt
  pom
  optitrack
  rotorcraft
"

pids=

atexit() {
    trap - 0 INT CHLD
    set +e

    kill $pids
    wait
    case $middleware in
        pocolibs) h2 end;;
    esac
    exit 0
}
trap atexit 0 INT
set -e

case $middleware in
    pocolibs) h2 init;;
    ros) roscore & pids="$pids $!";;
    *) echo "invalid middleware: $middleware";;
esac

genomixd & pids="$pids $!"

for c in $components; do
    $c-$middleware & pids="$pids $!"
done

if [ ! -f $gz_world ]; then
    echo "Cannot find world file: $gz_world"
    exit 1
fi

gz sim $gz_world & pids="$pids $!"

trap atexit CHLD
wait
