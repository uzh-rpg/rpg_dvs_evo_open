#!/bin/bash

source ~/setupros.sh
rostopic pub -1 candice/copilot/off std_msgs/Empty "{}"
