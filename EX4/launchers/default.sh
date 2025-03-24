#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# Launch AprilTag detection node
rosrun ex4 apriltag_detection.py

# # Launch Navigation Control node
# dt-exec rosrun packages navigate_template.py &

# # Launch Crosswalk Detection Node
# dt-exec rosrun packages crosswalk.py &

# # Launch Safe Navigation Node
# dt-exec rosrun packages safe_navigation.py &

# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
