# waypoint_nav

Drive the robot by sending a list of waypoints.

## mobile_waypoint_control.py
Simple waypoint control for mobile robot (only with vel.linear.x and vel.angular.z). When a waypoint is received the robot will at first turn to the waypoint and then forward to it.

When turning:
```python3
vel.linear.x = 0
vel.angular.z = ANG_K * angle
```

When forwarding:
```python3
vel.linear.x = FWD_K * distance
vel.angular.z = ANG_K * angle
```

Where FWD_K and ANG_K are the control parameters.

## arcore_localization.py
Robot localization based on ARCore. A Transformation needs to be applied if camera center is not align with robot center.

## waypoints_handler.PY
Hold a dictionry for a list of waypoints. Next waypoint is sent when the previous waypoint is reached.
