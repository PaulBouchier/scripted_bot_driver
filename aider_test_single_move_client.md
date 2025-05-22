# Test program for single_move_client

## High level goal

Send alternating spins and straight line moves

## Context

This python node will instantiate a SingleMoveClient and send it moves

## Low level instructions

Perform test steps in order.

1. Create the test file
/read-only scripted_bot_driver/single_move_client
/add test/single_move_client_test.py
/add setup.py

2. Initialize a node

- initialize ROS2 and make a node
- instantiate SingleMoveClient, passing it the mode

3. Call SingleMoveClient.execute_move() with the following sequence
of arguments. Repeat 10 times.

```aider
("rotate_odom" "roto 1.57")
("rotate_odom" "roto -1.57")
("drive_straight_odom" "movo 0.1")
("drive_straight_odom" "movo -0.1")
("rotate_odom" "roto 1.57")
```

4. Update the build files