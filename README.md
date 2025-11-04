# Task-7-Dynamic-Obstacle-Avoidance-Path-Replanning-with-ROS2-Nav2
Dynamic Obstacles in Robotics

## Gazebo Simulation
Start here after Task 6 Step 7 (Nav2 + AMCL + Waypoint follower running and robot moving).

### 0) Quick verification (do this first)

Confirm the system is running and topics exist:

```bash
# check nav2 lifecycle nodes are active
ros2 lifecycle list

# check important topics
ros2 topic list | grep -E "scan|cmd_vel|amcl_pose|tf|/global_costmap|/local_costmap|/follow_waypoints"
# inspect laser and odom
ros2 topic echo -n1 /scan
ros2 topic echo -n1 /amcl_pose
```
If anything is missing (e.g., /scan), fix the sensor config before proceeding.


### 1) Spawn a static obstacle while mission runs (force a replan)

Use the Gazebo spawn helper (you already have this in your notes). In a new terminal (remember to source):

```bash
source ~/.bashrc
# spawn a box at x=1.5 y=0.5 z=0.5 — tweak coords to put directly on the robot's path
ros2 run gazebo_ros spawn_entity.py -entity box -file ~/gazebo_models/box/model.sdf -x 1.5 -y 0.5 -z 0.5
```

What to watch:
- In RViz: global path (blue) and local path (green). After the box appears, the green path should bend around the obstacle; the robot should slow/stop then re-route.
- In Gazebo: visually confirm the box sits on the robot’s planned path.
- Topics: watch /amcl_pose and /cmd_vel to see immediate robot response:

```bash
ros2 topic echo /amcl_pose
ros2 topic echo /cmd_vel
```
Capture screenshots: (1) before spawn, (2) right after spawn, (3) after robot has rerouted.


### 2) Spawn multiple obstacles / test different positions

Repeat spawning with different coordinates to stress-test Nav2. Example:

```bash
ros2 run gazebo_ros spawn_entity.py -entity box2 -file ~/gazebo_models/box/model.sdf -x 0.8 -y 0.2 -z 0.5
ros2 run gazebo_ros spawn_entity.py -entity box3 -file ~/gazebo_models/box/model.sdf -x 1.0 -y -0.2 -z 0.5
```

Tip: spawn one directly in front of the robot, one near a narrow passage, and one temporarily blocking a doorway to check behavior in confined spaces.

### 3) Moving obstacle (two simple, reliable approaches)
A — Move it manually in Gazebo GUI (fast, no coding)
1. In Gazebo, select the Translate/Rotate tool (an icon of arrows).
2. Click the spawned model and drag it slowly across the robot’s path while the robot is moving.
- This is perfect for live demos — the robot should replan continuously as the obstacle moves.

B — Use a model trajectory plugin (for reproducible tests)
- Create a small SDF model plugin or write a short Gazebo model controller (in SDF) that moves the model on a preset path/velocity. (If you want, I can provide a minimal SDF plugin example.)
- Alternatively, use Gazebo's model trajectory features or a simple ROS interface if you already have one.

(Why avoid uncertain CLI topics? Gazebo installations and service/topic names vary across versions — the GUI approach always works.)

### 4) Observe and log replanning behavior

Useful things to monitor/record:
- RViz: show these displays:
  - Global path (Nav2 planner)
  - Local path (DWB / TEB)
  - Costmaps (local & global) — add Map displays pointing at /local_costmap/costmap and /global_costmap/costmap (topic name may vary by config)
  - Laser scans (/scan)
  - Pose estimate (/amcl_pose)

Terminal: tail Nav2 logs

```bash
# if launched with ros2 launch, view stdout from that terminal or run:
ros2 run nav2_util lifecycle_service_client # (if you use a log helper)
```

Record the run for analysis:

```bash
# record relevant topics (scan, amcl_pose, cmd_vel, /local_costmap/costmap, /global_costmap/costmap)
ros2 bag record /scan /amcl_pose /cmd_vel /tf /global_costmap/costmap /local_costmap/costmap -o task7_run1
```

After the run, you can replay or inspect the bag to extract timings (time from obstacle creation to replan, path length, etc.)

5) Change costmap & planner parameters (experiment systematically)

Edit your nav2_params.yaml (back up first). Example safe snippets and what they do:

Obstacle layer

```yaml
obstacle_layer:
  enabled: True
  observation_sources: scan
  scan:
    topic: /scan
    expected_update_rate: 10.0
    data_type: LaserScan
    marking: true
    clearing: true
    max_obstacle_height: 2.0
```

Inflation layer

```yaml
inflation_layer:
  inflation_radius: 0.4   # try 0.2, 0.4, 0.6
  cost_scaling_factor: 10.0
```

Footprint (very important)

```yaml
footprint: "[[0.2,0.2],[0.2,-0.2],[-0.2,-0.2],[-0.2,0.2]]"
```



What to test (run each config with the same obstacle scenario):

   1. inflation_radius: smaller -> tighter to obstacles, larger -> more clearance and longer avoidance paths.
   2. cost_scaling_factor: higher -> more strongly penalizes paths near obstacles.
   3. max_obstacle_height: set to robot height if ceiling clutter is present.
   4. Toggle obstacle_layer marking/clearing to control how obstacles are removed from maps.

How to apply changes

   - Stop Nav2 and relaunch the same launch file with the edited nav2_params.yaml (Nav2 doesn’t hot-reload params in many setups).
   - Re-run the exact spawn & movement steps to compare behavior.









