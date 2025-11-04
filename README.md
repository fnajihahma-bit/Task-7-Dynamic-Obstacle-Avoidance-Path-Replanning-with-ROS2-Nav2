# Task-7-Dynamic-Obstacle-Avoidance-Path-Replanning-with-ROS2-Nav2
Dynamic Obstacles in Robotics
----
## Prepare Obstacles Model:

### Step 1. Verify model path

Run this:

```bash
ls ~/gazebo_models
```
If you don’t see a folder named box, or if it’s empty, you must create or download the model.

A proper Gazebo model folder structure looks like:

```arduino
~/gazebo_models/box/
├── model.config
└── model.sdf
```
If you don’t have it, create one manually.

### Step 2. Create a simple box model (works in any Gazebo)


```bash
mkdir -p ~/gazebo_models/box
cd ~/gazebo_models/box
```
Now create a file named model.sdf:

```bash
nano model.sdf
```
Paste this minimal model definition:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="box">
    <static>false</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```
Save and exit (Ctrl+O, Enter, Ctrl+X).

Then add the model config file:

```bash
nano model.config
```

Paste:

```xml
<?xml version="1.0"?>
<model>
  <name>box</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Student</name>
  </author>
  <description>Simple red box obstacle</description>
</model>
```
Save and exit (Ctrl+O, Enter, Ctrl+X).


### Step 3. Test that Gazebo can find your model

Make sure Gazebo knows where your models live:

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/gazebo_models
```

(Optional: add that line to ~/.bashrc)

Verify again:

```bash
ls ~/gazebo_models/box/model.sdf
```

### Step 4. Try spawning again


```bash
source ~/.bashrc


ros2 run gazebo_ros spawn_entity.py \
  -entity box \
  -file ~/gazebo_models/box/model.sdf \
  -x 1.5 -y 0.5 -z 0.25
```

✅ Output:
```markdown
[INFO] [spawn_entity]: Spawn Entity started
[INFO] [spawn_entity]: Loading entity XML from file /home/.../box/model.sdf
[INFO] [spawn_entity]: Waiting for service /spawn_entity, timeout = 30
[INFO] [spawn_entity]: Calling service /spawn_entity
[INFO] [spawn_entity]: Successfully spawned entity [box]
```
And in Gazebo, a red cube will appear at coordinates (1.5, 0.5, 0.25).


----
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









