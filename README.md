
### Dependencies

*   ROS Noetic
*   Python 3
*   `rospy`, `numpy`, `scipy`
*   `ar_track_alvar`: The `fake_ar_publisher_node` publishes messages of this type to simulate the real `ar_track_alvar` package.

### Phase 1: Learning the Task

This phase runs a simulation where a human demonstrator performs the assembly task. The DNF system observes these actions and learns the sequence and timing.

**To Run Learning:**

`roslaunch fake_tiago_pkg dnf_learning_pipeline.launch`

What it does: Observes a pre-scripted demonstration of an assembly task.
Output: Saves the learned memory files (`u_sm_TIMESTAMP.npy` and` u_d_TIMESTAMP.npy`) into the `data_basic/` directory. 
### Phase 2: Recalling the Task
This phase uses the learned memory to predict and execute the assembly task. It can be run in two different modes using a launch argument.

#### Mode 1: Simple Simulation (Default)
This mode is for quickly testing the DNF's prediction logic without simulating the robot's  movements.

How it works: Uses `simulated_task_executive.py`, which simply waits for a fixed time (time.sleep()) to simulate a robot action.

To Run:
```bash
# This is the default mode
roslaunch fake_tiago_pkg recall.launch
```

#### Mode 2: Realistic Tiago Simulation
This mode is for testing the full robot integration logic before deploying on the real hardware.

How it works: It uses two nodes to simulate the real system:
* `tiago_task_executive.py` (The "Bridge"): Translates DNF predictions into robot commands (e.g., "move arm," "close gripper").
* `fake_robot_controller.py` (The "Stunt Double"): Pretends to be the real robot. It receives commands, simulates the time it takes to move, and sends back pose feedback.

To Run:
```bash
roslaunch fake_tiago_pkg recall.launch mode:=tiago
```

### Running on the Real Tiago Robot
Integrating the system with the real Tiago robot should be straightforward.

#### Step 1: Configure Object Positions (Required for Hardware Integration)
For the system to work on the hardware, the real-world coordinates of the assembly objects need to be configured. This is the only code modification required.

* File to Edit: `fake_tiago_pkg/scripts/tiago_task_executive.py`
* Action: The `OBJECT_POSES` dictionary needs to be updated with the correct `(x, y, z)` values for each object, relative to Tiago's `base_link` frame.

```python
# In tiago_task_executive.py, inside the __init__ method:
self.OBJECT_POSES = {
    # These placeholder values need to be replaced with measured coordinates
    'base':    Pose(position=Point(x=0.5, y= 0.3, z=0.1), ...),
    'load':    Pose(position=Point(x=0.5, y= 0.1, z=0.1), ...),
    # ... and so on for other objects
}
```

#### Step 2: Configure the Vision System
The simulation uses `fake_ar_publisher` to mimic the `ar_track_alvar` package by publishing fake` ar_track_alvar_msgs/AlvarMarkers` to the `/ar_pose_marker` topic.

For the real robot, this simulation node needs to be replaced by the actual perception stack. 

#### Step 3: Launch the Nodes
In the main launch file that starts the Tiago drivers and OpenSOT, the DNF nodes need to be included.

```xml
<!-- Add these nodes to your main Tiago launch file -->

<!-- The DNF Brain -->
<node name="dnf_recall_node" pkg="fake_tiago_pkg" type="dnf_model_recall_simple_node.py" output="screen" />

<!-- The Bridge to your robot controller -->
<node name="tiago_task_executive" pkg="fake_tiago_pkg" type="tiago_task_executive.py" output="screen" />

<!-- Plus the vision pipeline nodes (vision_to_dnf, etc.) -->
```

**IMPORTANT**: The `fake_robot_controller.py` node should not be launched when running on the real robot, as the OpenSOT stack provides the real control and feedback.


### OVERVIEW OF NODES

| Phase         | Number of Nodes | Purpose                                   |
| ------------- | --------------- | ----------------------------------------- |
| Learning Only | 2               | Demonstrate and learn the task            |
| Recall Only   | 5               | Execute and adapt based on learned memory |
| Both Phases   | 3               | Shared infrastructure (vision pipeline)   |

### LEARNING-ONLY NODES

#### human_action_simulator_node

*   **Role:** Simulates the expert human demonstrator.
*   **Why Learning Only:** In recall, the robot acts autonomously; no human demonstration is needed.
*   **Publishes:** `/simulation/human_pickup`
*   **Key Feature:** Scheduled pickup times to drive the demonstration.

#### dnf_model_learning_simple_node (the "brain" of the architecture)

*   **Role:** The "student" that observes the demonstration and learns the task sequence and timing.
*   **Subscribes:** `/dnf_inputs`
*   **Saves:** `u_sm.npy` (sequence memory) & `u_d.npy` (task duration).
*   **Fields:** 2 (u_sm, u_d).

### RECALL-ONLY NODES

#### fake_voice_commander

*   **Role:** Simulates human verbal cues to test the system's adaptability.
*   **Why Recall Only:** Used to communicate that the human is waiting for the robot's action.
*   **Publishes:** `/voice_command`
*   **Input Type:** Generates permanent Gaussians in the DNF input.

#### dnf_model_recall_simple_node (the "brain" of the architecture)

*   **Role:** Predicts the next action using learned memory. This is the "performer" that executes the learned task.
*   **Subscribes:** `/dnf_inputs`
*   **Publishes:** `/threshold_crossings` (predictions).
*   **Loads:** `u_sm.npy`, `u_d.npy`, `h_u_amem.npy`.
*   **Saves:** `h_u_amem.npy` (trial-specific adaptive memory).
*   **Fields:** 6 (u_act, u_sim, u_wm, u_f1, u_f2, u_error).

#### simulated_task_executive_node

*   **Role:** A **simple** action executor for fast simulations.
*   **Key Feature:** Simulates robot actions with a simple `time.sleep()` delay.
*   **Mode:** Used when `mode:=simple` (default).

#### tiago_task_executive_node

*   **Role:** A **realistic "bridge"** to a real or simulated robot controller.
*   **Key Feature:** Translates a DNF prediction into a multi-step sequence (e.g., move-down, grasp, lift-up).
*   **Mode:** Used when `mode:=tiago`.
*   **Publishes:** `/dxl_input/pos_right` (arm commands), `/dxl_input/gripper_right` (gripper commands).
*   **Subscribes:** `/cartesian/right_arm/pose` for closed-loop feedback.

#### fake_robot_controller_node

*   **Role:** A **"stunt double"** for the real Tiago robot.
*   **Why Recall Only:** Simulates the robot's physical body and low-level controller.
*   **Key Feature:** Listens for commands from `tiago_task_executive`, waits for a simulated time, and publishes fake pose feedback.
*   **Mode:** Used when `mode:=tiago`.
*   **Important:** This node should **NOT** be used with the real robot, as the robot's own drivers serve this purpose.

### NODES USED IN BOTH PHASES

#### fake_ar_publisher_node

*   **Role:** Simulates the world state and AR marker detections.
*   **Publishes:** AR marker data (reacts to robot pickups).
*   **Subscribes:** Robot action commands (to update world state).

#### robot_vision_bridge_node

*   **Role:** Translates AR marker data to a simple JSON format.
*   **Publishes:** `/object_detections` (String, JSON).
*   **Subscribes:** AR marker topic from `fake_ar_publisher`.

#### vision_to_dnf_node

*   **Role:** Converts object detections into DNF-compatible Gaussian inputs.
*   **In Learning:**
    *   Only vision input is active (temporary Gaussians).
    *   Robot feedback and voice inputs are zero/unused.
*   **In Recall:**
    *   Vision input (temporary).
    *   Robot feedback (permanent, accumulates).
    *   Human voice (permanent, accumulates).
*   **Publishes:** `/dnf_inputs` (3-part array: [vision | robot | human]).