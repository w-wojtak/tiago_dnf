
### Dependencies

*   ROS Noetic
*   Python 3
*   `rospy`, `numpy`, `scipy`
*   `matplotlib` (for plotting and visualization; requires a GUI backend such as `TkAgg` or `Qt5Agg`)
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
roslaunch fake_tiago_pkg dnf_recall_pipeline.launch
```

#### Mode 2: Realistic Tiago Simulation
This mode is for testing the full robot integration logic before deploying on the real hardware.

How it works: It uses two nodes to simulate the real system:
* `tiago_task_executive.py` (The "Bridge"): Translates DNF predictions into robot commands (e.g., "move arm," "close gripper").
* `fake_robot_controller.py` (The "Stunt Double"): Pretends to be the real robot. It receives commands, simulates the time it takes to move, and sends back pose feedback.

To Run:
```bash
roslaunch fake_tiago_pkg dnf_recall_pipeline.launch mode:=tiago
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

| Phase          | Purpose                                   |
| -------------  | ----------------------------------------- |
| Learning Only  | Demonstrate and learn the task            |
| Recall Only    | Execute and adapt based on learned memory |
| Both Phases    | Shared infrastructure (vision and voice pipeline)   |

### LEARNING-ONLY NODES

#### human_action_simulator_node

*   **Role:** Simulates the expert human demonstrator.
*   **Why Learning Only:** In recall, the robot acts autonomously; no human demonstration is needed.
*   **Publishes:** `/simulation/human_pickup` (`String`, object name). 
*   **Key Feature:** Scheduled pickup times to drive the demonstration.

#### dnf_model_learning_simple_node (the "brain" of the architecture)

*   **Role:** The "student" that observes the demonstration and learns the task sequence and timing.
*   **Subscribes:** `/dnf_inputs`
*   **Saves:** `u_sm.npy` (sequence memory) & `u_d.npy` (task duration).
*   **Fields:** 2 (u_sm, u_d).

### RECALL-ONLY NODES


#### dnf_model_recall_simple_node (the "brain" of the architecture), and
#### dnf_model_recall_extended_node

*   **Role:** Predicts the next action using learned memory. This is the "performer" that executes the learned task.
*   **Subscribes:** `/dnf_inputs`
*   **Publishes:** `/threshold_crossings` (predictions).
*   **Loads:** `u_sm.npy` (or u_sm1 and u_sm2 in extended version), `u_d.npy`, `h_u_amem.npy`.
*   **Saves:** `h_u_amem.npy` (trial-specific adaptive memory).
*   **Fields:** 4 (u_act, u_wm, u_f1, u_f2) or 6 (+ u_sim and u_error).

#### fake_voice_commander

*   **Role:** Simulates human voice commands by publishing scheduled object requests.  
*   **Publishes:** `/voice_command` (`String`) – requested object name (e.g., `"base"`, `"motor"`).  
    * *( remapped to `/parsed_voice_command` in launch files.)*  
*   **Subscribes:** —  

**Description:**  
The **Fake Voice Commander Node** mimics a human issuing verbal requests for objects at specific times.  
Each scheduled command publishes the object name as a string, optionally remapped to match the downstream input of the DNF aggregator or task executive.  
Used for automated testing without real speech input.

#### voice_command_parser_node

*   **Role:** Parses raw voice messages into standardized object names.  
*   **Subscribes:** `/voice_message` (`String`) – raw voice command (e.g., `"give_motor"`).  
*   **Publishes:** `/parsed_voice_command` (`String`) – parsed object name (e.g., `"motor"`).  

**Description:**  
This node listens for raw spoken or textual commands referring to objects (e.g., `"give_motor"`, `"give_base"`).  
It converts them into simplified object identifiers (`"motor"`, `"base"`, etc.) that downstream nodes (e.g., robot controllers) can easily interpret.

**Mapping:**  
| Command         | Parsed Object |
|-----------------|----------------|
| `give_motor`    | `motor`        |
| `give_load`     | `load`         |
| `give_bearing`  | `bearing`      |
| `give_base`     | `base`         |

**Behavior:**  
If an unrecognized command contains one of the known object names (e.g., `"please give the motor"`), it will still correctly extract `"motor"`.  
Otherwise, it logs a warning and ignores the message.

#### simulated_task_executive_node

*   **Role:** A simple action executor based on predictions from DNFs.  
*   **Key Feature:** Simulates robot actions with a simple `time.sleep()` delay.
*   **Mode:** Used when `mode:=simple` (default).
*   **Subscribes:** `threshold_crossings` (`Float32MultiArray`) – position(s) where the DNF predicts an action onset.  
*   **Publishes:**  
    * `/simulation/robot_pickup` (`String`) – informs the vision system that an object has been picked up (for “vis” input).  
    * `/simulation/robot_feedback` (`String`) – provides robot feedback to the DNF (for “rob” input).  

**Description:**  
The **Task Executive Node** listens for DNF-predicted action triggers and coordinates simulated robot behavior accordingly.  
When a DNF prediction is received, it identifies which object the prediction refers to based on a predefined spatial mapping (`DNF_POS_TO_OBJECT`), then simulates the robot’s response with a time delay.


#### tiago_task_executive_node

*   **Role:** A **realistic "bridge"** to a real or simulated robot controller.
*   **Key Feature:** Translates a DNF prediction into a multi-step sequence (e.g., move-down, grasp, lift-up).
*   **Mode:** Used when `mode:=tiago`.
*   **Subscribes:**  
    * `threshold_crossings` (`Float32MultiArray`) – DNF action predictions.  
    * `/cartesian/right_arm/pose` (`PoseStamped`) – real-time robot arm pose feedback.  
    * `/manual_robot_feedback` (`String`) – allows manual triggering of feedback messages.  
*   **Publishes:**  
    * `/dxl_input/pos_right` (`PoseStamped`) – target arm poses for movement.  
    * `/dxl_input/gripper_right` (`PointStamped`) – gripper open/close commands.  
    * `/simulation/robot_pickup` (`String`) – notifies the vision system that an object has been picked up.  
    * `/simulation/robot_feedback` (`String`) – provides robot feedback to the DNF.  

**Description:**  
The **TIAGo Task Executive Node** coordinates arm and gripper control for object pickup actions triggered by DNF predictions.  
When a threshold crossing occurs, it maps the DNF position to an object, retrieves the corresponding 3D pose, and executes a predefined pickup sequence:  
1. Move to pre-grasp height.  
2. Approach and grasp the object.  
3. Return to home position.  

Feedback (to both the DNF and the vision system) is sent **manually** or automatically if `automatic_robot_feedback = True`.


#### fake_robot_controller_node

*   **Role:** A **"stunt double"** for the real Tiago robot. Simulates a simple robot arm and gripper for testing  task logic. 
*   **Why Recall Only:** Simulates the robot's physical body and low-level controller.
*   **Key Feature:** Listens for commands from `tiago_task_executive`, waits for a simulated time, and publishes fake pose feedback.
*   **Mode:** Used when `mode:=tiago`.
*   **Important:** This node should **NOT** be used with the real robot, as the robot's own drivers serve this purpose.
*   **Subscribes:**  
    * `/dxl_input/pos_right` (`PoseStamped`) – target arm pose commands.  
    * `/dxl_input/gripper_right` (`PointStamped`) – gripper open/close commands.  
*   **Publishes:**  
    * `/cartesian/right_arm/pose` (`PoseStamped`) – continuous simulated arm pose feedback. 



#### udp_listener_node

* **Role:** Listens for UDP voice commands and publishes them to ROS.  
* **Publishes:** `/voice_message` (`std_msgs/String`) — recognized voice command.  
* **Receives:** UDP messages on port `5005` (commands like `give_motor`, `give_load`, etc.).  
* **Sends:** UDP confirmations to `10.205.240.222:5006`.  
* **Notes:** Each command can be published up to `REPEAT_COUNT` times.  
* **Dependencies:** `rospy`, `std_msgs`, `socket`, `threading`.

### Parameters
| Variable | Description | Default |
|-----------|--------------|----------|
| `REPEAT_COUNT` | Maximum number of times each command can be published | `2` |


#### udp_response_sender_node

* **Role:** Sends ROS string messages as UDP packets to a target IP and port.  
* **Subscribes:** `/response_command` (`std_msgs/String`) — response message to send.  
* **Sends:** UDP packets to `target_ip:target_port` (default `127.0.0.1:5006`).  
* **Dependencies:** `rospy`, `std_msgs`, `socket`.  




### NODES USED IN BOTH PHASES

#### fake_ar_publisher_node

*   **Role:** Simulates the world state and AR marker detections.
*   **Publishes:** AR marker data (reacts to robot pickups).
*   **Subscribes:** Robot action commands (to update world state).

#### robot_vision_bridge_node

*   **Role:** Translates AR marker data to a simple JSON format.
*   **Publishes:** `/object_detections` (String, JSON).
*   **Subscribes:** AR marker topic from `fake_ar_publisher`.

#### vision_to_dnf_aggregator_node

*   **Role:** Aggregates vision detections, robot feedback, and human voice commands into unified DNF inputs.  
*   **Publishes:** `/dnf_inputs` (`Float32MultiArray`, 3-part array: [vision | robot | human]).  
*   **Subscribes:**  
    *   `/object_detections` (vision-based object positions).  
    *   `/simulation/robot_feedback` (automatic robot signals).  
    *   `/manual_robot_feedback` (manual robot feedback for testing).  
    *   `/parsed_voice_command` (human voice commands).  
    *   `/response_command` (resets active human voice inputs (after error, so the next one is possible)).

