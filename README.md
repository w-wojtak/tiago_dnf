
### Dependencies

*   ROS Noetic
*   Python 3
*   `rospy`, `numpy`, `scipy`
*   `matplotlib` (for plotting and visualization; requires a GUI backend such as `TkAgg` or `Qt5Agg`)
*   `ar_track_alvar`: The `fake_ar_publisher_node` publishes messages of this type to simulate the real `ar_track_alvar` package. This is only required for simulating QR code detection.
 
### Phase 1: Learning the Task

This phase runs a simulation where a human demonstrator performs the assembly task. The DNF system observes these actions and learns the sequence and timing.

**To Run Learning:**

`roslaunch fake_tiago_pkg dnf_learning_pipeline.launch`

What it does: Observes a pre-scripted demonstration of an assembly task.
Output: Saves the learned memory files (`u_sm_TIMESTAMP.npy` and` u_d_TIMESTAMP.npy`) into the `data_basic/` directory. 

**There are three available learning scenarios:**

1. Robot’s Temporal Adaptation — Run using the dnf_learning_pipeline.launch file.
The standard setup using simulated QR code detections.

2. Extended Architecture (with Error Detection) — Run using the dnf_learning_extended_pipeline.launch file.
Includes the additional error detection mechanism integrated into the DNF architecture.

3. Simulated Learning (No External Inputs) — Run using the dnf_learning_simulated.launch file.
Runs the learning process with predefined inputs specified in a file. This mode does not connect to any QR code simulation or real sensors and is useful for testing or visualizing how the learning mechanism works in isolation.


### Phase 2: Recalling the Task

In this phase, the system uses the learned memory from **Phase 1** to predict and execute the assembly task. The recall phase reproduces the learned sequence and timing to drive robot actions, either in simulation or on the real robot.

---

#### 1. Simulation Mode (Default)

This mode is used for quickly testing the DNF’s prediction and recall logic without involving real robot movements or complex communication pipelines.

**How it works:**
Uses `simulated_task_executive.py`, which waits for fixed time intervals (`time.sleep()`) to simulate robot actions. This allows you to verify that the DNF correctly recalls the learned sequence and generates predictions as expected.

**To run:**

```bash
# Default recall mode (no real robot interaction)
roslaunch fake_tiago_pkg dnf_recall_pipeline.launch
```

You can also choose among several simulation configurations depending on the scenario:

* **Temporal Adaptation:**

  ```bash
  roslaunch fake_tiago_pkg dnf_recall_simple.launch
  ```
* **Extended Architecture (with Error Detection):**

  ```bash
  roslaunch fake_tiago_pkg dnf_recall_extended.launch
  ```
* **Without Voice Inputs:**
  Versions of the above that do not use real or simulated voice commands are available as:

  ```bash
  roslaunch fake_tiago_pkg dnf_recall_novoice_pipeline.launch
  roslaunch fake_tiago_pkg dnf_recall_extended_novoice_pipeline.launch
  ```

---

#### 2. Running on the Real Tiago Robot

When deploying on the real Tiago, the main difference is that **no “fake” input or output nodes** should be used. Instead, the DNF system must connect directly to the appropriate **ROS topics** used by the real robot’s perception and control stacks.

**Integration guidelines:**

* **Perception:**
  In simulation, QR-code detection is mimicked using the `fake_ar_publisher` node, which publishes `ar_track_alvar_msgs/AlvarMarkers` messages to `/ar_pose_marker`.
  On the real robot, this node should be **disabled**, and the DNF should instead subscribe to the corresponding topic published by the **real perception system** (e.g., ZED camera or Tiago’s onboard vision).

* **Control:**
  In simulation, robot actions are represented by simple time delays or fake controllers.
  On the real robot, these should be replaced by the actual control interface.
  The DNF output nodes must **publish robot commands** (e.g., arm trajectories, gripper control, or task triggers) to the appropriate ROS topics expected by the **Tiago control stack** (e.g., OpenSOT or MoveIt).

* **Launch configuration:**
  You can integrate the DNF nodes into the Tiago’s main launch file alongside its drivers and control framework:

  ```xml
  <!-- Example addition to Tiago’s main launch file -->

  <!-- The DNF Recall Node -->
  <node name="dnf_recall_node" pkg="fake_tiago_pkg" type="dnf_model_recall_simple_node.py" output="screen" />

  <!-- Vision-to-DNF and other interface nodes as required -->
  ```

**Important:**
Do **not** launch any “fake” publishers or simulated controllers (e.g., `fake_robot_controller.py`, `fake_ar_publisher`, etc.) when running on the real robot. The DNF should operate solely on the live sensory input and control output topics defined in the Tiago environment.




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

