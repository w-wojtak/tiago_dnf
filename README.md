### OVERVIEW

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