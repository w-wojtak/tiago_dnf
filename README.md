
### OVERVIEW

| Phase         | Number of Nodes | Purpose                                   |
| ------------- | --------------- | ----------------------------------------- |
| Learning Only | 2               | Demonstrate and learn the task            |
| Recall Only   | 3               | Execute and adapt based on learned memory |
| Both Phases   | 3               | Shared infrastructure (vision pipeline)   |

### LEARNING-ONLY NODES

#### human_action_simulator_node
 
Role: Simulates human demonstrator
Why Learning Only: In recall, the robot acts autonomously; no human demonstration needed
Publishes: /simulation/human_pickup
Key Feature: Scheduled pickup times


#### dnf_model_learning_simple_node (the "brain" of the architecture)

Role: Observes demonstration and learns sequence
Subscribes: /dnf_inputs
Saves: u_sm.npy, u_d.npy (sequence memory & duration)
Fields: 2 (u_sm, u_d)


### RECALL-ONLY NODES

#### fake_voice_commander
 
Role: Simulates human verbal cues
Why Recall Only: Used to communicate that the human is waiting for the robot's action
Publishes: /voice_command
Input Type: Permanent Gaussians


#### dnf_model_recall_simple_node  (the "brain" of the architecture)

Role: Predicts next actions using learned memory. This is the "performer" that executes the learned task

Subscribes: /dnf_inputs
Publishes: /threshold_crossings (predictions)
Loads: u_sm.npy, u_d.npy, h_u_amem.npy
Saves: h_u_amem.npy (trial-specific adaptive memory)
Fields: 6 (u_act, u_sim, u_wm, u_f1, u_f2, u_error)


#### task_executive_node

Purpose: translates DNF predictions into robot actions
Publishes: /simulation/robot_feedback (String)
Subscribes: /threshold_crossings (Float32MultiArray)



### NODES USED IN BOTH PHASES

#### fake_ar_publisher_node

Purpose: Simulates  world state and AR marker detections
Publishes: AR marker data (reacts to robot pickups)
Subscribes: Robot action commands (to update world state)


#### robot_vision_bridge_node

Purpose: Translates AR markers to simple JSON format
Publishes: /object_detections (String, JSON)
Subscribes: AR marker topic from fake_ar_publisher

#### vision_to_dnf_node
 
Role: Converts detections to DNF-compatible Gaussian inputs

In Learning:
Only vision input is active (temporary Gaussians)
Robot feedback and voice inputs are zero/unused

In Recall:
Vision input (temporary)
Robot feedback (permanent, accumulates)
Human voice (permanent, accumulates)
Publishes: /dnf_inputs (3-part array: [vision | robot | human])