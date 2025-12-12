# 🤖 Cooperative Robotics for Assembly Task

This project was developed for the course **Robot Controls Application** ([course link](https://www.icorsi.ch/course/view.php?id=21192)).

## 🪢 Teams members
- [Alessia Bernacchia](https://github.com/AlessiaBernacchia)
- [Alessandro Mecchia](https://github.com/AlessandroMecchia)
- [Giacomo Villani](https://github.com/DownToTheGround)

## 📝 Project Description

This repository contains a **collaborative simulation system** where two robotic manipulators, modeled as **Panda arms** using the **Swift** library, work together to achieve a common assembly goal: **building a vertical tower**.

The core focus of this implementation is establishing a robust **cooperation protocol** that manages shared resources and successfully handles **collision avoidance** between the two robotic agents, even when operating at variable speeds.

> Here the original link to the project description made for the course: [original description of the project](./ProjectDescription.md).

### 🎯 Key Features Implemented

  * **Dual Panda Arm Simulation:** Control and simulation of two independent Panda robotic manipulators.
  * **Safe Waypoint Trajectory Generation:** Robots utilize custom waypoint planning that ensures movements are executed at a pre-defined **"safe height"** when moving horizontally to prevent collision with or destruction of existing tower structures.
  * **Autonomous Tower Construction:** Each robot can independently construct parts of the final target tower, dynamically selecting the most accessible and highest-placed bricks from their resource pool.
  * **Synchronized Cooperation:** Implementation of a **lock mechanism** to manage common resources (bricks) and the shared target structure, preventing race conditions and ensuring successful collaboration.
  * **Robust Collision Avoidance:** A mechanism that actively prevents collision between the two robot arms, proven to function reliably even when the robots operate at different simulated speeds.

## ⚙️ Technologies Used

  * **Python:** Primary programming language.
  * **Swift (Python Robotics):** Used for the simulation and control of the Panda robotic manipulators.
  * **Standard Python Libraries:** (You can add specific ones here if necessary, e.g., NumPy, Matplotlib, if you used them for plotting/calculations.)

-----

## 📁 Project Structure

```
project_folder/
├── classes/
│   ├── __pycache__/
│   ├── __init__.py
│   ├── controller.py      # Contains logic for robot control and movement planning.
│   ├── objects.py         # Defines the objects in the environment (e.g., bricks, towers).
│   ├── robot.py           # Defines the Panda robot agent and its core functionalities.
│   ├── sensor.py          # Likely handles sensor data or environment state reading.
│   └── task_manager.py    # Manages the overall assembly task flow and coordination.
├── plots/
│   ├── _1a/
│   ├── ...                # Subdirectories likely containing plots for different scenarios.
│   └── plot_metrics_and_trajectory.png # Final output plot showing results/paths.
├── .gitignore
├── main.py                # Main execution script to start the simulation.
├── Pipfile
├── Pipfile.lock
├── ProjectDescription.md  # Original project brief.
├── README.md              # This file.
└── requirements.txt       # List of required Python packages for installation.
```

-----

## 🚀 Installation and Setup

This project uses **Python 3.11** and `pipenv` for dependency management. To set up the required environment and run the simulation, follow these steps:

### 1\. Prerequisites

Ensure you have the following installed on your system:

  * **Python 3.11**
  * **pipenv:** You can install it globally via pip: `pip install pipenv`
  * **git**

### 2\. Environment Setup

1.  **Clone the Repository:**

    ```bash
    git clone https://github.com/AlessiaBernacchia/collaborative-robots.git
    cd your_repo_name
    ```

2.  **Create and Install Environment:**
    The `Pipfile` or `requirements.txt` contains all necessary dependencies, including `robotics-toolbox-python` and `swift`.

    ```bash
    pipenv install -r requirements.txt
    ```

3.  **Activate the Virtual Environment:**

    ```bash
    pipenv shell
    ```

### 3\. **Critical Fix for Swift/Websockets**

Due to a compatibility issue between `swift` and recent versions of `websockets` (especially on Windows/Linux environments), the following manual steps are required within the activated environment to ensure the simulation launches correctly:

1.  **Downgrade `websockets`:**

    ```bash
    pip uninstall websockets
    pip install websockets==13.1
    ```

2.  **Modify `swift/Swiftroute.py`:**
    You need to manually edit a file within your installed `swift` package. Locate the `Swiftroute.py` file inside your environment's packages and change line **390** from its current state to the following:

    ```python
    self.path = urllib.parse.unquote(self.path[10:])
    ```

    > **Note:** The exact path to this file will vary based on your operating system and environment location (e.g., `.../.venv/lib/python3.11/site-packages/swift/Swiftroute.py`).

---

## ▶️ Usage and Simulation Scenarios

The `main.py` script serves as the primary interface for running simulations, configuring the environment, launching the visualization, and generating all performance plots.

### 1. `main.py` Execution Flow

The `main()` function handles the following sequence of operations:

1.  **Configuration Check:** Displays the configurable global parameters:
    * `GAIN` ($K_p$ in the controller, default **2.5**): Higher gain results in faster (but potentially less stable) robot movements.
    * `DT` (Time Step, default **0.01**): The simulation step size.

2.  **Task Selection:** Prompts the user to select one of the 8 predefined simulation scenarios (e.g., `'1b'`, `'4a'`).

3.  **Environment Initialization:** Calls the `task()` function, which:
    * Initializes the `swift` simulation environment.
    * Calls the appropriate `initialize_task_X()` function to define the number of robots, the position and color of the `Brick` resources, and the number/goal of the `Tower` targets.

4.  **Simulation Run:** The `run_task()` function:
    * Registers all `Robot_arm`s and `Brick`s into the simulation environment.
    * Initializes the **`Sensor`**, **`Controller`**, and **`TaskManager`** instances.
    * Calls `task_manager.start()`, which spawns separate threads for each robot to work in parallel.

5.  **Data Plotting and Saving:** Once the tasks are complete, `main.py`:
    * Calculates global axis limits (`global_limits`).
    * Calls `plot_all_metrics_combined` to generate and display the three synchronized performance, collision, and trajectory plots.
    * Saves all figures (including metrics, trajectories, and inter-robot distance) into the dedicated `./plots/_<task_name>` subdirectory.

### 2. Available Simulation Tasks

The project includes 8 distinct cooperative assembly scenarios, categorized by complexity and collaboration requirements:

| Task Name | Number of Robots | Target Structure | Collaboration Focus |
| :--- | :--- | :--- | :--- |
| **`1a`** | **One** Panda | Single Tall Tower | Single-robot motion and obstacle avoidance (the bricks themselves). |
| **`1b`** | **Two** Pandas | Single Tall Tower | **Core Collaboration:** Resource locking and collision avoidance around a single, central task point. |
| **`2a`** | **One** Panda | Multiple Towers (Wall) | Sequential task switching and broad reach planning. |
| **`2b`** | **Two** Pandas | Multiple Towers (Wall) | **Advanced Resource Management:** Task allocation across three distinct target towers. |
| **`3a`** | **One** Panda | Two Towers (Color-Defined) | Sequential pick-and-place based on *color* requirements (resource type management). |
| **`3b`** | **Two** Pandas | Two Towers (Color-Defined) | **Complex Resource/Target Matching:** Robots must coordinate to pick the correct color brick *and* lock the corresponding color tower. |
| **`4a`** | **One** Panda | Wall with Color Pattern | Single-robot execution of complex, mixed-color task sequence. |
| **`4b`** | **Two** Pandas | Wall with Color Pattern | **Full System Test:** Parallel, multi-colored assembly with collision avoidance and complex shared resource/target locks. |

To launch a simulation, simply run `python main.py` and input the desired task index (e.g., `4b`) when prompted. The simulation will launch in a browser window, and you will see the two Panda robots performing the block-stacking task in parallel. At the end the plots will be shown in other windows (if specified they could be saved).

## 💻 Core Classes and Architecture (in `classes/`)

The simulation logic is organized into four main Python classes that manage the physical environment, robot control, sensing, and high-level task coordination.

### 1. `Brick` and `Tower` (in `objects.py`)

These classes define the fundamental elements of the assembly task, including the resource management logic necessary for multi-robot collaboration.

| Class | Purpose | Key Attributes & Methods |
| :--- | :--- | :--- |
| **`Brick`** | Represents a physical block in the environment, utilizing `spatialgeometry.Cuboid` for simulation. | `__init__`: Initializes pose, scale, and color. `lock`/`unlock`/`try_lock`: **Implements a mutex lock** to prevent multiple robots from simultaneously targeting or moving the same brick resource. |
| **`Tower`** | Represents the target structure to be built. | `get_next_pose()`: Calculates the 3D pose for the next brick placement. `is_complete()`: Checks if the target height (`max_height`) has been reached. `lock`/`unlock`/`try_lock`: **Implements a mutex lock** to synchronize access when adding a new brick to the tower stack. |

### 2. `Robot_arm` (in `robot.py`)

This class acts as an **Agent** wrapper around the core `rtb.models.Panda` manipulator. It manages the robot's state and performance metrics.

| Class | Purpose | Key Attributes & Methods |
| :--- | :--- | :--- |
| **`Robot_arm`** | Manages the Panda robot model, tracking its status and history. | `is_busy()`: Checks if the robot is currently executing a motion task. `apply_velocity_cmd()`: Updates the robot's joint velocities ($\dot{q}$) based on the controller output. `record_collision_event()`: Stores data points whenever a near-collision is detected by the sensor/task manager for post-simulation analysis. |

### 3. `Sensor` (in `sensor.py`)

The `Sensor` class acts as the centralized point for querying the current state of the environment, essential for decision-making and safety.

| Class | Purpose | Key Functionality |
| :--- | :--- | :--- |
| **`Sensor`** | Provides real-time environmental data (e.g., list of bricks, towers, robots). | `get_available_bricks()` / `get_available_towers()`: Returns only resources that are **unlocked** (not currently being used by another robot). `check_collision()`: **Core Safety Check.** Returns `True` if the end-effectors of the two robots are closer than a defined `safe_dist`. |

### 4. `Controller` (in `controller.py`)

The `Controller` translates high-level goals into safe, low-level joint velocity commands, managing the physical motion of the robot.

| Class | Purpose | Key Functionality |
| :--- | :--- | :--- |
| **`Controller`** | Implements the motion control loop (Inverse Kinematics) and path generation. | `generate_path_points()`: Generates the safe, multi-step trajectory for pick and place, including the crucial **"safe height"** transition point to prevent collisions with existing structures. `compute_qdot()`: Calculates the required joint velocity ($\dot{q}$) using the **Damped Pseudoinverse Jacobian** method to minimize the end-effector error. `move_to_pose()`: Executes the movement, calling the `TaskManager`'s collision resolution logic at *every time step* to ensure safety. |

### 5. `TaskManager` (in `task_manager.py`)

This is the **coordination hub** responsible for sequencing the high-level tasks, assigning resources, and implementing the multi-robot collaboration rules.

| Class | Purpose | Key Functionality |
| :--- | :--- | :--- |
| **`TaskManager`** | Orchestrates the entire assembly process using Python threading to run robots in parallel. | `available_brick(agent)`: Implements the **resource selection strategy** (highest and nearest brick within reach). `resolve_collision_precedence()`: **Critical Collaboration Logic.** If `check_collision()` is true, it compares the robots' remaining distance-to-target to determine which robot has **precedence** (can continue moving) and which must pause/move to a safe transition point. |

This is crucial information, as the combination of plots gives the full picture of your cooperative control system's performance and safety!

I will now create the **"Performance and Trajectory Analysis"** section, detailing both the individual robot metrics (from the `Robot_arm` class) and the combined, high-level analysis (from your `main.py` functions).


## 📊 Performance and Trajectory Analysis

The project includes extensive plotting capabilities to visualize the robots' performance, trajectory, and the effectiveness of the collision avoidance system. These visualizations are generated by methods within the `Robot_arm` class and aggregated into comprehensive figures by the main script using `plot_all_metrics_combined`.

### 1. Robot-Specific Trajectory Plots

The `Robot_arm` class includes methods to plot the end-effector trajectory across three 2D projections, often combined using `plot_3d_trajectory_views` in the main script. This allows for a complete spatial assessment of the robot's movement paths. 

| Method | Plot Type | Key Axes | Purpose |
| :--- | :--- | :--- | :--- |
| `plot_top_view` | **Top View** (XY Plane) | X position vs. Y position | Shows horizontal movement and coverage area. |
| `plot_side_view` | **Side View** (XZ Plane) | X position vs. Z position | Shows reaching depth and lifting height. |
| `plot_front_view` | **Front View** (YZ Plane) | Y position vs. Z position | Shows lateral movement and lifting height. |

### 2. Robot-Specific Metric Plots

These methods, also part of the `Robot_arm` class, track the robot's internal control state over time to diagnose performance.

| Method | Metric | Key Axes | Purpose |
| :--- | :--- | :--- | :--- |
| `plot_performance_metrics` | **Joint Velocities ($\dot{q}$)** | Time vs. Joint Angle Rate | Monitors control command magnitude to ensure stability and reasonable acceleration/deceleration. |
| `plot_performance_metrics` | **Jacobian Condition Number ($\kappa$)** | Time vs. $\kappa$ Value | Identifies instances of **near-singularity** (high $\kappa$) during movement, which indicates potential control difficulty or loss of dexterity. |
| `plot_height_over_time` | **End-Effector Height (Z)** | Time vs. Z Position | Clearly shows the sequence of tasks: rising to the `MAX_SAFE_LIFT_HEIGHT`, horizontal movement, and precise lowering for brick placement. |

### 3. Combined Cooperative Analysis (The Main Result)

The `main.py` utilizes `plot_all_metrics_combined` to synchronize and overlay data from both robots across multiple figures, providing direct insight into the collaboration:

| Plot | Description | Insight Provided |
| :--- | :--- | :--- |
| **Combined Performance** | Joint Velocity ($\dot{q}$) and Condition Number ($\kappa$) for both robots on the same time axis. | Allows direct comparison of robot effort and stability during parallel tasks. |
| **Combined Trajectory** | Overlays the 3D trajectory views of both robots. | Visually confirms that the planned paths do not intersect dangerously and that the "safe height" transitions are utilized for horizontal movement. |
| **Inter-Robot Distance** | **(Crucial Safety Plot)** Plots the Euclidean distance between the two end-effectors over time.  | Shows that the distance never drops below the defined **Safety Threshold (0.2m)**, validating the effectiveness of the `resolve_collision_precedence` logic in `TaskManager`. |

All generated figures are saved to the `./plots` directory using the `save_image` utility function for easy documentation and review.
