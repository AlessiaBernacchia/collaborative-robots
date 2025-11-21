# 🤖 Cooperative Robotics for Assembly Task
Project of the course of [Robot Controls Application](https://www.icorsi.ch/course/view.php?id=21192).

*This proposal outlines a project focused on developing a collaborative system where two robotic agents work together to achieve a common assembly goal.*

## 🪢 Teams members
- [Alessia Bernacchia](https://github.com/AlessiaBernacchia)
- [Alessandro Mecchia](https://github.com/AlessandroMecchia)
- [Giacomo Villani](https://github.com/DownToTheGround)

## 🎯 Main Objective
The main objective of this project is to develop a control program for **two cooperating robotic arms** to successfully execute a **simple block-stacking task** (building a basic tower). 

The primary focus is on establishing robust inter-robot communication and collision avoidance. 

## 📌 Requirements
### 🧱 Minimum Project Requirements
* develop a code program that simulates and controls, using [**Swift**](https://pythonrobotics.io/), two ***Pandas*** arm robotic manipulators
* the task is to build **a simple, vertical tower** using a predefined number of blocks, where each robot has own resource pool
* the robots must successfully **collaborate** to complete the tower
* to make the collaboration successfull is fundamental to implement a mechanism to ensure collision avoidance between the two robot arms (even if they operate at different simulated speeds)

### 🏰 Extra Possible Developments and Expansions
Potential advanced features to exceed the minimum requirements and deepen the project's technical complexity:
#### 🎲 Alternative Structures and Dynamics
* **Sequential Pattern**: complex task where the tower must be built with a defined sequence of colors, shapes, or materials; each robot must verify that the piece it is about to place follow the required sequence before committing to the action.
* **Complex Structure**: replace the simple towel with a more challenging structure, such as a small wall or a bridge segment.
* **Asynchronous Operations**: develop the collaboration protocol to be highly robust to asynchronous operation where robots may pause unexpectedly or have variable processing times, ensuring the system doesn't deadlock.
* **Dynamic Obstacle**: introduce a non-static element (e.g., a simple moving obstacle or temporary blockage) that the robots must dynamically avoid while executing their tasks, requiring real-time path replanning.

#### 🧠 Different Robots Roles
* **Error Checking/Verifier**: one robot acts as a *verifier*, checking the stability or correctness of the stack built by the other robot (e.g., checking for tilt or misalignment before the next piece is placed).
* **Wrecker**: one robot *constructs* a segment of the wall/tower, while the other robot is dedicated to *disassembling* it (e.g. removing random blocks or 'defective' pieces).
* **Refiller (Frankie robot)**: introduce a limited pool of common resources, in this scenario at least one robot is a ***Frankie*** arm mobile robotic manipulator able to move in the environment; one robot is designated as the *logistic refiller*: it must monitor the resource level and, when the common pool drops below a threshold, this robot must refill it from a separate, larger supply area (or take the blocks sparse around).
### ✨Just for fun
* **FINAL CLAP**: upon successful completion of the main assembly task, the two robots must execute a programmed synchronous "clap" gesture as a sign of job completion and success.