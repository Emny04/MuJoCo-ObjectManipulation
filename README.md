# MuJoCo-ObjectManipulation

This project utilizes MuJoCo to simulate object manipulation tasks, including pick-and-place operations. It showcases how to control a robotic gripper to grasp and place objects, offering a realistic environment for testing manipulation algorithms and grasping techniques.

## Requirements

To run this project, you will need:

- **MuJoCo** (version 2.1.0 or above)  
  Download and install MuJoCo from [https://mujoco.org/](https://mujoco.org/).
  
- **Python 3.x**  
  This project is compatible with Python 3.7 or above.

- Required Python packages:
  - `numpy`
  - `mujoco-py`
  - `time`
  - `src` (custom modules from your project)

You can install the required Python packages using `pip`:

```bash
pip install numpy mujoco-py
```

## How to Run

### Clone the Repository
First, clone the repository to your local machine:
```bash
git clone https://github.com/yourusername/MuJoCo-ObjectManipulation.git
```

### Install Dependencies
Navigate into the project directory and install the necessary Python dependencies:
```bash
cd MuJoCo-ObjectManipulation
pip install -r requirements.txt
```

### Run the Simulation
After setting up the environment, run the main script:
```bash
python main.py
```

This will start the simulation, and you should see the robot performing object manipulation tasks.

### Control the Gripper
The gripper can be set to open or close during different phases of the task. Adjust the option in the `set_gripper` function to control it ("open" or "close").

## Project Structure

* `main.py`: The main file that contains the logic for simulating the object manipulation tasks using the MuJoCo environment.
* `src/`: Custom Python modules used in the project, including:
   * `mujoco_parser.py`: A parser for MuJoCo simulation data.
   * `PID.py`: A PID controller implementation for robotic joint control.
   * `get_grasp_pose_using_ik.py`: Inverse Kinematics solver for getting the grasp pose of the gripper.
* `asset/panda/`: Folder containing MuJoCo assets for the Panda robot and objects to manipulate.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
