import numpy as np
import time
from src.mujoco_parser import MuJoCoParserClass
from src.PID import PID_ControllerClass
from get_grasp_pose_using_ik import get_q_from_ik

def set_gripper(desired_q, option="open"):
    """
    Set the gripper to open or close by modifying the last two joint angles.
    """
    if option == "open":
        desired_q[7], desired_q[8] = np.pi, -np.pi  # Open gripper
    elif option == "close":
        desired_q[7], desired_q[8] = 0.0, 0.0  # Close gripper
    return desired_q

def main():
    # MuJoCo Panda parsing
    xml_path = 'asset/panda/franka_panda_w_objs.xml'
    env = MuJoCoParserClass(name='Panda', rel_xml_path=xml_path, VERBOSE=False)
    env.forward()

    # Select the last 9 indices for Panda robot joints
    n_joints = env.n_ctrl
    panda_joint_idxs = np.array(range(n_joints-9, n_joints))
        
    # Get IK solutions for the task
    pre_grasp_q, rotate_eef_q_lst, move_down_q_1_lst, move_down_q_2_lst = get_q_from_ik(env)
    # Initialize MuJoCo viewer
    env.init_viewer(viewer_title="Custom Robot Control", viewer_width=1600, viewer_height=900,
                    viewer_hide_menus=False)
    env.update_viewer(cam_id=0)
    env.reset()

    # Get control ranges for Panda joints only
    ctrl_ranges = env.ctrl_ranges[panda_joint_idxs]

    # Initialize PID controller for joint control (only last 9 joints)
    PID = PID_ControllerClass(
        name='PID',  
        dim=9,  # 7 arm joints + 2 gripper joints
        k_p=400.0,  # Reduced from 800.0
        k_i=10.0,   # Reduced from 20.0
        k_d=50.0,  
        out_min=ctrl_ranges[:, 0],
        out_max=ctrl_ranges[:, 1],
        ANTIWU=True
    )
    PID.reset()
    

    task_sequence = [
        {"task": "pre_grasp", "desired_q": pre_grasp_q, "gripper": "open"},
        {"task": "rotate_eef", "desired_q": rotate_eef_q_lst[0], "gripper": "open"},
        {"task": "move_down_1", "desired_q": move_down_q_1_lst[0], "gripper": "open"},
        {"task": "grasp", "desired_q": move_down_q_1_lst[0], "gripper": "close"},
        {"task": "pre_grasp_with_close", "desired_q": pre_grasp_q, "gripper": "close"},
        {"task": "move_left_with_close", "desired_q": pre_grasp_q + np.array([0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), "gripper": "close"},
        {"task": "rotate_eef_with_close", "desired_q": rotate_eef_q_lst[1] + np.array([0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), "gripper": "close"},
        {"task": "move_down_2_with_close", "desired_q": move_down_q_2_lst[1] + np.array([0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), "gripper": "close"},
        {"task": "release", "desired_q": move_down_q_2_lst[1]+ np.array([0.9, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]), "gripper": "open"},
        {"task": "return_with_open", "desired_q": pre_grasp_q, "gripper": "open"}
    ]

    # Initialize scene and wait for stability
    print("Initializing scene... Please wait.")
    initialization_steps = 100
    for _ in range(initialization_steps):
        full_ctrl = np.zeros(env.n_ctrl)
        env.step(ctrl=full_ctrl)
        env.render()

    print("Scene initialized. Starting task sequence...")
    input("Press enter to begin:")

    max_tick = 1000000
    task_idx = 0
    steps_per_task = 1500

    while env.tick < max_tick:
        if task_idx >= len(task_sequence):
            print("Task sequence completed!")
            break

        current_task = task_sequence[task_idx]
        desired_q = current_task["desired_q"]
        desired_q = set_gripper(desired_q, option=current_task["gripper"])

        print(f"[{env.tick}] Task {task_idx + 1}: {current_task['task']} with gripper {current_task['gripper']}")

        # Get current joint positions for Panda joints only
        current_q = env.get_q(joint_idxs=panda_joint_idxs)
        
        # Update PID controller with the last 9 elements of desired_q
        PID.update(x_trgt=desired_q[-9:])  # Pass only the last 9 elements
        PID.update(t_curr=env.get_sim_time(), x_curr=current_q, VERBOSE=False)
        torque = PID.out()
        
        # Create a zero array for all joints and set only the Panda joint torques
        full_ctrl = np.zeros(env.n_ctrl)
        full_ctrl[-9:] = torque  # Apply torques to last 9 indices
        
        # Apply the control
        env.step(ctrl=full_ctrl)
        time.sleep(0.001)
        # Render
        if (env.tick % 3) == 0:
            env.render()

        # Move to the next task after a fixed number of steps
        if env.tick % steps_per_task == 0:
            task_idx += 1
    input("Press enter to exit:")
    env.close_viewer()
    print("Simulation complete!")

if __name__ == "__main__":
    main()