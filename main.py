import mujoco
import mujoco.viewer
import numpy as np
import time
from Utils.utils import *
from controller import OpsaceController
from mujoco_ar import MujocoARConnector
from Model.model import *
import random
import threading
from collections import deque
import cv2
from ultralytics import YOLO
from PIL import Image


class ImitationSimulation:
    
    def __init__(self):

        # Configs
        self.scene_path = 'Environment/scene.xml'
        self.mjmodel = mujoco.MjModel.from_xml_path(self.scene_path)
        self.mjdata = mujoco.MjData(self.mjmodel)
        self.dt = 0.002
        self.grasp = 0
        self.toggle = True
        self.button_state = 0 


        self.placement_time = -1
        self.rgb_renderer = mujoco.Renderer(self.mjmodel, height=360, width=480)
        self.depth_renderer = mujoco.Renderer(self.mjmodel, height=360, width=480)
        self.depth_renderer.enable_depth_rendering()
        self.cameras = ["front_camera","side_camera","top_camera","gripper_camera"]
        self.yolo_model = YOLO('yolov8m.pt')

        # Override the simulation timestep
        self.pick_objects = ["banana","orange","banana2","orange2"]
        self.pick_joints = ["banana_joint","orange_joint","banana2_joint","orange2_joint"]
        self.place_object = "plate"
        self.mjmodel.opt.timestep = self.dt
        self.frequency = 1000
        self.target_pos = np.array([0.5, 0.0, 0.4])
        self.target_rot = rotation_matrix_x(np.pi)@np.identity(3)
        self.pos_origin = self.target_pos.copy()
        self.rot_origin = self.target_rot.copy()
        self.target_quat = np.zeros(4)
        self.eef_site_name = 'eef'
        self.site_id = self.mjmodel.site(self.eef_site_name).id
        self.camera_name = 'gripper_camera'
        self.last_recording_time = -1
        self.camera_data = None
        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
            
        ]

        # Recording and Policy Related
        self.record = False
        self.run_policy = False
        self.recording_frequency = 10
        self.prev_datas = deque(maxlen=10)
        self.prev_times = deque(maxlen=10)

        # Controller
        self.controller = OpsaceController(self.mjmodel,self.joint_names,self.eef_site_name)
        self.q0 = np.array([0, 0.2686, 0, -1.5423, 0, 1.3307, 0])
        self.dof_ids = np.array([self.mjmodel.joint(name).id for name in self.joint_names])
        self.actuator_ids = np.array([self.mjmodel.actuator(name).id for name in self.joint_names])
        self.grasp_actuator_id = self.mjmodel.actuator("fingers_actuator").id 
        self.mjdata.qpos[self.actuator_ids] = self.q0

        # MujocoAR Initialization
        self.mujocoAR = MujocoARConnector(mujoco_model=self.mjmodel,mujoco_data=self.mjdata)

        # Linking the target site with the AR position
        self.mujocoAR.link_site(
            name="eef_target",
            scale=2.75,
            position_origin=self.pos_origin,
            toggle_fn=self.toggle_mode,
            button_fn=lambda: self.button_pressed(),
            disable_rot=True,
        )



    def detect_objects(self):
        camera_data = self.get_camera_data()
        gripper_rgb = camera_data.get("gripper_camera_rgb")
        if gripper_rgb is None:
            return []

        
        input_image = cv2.resize(input_image, (640, 640))

        results = self.yolo_model(input_image)
        result_image = results[0].plot()
        Image.fromarray(result_image).show()

        objects = []
        for box in results[0].boxes:
            cls = int(box.cls)
            name = results[0].names[cls]
            coords = box.xyxy.numpy()
            objects.append((name, coords))
        return objects

    def get_camera_data(self):
        data = {}
        for camera in ["gripper_camera"]:
            self.rgb_renderer.update_scene(self.mjdata, camera)
            data[camera + "_rgb"] = self.rgb_renderer.render()
        return data
    
    def is_valid_position(self, pos1, pos2_multiple, min_dist):
        if pos1 is None or pos2_multiple is None:
            return False
        for pos2 in pos2_multiple:
            distance = np.linalg.norm(np.array(pos1[:2]) - np.array(pos2[:2])) 
            if distance<min_dist:
                return False
        for i, pos2 in enumerate(pos2_multiple):
            for j, pos2_2nd in enumerate(pos2_multiple):
                if i==j:
                    continue
                if pos2_2nd is None or pos2 is None:
                    return False
                distance = np.linalg.norm(np.array(pos2_2nd[:2]) - np.array(pos2[:2])) 
                if distance<min_dist:
                    return False
        return True
        
    def get_camera_data(self) -> dict:
        data = {}    
        for camera in self.cameras:
            self.rgb_renderer.update_scene(self.mjdata, camera)
            self.depth_renderer.update_scene(self.mjdata, camera)
            data[camera+"_rgb"] = self.rgb_renderer.render()
            data[camera+"_depth"] = self.depth_renderer.render()
        self.prev_datas.append(data)
        self.prev_times.append(time.time())
        return data
    
    def get_pos_from_range(self,range):
        return np.array([random.uniform(range[0,0], range[0,1]), random.uniform(range[1,0], range[1,1]),range[2,0]])

    def was_placed(self):
        for pick_object in self.pick_objects:
            pick_pos = self.mjdata.body(pick_object).xpos.copy()
            place_pos = self.mjdata.body(self.place_object).xpos.copy()
            if np.linalg.norm(np.array(pick_pos)[0:2] - np.array(place_pos)[0:2]) > 0.07 or pick_pos[2]>0.25:
                return False
        return True
        
    def fell(self):
        for pick_object in self.pick_objects:
            pick_pos = self.mjdata.body(pick_object).xpos.copy()
            if pick_pos[2]<0.1 or pick_pos[2]>2 :
                return True
        return False

    def random_placement(self, min_seperation=0.2):

        place_range = np.array([[0.4,0.7],[-0.33,0.33],[0.21,0.21]])
        place_range = np.array([[0.61,0.61],[0.0,0.0],[0.21,0.21]]) # static

        pick_range = np.array([[0.4,0.7],[-0.3,0.3],[0.24,0.24]])
        
        place_pos, pick_pos_multiple = None, None
        while not self.is_valid_position(place_pos,pick_pos_multiple,min_seperation):
            pick_pos_multiple = []
            place_pos = self.get_pos_from_range(place_range)
            for _ in self.pick_objects:
                pick_pos_multiple.append(self.get_pos_from_range(pick_range))
        
        self.mujocoAR.pause_updates()
        self.mujocoAR.reset_position()
        self.mjdata.qpos[self.actuator_ids] = self.q0
        mujoco.mj_step(self.mjmodel, self.mjdata)
        self.grasp = 0
        for i in range(len(pick_pos_multiple)):
            self.mjdata.joint(self.pick_joints[i]).qvel = np.zeros(6)
            self.mjdata.joint(self.pick_joints[i]).qpos = np.block([pick_pos_multiple[i],1,0,0,0])        
        set_body_pose(self.mjmodel,self.place_object,place_pos)
        mujoco.mj_step(self.mjmodel, self.mjdata)


        self.mujocoAR.resume_updates()

    def start_place_process(self):
        """
        Starts the Place process: pauses the simulation, performs YOLO detection,
        and allows the user to select a position for placing.
        """
        print("[INFO] Starting Place process.")
        #self.mujocoAR.pause_updates()  # Pause simulation updates

        # Perform YOLO detection
        print("[INFO] Running YOLO detection...")
        detection_results, final_result_image = self.run_yolo_and_get_final_results()

        # Add User Defined Position option
        detection_results.append({"name": "User Defined Position", "confidence": 1.0})

        # Display YOLO results
        print("[INFO] Detected objects:")
        for i, obj in enumerate(detection_results, 1):
            print(f" {i}. {obj['name']} | Confidence: {obj['confidence']:.2f}")

        if final_result_image is not None:
            Image.fromarray(final_result_image).show()

        # Prompt user to select a position
        selected_index = -1
        while selected_index not in range(1, len(detection_results) + 1):
            try:
                selected_index = int(input("Select a place position (enter the number): "))
                if selected_index not in range(1, len(detection_results) + 1):
                    print("[ERROR] Invalid selection. Try again.")
            except ValueError:
                print("[ERROR] Please enter a valid number.")

        selected_object = detection_results[selected_index - 1]

        # Handle User Defined Position
        if selected_object["name"] == "User Defined Position":
            print("[INFO] User selected 'User Defined Position'.")
            self.button_state = 2  # Change state to User Defined Position process
        else:
            print(f"[INFO] Selected object: {selected_object['name']}")
            self.target_pos = self.get_object_position_from_simulation(selected_object['name'])
            print(f"[INFO] Place position set to: {self.target_pos}")

        self.mujocoAR.resume_updates()
        # Reset state to prevent premature transitions
        if self.button_state != 2:
            self.button_state = 0

    def start_pick_process(self):
        """
        Starts the Pick process: pauses the simulation, performs YOLO detection,
        and allows the user to select an object for picking.
        """
        print("[INFO] Starting Pick process.")
        self.mujocoAR.pause_updates()  # Pause simulation updates

        # Perform YOLO detection
        print("[INFO] Running YOLO detection...")
        detection_results, final_result_image = self.run_yolo_and_get_final_results()

        # Add User Defined Object option
        detection_results.append({"name": "User Defined Object", "confidence": 1.0})

        # Display YOLO results
        print("[INFO] Detected objects:")
        for i, obj in enumerate(detection_results, 1):
            print(f" {i}. {obj['name']} | Confidence: {obj['confidence']:.2f}")

        if final_result_image is not None:
            Image.fromarray(final_result_image).show()

        # Prompt user to select an object
        selected_index = -1
        while selected_index not in range(1, len(detection_results) + 1):
            try:
                selected_index = int(input("Select an object to pick (enter the number): "))
                if selected_index not in range(1, len(detection_results) + 1):
                    print("[ERROR] Invalid selection. Try again.")
            except ValueError:
                print("[ERROR] Please enter a valid number.")

        selected_object = detection_results[selected_index - 1]

        # Handle User Defined Object
        if selected_object["name"] == "User Defined Object":
            print("[INFO] User selected 'User Defined Object'.")
            self.button_state = 5  # Change state to User Defined Object process
        else:
            print(f"[INFO] Selected object: {selected_object['name']}")
            self.selected_object = selected_object["name"]
            self.target_pos = self.get_object_position_from_simulation(selected_object["name"])
            print(f"[INFO] Pick position set to: {self.target_pos}")

        self.mujocoAR.resume_updates()
        # Reset state to prevent premature transitions
        if self.button_state != 5:
            self.button_state = 7

    def define_user_position(self):
        """
        Captures and saves the gripper's current position as the user-defined place position.
        """
        print("[INFO] Starting User Defined Position process.")
        self.mujocoAR.resume_updates() # Restart simulation updates

        print("[INFO] Press the button to capture the position.")
        while self.button_state != 3:  # Wait until the button state changes to 3
            time.sleep(0.1)  # Avoid busy waiting

        # When button is pressed, capture position
        print("[INFO] Button pressed. Capturing gripper position.")
        gripper_position = self.mjdata.site(self.eef_site_name).xpos.copy()
        print(f"[INFO] Captured position: {gripper_position}")
        self.target_pos = gripper_position  # Save position  
        print("[INFO] PUSH Toggle to pick mode  ")
        self.button_state = 0  # Reset state after capturing position
        time.sleep(2)

    def define_object(self):
        """
        Captures and saves the gripper's current position as the user-defined pick position,
        and assigns the closest object to pick.
        """
        print("[INFO] Starting User Defined Position process.")
        self.mujocoAR.resume_updates()  # Restart simulation updates

        print("[INFO] Press the button to capture the position.")
        while self.button_state != 6:  # Wait until the button state changes to 3
            time.sleep(0.1)  # Avoid busy waiting

        # When button is pressed, capture gripper position
        print("[INFO] Button pressed. Capturing gripper position.")
        gripper_position = self.mjdata.site(self.eef_site_name).xpos.copy()
        print(f"[INFO] Captured Gripper Position: {gripper_position}")

        # Find the closest object to the gripper position
        closest_object = self.find_closest_object(gripper_position)
        if closest_object:
            self.selected_object = closest_object["name"]
            self.target_pos = closest_object["position"]
            print(f"[INFO] Closest object: {self.selected_object} at {self.target_pos}")
        else:
            print("[INFO] No valid object found near the gripper position.")

        self.mujocoAR.resume_updates()  # Pause simulation again
        self.button_state = 7  # Reset state after capturing position


    def find_closest_object(self, gripper_position, object_name=None):
        """
        Finds the closest object to the provided gripper position.
        If object_name is specified, only consider that object.
        """
        closest_object = None
        min_distance = float("inf")

        # Iterate over pickable objects or a specific object
        objects_to_check = [object_name] if object_name else self.pick_objects

        for obj_name in objects_to_check:
            try:
                # Get the object's ID using its name
                obj_id = mujoco.mj_name2id(self.mjmodel, mujoco.mjtObj.mjOBJ_BODY, obj_name)
                # Get the object's position using xpos (2D array)
                obj_position = self.mjdata.xpos[obj_id]

                # Calculate the distance from the gripper position
                distance = np.linalg.norm(gripper_position - obj_position)
                if distance < min_distance:
                    closest_object = {"name": obj_name, "position": obj_position}
                    min_distance = distance

            except ValueError:
                print(f"[WARNING] Object '{obj_name}' not found in simulation.")
                continue

        if closest_object:
            return closest_object
        else:
            print("[INFO] No valid object found in the simulation.")
            return None




    def toggle_mode(self):
        """
        Toggles between Place and Pick modes.
        """
        self.toggle = not self.toggle
        if self.toggle:
            print("[INFO] Toggled to Place mode.")
        else:
            self.button_state = 0 
            print("[INFO] Toggled to Pick mode.")


    def button_pressed(self):
        """
        Handles button press events based on the toggle state (Place or Pick).
        """
        if self.toggle:  # Place 
            if self.button_state == 0:  # Ready state
                print("[BUTTON] Button pressed. Starting Pre-Place process.")
                self.button_state = 1  # Start Place process
            elif self.button_state == 2:  # During User Defined Position
                print("[BUTTON] Button pressed. Capturing User Defined Position.")
                self.button_state = 3  # Mark as position captured

        else:  # Pick 
            if self.button_state == 0:  # Ready state
                print("[BUTTON] Button pressed. Starting Pre-Pick process.")
                self.button_state = 4  # Start Pick process
            elif self.button_state == 5:  # During Pick task execution
                print("[BUTTON] Button pressed. Capturing user defined object")
                self.button_state = 6  # Mark as Pick completed
            elif self.button_state == 7 :
                print("Pregrasp start")
                self.button_state = 8 
                self.track_and_pregrasp_object()

            elif self.button_state == 9:
                print("Grasp start")
               



    def run_yolo_and_get_final_results(self):
        """
        Runs YOLO detection three times and returns only the final detection results and image.
        """
        print("[INFO] Running YOLO detection...")

        final_result_image = None
        detection_results = []

        for attempt in range(3):
            print(f"[INFO] YOLO detection attempt {attempt + 1}...")

            mujoco.mj_step(self.mjmodel, self.mjdata)

            camera_data = self.get_camera_data()
            gripper_rgb = camera_data.get("gripper_camera_rgb")

            if gripper_rgb is not None:
                gripper_rgb_bgr = cv2.cvtColor(gripper_rgb, cv2.COLOR_RGB2BGR)
                input_image = cv2.resize(gripper_rgb_bgr, (640, 640))
                results = self.yolo_model(input_image)

                if attempt == 2:
                    detection_results = []
                    for box in results[0].boxes:
                        cls = int(box.cls)
                        name = results[0].names[cls]
                        confidence = box.conf.numpy()[0]

                        if confidence >= 0.45:
                            detection_results.append({"name": name, "confidence": confidence})

                    final_result_image = cv2.cvtColor(results[0].plot(), cv2.COLOR_BGR2RGB)

        return detection_results, final_result_image










    def get_object_position_from_simulation(self, object_name):
        """
        Returns the position of the specified object from the simulation.
        Args:
            object_name: Name of the object to find.
        Returns:
            The position of the object as a numpy array, or None if not found.
        """
        try:
            return self.mjdata.body(object_name).xpos.copy()
        except KeyError:
            return None

    def track_and_pregrasp_object(self):
        print("Tracking and moving to pregrasp position for", self.selected_object)

        while True:
            obj_pos = self.mjdata.body(self.selected_object).xpos.copy()
            obj_rot = self.mjdata.body(self.selected_object).xmat.copy()

            pregrasp_pos = obj_pos.copy()
            pregrasp_pos[2] += 0.20

            self.mjdata.site("eef_target").xpos[:] = pregrasp_pos
            self.mjdata.site("eef_target").xmat[:] = obj_rot.reshape(-1)

            eef_pos = self.mjdata.site("eef_target").xpos.copy()

            distance = np.linalg.norm(pregrasp_pos - eef_pos)
            print(f"Pregrasp Position: {pregrasp_pos}")
            print(f"EEF Position: {eef_pos}")
            print(f"Distance to Pregrasp: {distance}")

            if distance < 0.05:
              
                print("Pregrasp position reached for", self.selected_object)
                time.sleep(2)
                self.button_state = 9

                print("Button state updated to 2: Ready to grasp", self.selected_object)
                self.track_and_grasp_object()
                break


            time.sleep(0.5)

    def track_and_grasp_object(self):
        print("Tracking and grasping", self.selected_object)

        while True:
            obj_pos = self.mjdata.body(self.selected_object).xpos.copy()
            obj_rot = self.mjdata.body(self.selected_object).xmat.copy()

            self.mjdata.site("eef_target").xpos[:] = obj_pos
            self.mjdata.site("eef_target").xmat[:] = obj_rot.reshape(-1)

            eef_pos = self.mjdata.site("eef_target").xpos.copy()

            distance = np.linalg.norm(obj_pos - eef_pos)
            print(f"Object Position: {obj_pos}")
            print(f"EEF Position: {eef_pos}")
            print(f"Distance to Object: {distance}")

            if distance < 0.05:
                print("Object reached by end effector.")
                time.sleep(2)

                self.grasp = 1
                self.mjdata.ctrl[self.grasp_actuator_id] = 255.0
                print("Gripper closed to grasp", self.selected_object)
                time.sleep(2)

                self.button_state = 10
                print("Button state updated to 3: Ready to place", self.selected_object)
                self.place_object_action()
                break

            time.sleep(0.5)

    def place_object_action(self):
        print("Placing", self.selected_object)

        while True:
            plate_pos = self.mjdata.body(self.place_object).xpos.copy()
            plate_rot = self.mjdata.body(self.place_object).xmat.copy()

            plate_pos[2] += 0.20

            self.mjdata.site("eef_target").xpos[:] = plate_pos
            self.mjdata.site("eef_target").xmat[:] = plate_rot.reshape(-1)

            eef_pos = self.mjdata.site("eef_target").xpos.copy()

            distance = np.linalg.norm(plate_pos - eef_pos)
            print(f"Plate Position: {plate_pos}")
            print(f"EEF Position: {eef_pos}")
            print(f"Distance to Plate: {distance}")

            if distance < 0.05:
                print(f"{self.selected_object} placed on the plate.")
                time.sleep(2)
                self.grasp = 0
                self.mjdata.ctrl[self.grasp_actuator_id] = 0.0
                print("Gripper opened to release", self.selected_object)
                time.sleep(2)
                self.button_state = 0
                print("Button state reset to 0: Ready for next task.")
                break

            time.sleep(0.5)




    def start(self):
        
        threading.Thread(target=self.mac_launch).start()
        if self.run_policy:
            self.run_model()

 
    def mac_launch(self):

            if not self.run_policy:
                self.mujocoAR.start()

            with mujoco.viewer.launch_passive(self.mjmodel, self.mjdata, show_left_ui=False, show_right_ui=False) as viewer:

                self.random_placement()
                self.reset_data()

                while viewer.is_running():

                    step_start = time.time()

                    if self.toggle:  # Place 
                        if self.button_state == 1:  # Place task triggered
                            self.start_place_process()
                        elif self.button_state == 3:  # User Defined Position process
                            self.define_user_position()

                    else:  # Pick 
                        if self.button_state == 4:  # Pick task triggered
                            self.start_pick_process()
                        elif self.button_state == 6:  # User Defined Object process
                            self.define_object()

                    self.record_data()

                    tau = self.controller.get_tau(self.mjmodel, self.mjdata, self.target_pos, self.target_rot)
                    self.mjdata.ctrl[self.actuator_ids] = tau[self.actuator_ids]
                    self.mjdata.ctrl[self.grasp_actuator_id] = (self.grasp) * 255.0
                    mujoco.mj_step(self.mjmodel, self.mjdata)
                    viewer.sync()

                    if self.was_placed() or self.fell():
                        self.record_data()
                        if not self.fell() and time.time() - self.placement_time > 2.0:
                            self.save_data()
                        if time.time() - self.placement_time > 2.0 or self.placement_time == -1:
                            self.random_placement()
                            self.reset_data()
                            self.placement_time = time.time()

                    if self.button_state == 8:  # Move to object pregrasp
                        self.saved_pick_position = self.get_object_position_from_simulation(self.selected_object)
                        obj_pos = self.saved_pick_position.copy()
                        obj_pregrasp_pos = obj_pos.copy()
                        obj_pregrasp_pos[2] += 0.15  # 15cm above the object
                        self.mjdata.site("eef_target").xpos[:] = obj_pregrasp_pos

                    elif self.button_state == 9: #Move to object
                        obj_pos = self.saved_pick_position.copy()
                        self.mjdata.site("eef_target").xpos[:] = obj_pos

                    elif self.button_state == 10: #Move to place position 
                        plate_pos = self.mjdata.body("plate").xpos.copy()
                        plate_pos[2] += 0.15  # 15cm above the plate
                        self.mjdata.site("eef_target").xpos[:] = plate_pos

                    self.target_pos = self.mjdata.site("eef_target").xpos.copy()

                    time_until_next_step = self.dt - (time.time() - step_start)
                    if time_until_next_step > 0:
                        time.sleep(time_until_next_step)
                        

    def record_data(self):

        if not self.record or self.camera_data is None:
            return
     
        if self.camera_data is not None and self.last_recording_time != -1 and time.time()-self.last_recording_time < (1/self.recording_frequency):
            return
        
        if self.record_start_time == None:
            self.record_start_time = time.time()
            time_diff = 0.0
        else:
            time_diff = time.time() - self.record_start_time
        pose = np.identity(4)
        pose[0:3,3] = self.mjdata.site(self.site_id).xpos.copy()
        pose[0:3,0:3] = self.mjdata.site(self.site_id).xmat.copy().reshape((3,3))

        q = self.mjdata.qpos[self.dof_ids].copy()
        dq = self.mjdata.qvel[self.dof_ids].copy()
        
        camera1_rgb = self.camera_data[self.cameras[0]+"_rgb"]
        camera1_depth = self.camera_data[self.cameras[0]+"_depth"]
        camera2_rgb = self.camera_data[self.cameras[1]+"_rgb"]
        camera2_depth = self.camera_data[self.cameras[1]+"_depth"]
        camera3_rgb = self.camera_data[self.cameras[2]+"_rgb"]
        camera3_depth = self.camera_data[self.cameras[2]+"_depth"] 

        self.camera1_rgbs.append(camera1_rgb)
        self.camera1_depths.append(camera1_depth)
        self.camera2_rgbs.append(camera2_rgb)
        self.camera2_depths.append(camera2_depth)
        self.camera3_rgbs.append(camera3_rgb)
        self.camera3_depths.append(camera3_depth)

        self.poses.append(pose)
        self.grasps.append(self.grasp)
        self.times.append(time_diff)
        self.q.append(q)
        self.dq.append(dq)

        self.last_recording_time = time.time()

    def save_data(self):

        if not self.record:
            return

        new_file_name = "Data/" + str(get_latest_number("Data")+1)+".npz"
        camera1_rgbs = np.array(self.camera1_rgbs)
        camera1_depths = np.array(self.camera1_depths)
        camera2_rgbs = np.array(self.camera2_rgbs)
        camera2_depths = np.array(self.camera2_depths)
        camera3_rgbs = np.array(self.camera3_rgbs)
        camera3_depths = np.array(self.camera3_depths)

        poses = np.array(self.poses)
        times = np.array(self.times)
        grasps = np.array(self.grasps)
        q = np.array(self.q)
        dq = np.array(self.dq)

        np.savez(new_file_name, camera1_rgbs=camera1_rgbs, camera1_depths=camera1_depths, camera2_rgbs=camera2_rgbs, camera2_depths=camera2_depths, camera3_rgbs=camera3_rgbs, camera3_depths=camera3_depths, poses=poses, grasps=grasps, times=times, q=q, dq=q)

    def reset_data(self):
        self.camera1_rgbs = []
        self.camera1_depths = []
        self.camera2_rgbs = []
        self.camera2_depths = []
        self.camera3_rgbs = []
        self.camera3_depths = []
        self.poses = []
        self.times = []
        self.grasps = []
        self.q = []
        self.dq = []
        self.record_start_time = time.time()
    
    def run_poses_from_npz(self, npz_file_path):

        data = np.load(npz_file_path)
        poses = data['poses']
        times = data['times']
        grasps = data['grasps']

        while True:
          
            start_time = time.time()
            data_time = times[0]

            i = 1

            while i<len(poses)-1:
                
                # Set the pose
                if (time.time()-start_time) - (times[i]-data_time) >= 0:
                    set_site_pose(self.mjmodel,"eef_target",poses[i][0:3,3])
                    self.grasp = grasps[i]
                    i += 1


    def run_model(self):

        checkpoint_path = "Model/checkpoint_epoch.pth.tar"
        policy = load_model(checkpoint_path,"cpu")
        last_time = None
        
        while True:

            sim.camera_data = sim.get_camera_data()
        
            if (last_time is None or (time.time()-last_time) >= 1/5):
                
            
                last_data = self.prev_datas[-1]
                last_time = self.prev_times[-1]

                prev_data = self.prev_datas[0]
                j = 0
                for i, prev_time in enumerate(self.prev_times):
                    if last_time - prev_time >= 0.2:
                        prev_data = self.prev_datas[i]
                        j = i
                    else:
                        break
                
                print(self.prev_times[j]-self.prev_times[-1])
                            
                rgb1 = last_data["top_camera_rgb"]
                depth1 = last_data["top_camera_depth"]
                rgb2 = last_data["front_camera_rgb"]
                depth2 = last_data["front_camera_depth"]

                prev_rgb1 = prev_data["top_camera_rgb"]
                prev_depth1 = prev_data["top_camera_depth"]
                prev_rgb2 = prev_data["front_camera_rgb"]
                prev_depth2 = prev_data["front_camera_depth"]

                delta_pos = predict_action(policy, rgb1, depth1, rgb2, depth2, prev_rgb1, prev_depth1, prev_rgb2, prev_depth2, torch.tensor([sim.mjdata.site(sim.site_id).xpos.copy()]).float(), torch.tensor([[sim.grasp]]), "cpu")[0]
                
                new_pos = sim.mjdata.site("eef_target").xpos.copy()+delta_pos[:3]
                new_pos[2] =  max(new_pos[2],0.21)

                set_site_pose(sim.mjmodel,"eef_target",new_pos,None)

                sim.grasp = round(delta_pos[-1])
                last_time = time.time()



if __name__ == "__main__":
    sim = ImitationSimulation()
    sim.start()

    print("[INFO] Simulation started. Gripper Camera is live.")

    while True:
        # Fetch Gripper Camera data continuously
        camera_data = sim.get_camera_data()
        gripper_rgb = camera_data.get("gripper_camera_rgb")

        if gripper_rgb is not None:
            # Display Gripper Camera in real-time
            gripper_rgb = cv2.cvtColor(gripper_rgb, cv2.COLOR_RGB2BGR)
            cv2.imshow("Gripper Camera", gripper_rgb)

        # Check for button press to trigger YOLO detection
        if sim.button_state == 1:  # Button press triggers YOLO detection and Pick-and-Place
            sim.button_pressed()

        # Exit on ESC key press
        if cv2.waitKey(1) & 0xFF == 27:
            break



