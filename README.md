# mujoco-pick-and-place

## **introduction**
In this study, we propose an **interactive robotic control framework** that integrates iOS devices with the MuJoCo simulator while utilizing YOLO-based object recognition. Conventional vision-based YOLO object detection methods often struggle with accurate detection, even within simulation environments, presenting a significant challenge. To address this issue, we developed a User-Interactive System that enhances the success rate of Pick-and-Place tasks by incorporating user intervention.

![image](https://github.com/user-attachments/assets/97ac48af-a86e-4e51-89a9-2b7a3fd6ee2f)

Users can transmit hand motions via a mobile device, allowing real-time adjustment of the robot's end effector position, enabling more intuitive control. This approach provides a more natural way to operate the robot’s end-effector. By using an easily accessible device like a smartphone, it also allows for remote operation of the robot. Furthermore, the system combines vision-based detection with user intervention, creating a more intuitive and successful Pick-and-Place system compared to vision-only methods. Finally, experimental evaluations demonstrated that the proposed system achieves a higher success rate than conventional Vision-Only approaches. These results verify that our approach improves task performance by integrating vision-based detection and user intervention, demonstrating its potential for application in various unstructured environments.


## **Key Features**
**iOS ARKit Integration**: Control the robot using the MuJoCo Controller application on an iOS device. This feature is based on **github - omarrayyan/MujocoAR**

**YOLO-Based Object Recognition**: Automatically detects objects and performs Pick-and-Place tasks.

**User Interactive System**: Allows manual adjustments when object detection fails.

**Automated Pick-and-Place Execution**: Moves objects to predefined locations automatically.



##  Installation

### 1. Install the iOS App  
Download the **MuJoCo Controller** app from the App Store.  


### 2. Install Python Package  
Run the following command in your terminal to install the required Python package:

```bash
pip install mujoco_ar
```

### 3. Clone the GitHub Repository
Clone this repository and move into the project directory:

```bash
git clone https://github.com/garammi/mujoco-pick-and-place.git
cd mujoco-pick-and-place
```
### 4. Run the Simulation
Start the simulation by executing main.py:

```bash
python main.py
```
### 5. Check IP Address
Once main.py is running, check the terminal (CMD) window for the displayed IP address and Port number.

### 6. Connect via iOS App
![image](https://github.com/user-attachments/assets/d1a06a4b-d82b-4d75-b197-41bf6f7f9143)

1. Open the MuJoCo Controller app on your iOS device.  
2. Enter the IP address and port number shown in the terminal.  
3. If the connection is successful, you’ll see that the iOS device is now linked with the simulation in real-time.


## Pick-and-Place System

Once the iOS device connection has been verified and synchronization between the simulation and the device is successfully established, the pick-and-place process can begin.

The process consists of the following three stages, as shown in **Figure 2**:
1. **Pre-Place**: Decide the final placement position of the object.
2. **Pre-Pick**: Select the object to be picked.
3. **Pick and Place**: Move the selected object to the designated location.

> The user interaction system for the Pre-Place and Pre-Pick stages will be discussed in detail in Section D.

![image](https://github.com/user-attachments/assets/618267c6-ee3e-4127-9e7a-f00d596993c2)


To implement the algorithm, the functionality of the **MuJoCo Controller** iOS app is utilized to send ARKit data from the device.  
- The **Toggle** button is used to switch between the Pre-Place and Pre-Pick stages.  
- The **Button** is used to confirm the Pick and Place targets.  
Each time the button is pressed, the `Button_state` is updated to ensure the flow of each stage progresses smoothly.

![image](https://github.com/user-attachments/assets/3f81d1c4-d12a-4f7e-b792-815b23daea8f)


### Step-by-step Execution:

1. When the device and simulation are connected, pressing the button once will **start the Pre-Place stage**, and the simulation will pause.
2. While paused, the camera attached to the robot's end effector captures an image.
3. This image is analyzed by the **Vision-Only Object Detection Module**.
4. Detected candidate placement positions are shown in a list along with their **confidence scores**, and the user selects one to set as the **Place** location.

Once the Place position is selected, the user presses the Toggle to **switch to Pre-Pick mode** and resumes synchronization.

5. Pressing the button again pauses the simulation.
6. The current gripper camera view is sent to the Vision-Only module.
7. Detected objects are listed with their confidence scores (as shown in **Figure 4**), and the user selects an object to **Pick**.

![image](https://github.com/user-attachments/assets/7f2a6706-b87e-406c-85b1-ee030f7ed670)


With both the Pick object and Place location selected, the **Pick and Place execution** begins.

- During normal synchronization, the iOS device position determines the target for the robot’s end effector.
- However, in the **execution phase**, the robot uses the user-specified Pick and Place positions.

**Execution sequence:**
1. Move the end effector to a position **15cm above** the Pick target (along the Z-axis).
2. Wait briefly for stability, then lower to the **exact Pick position**.
3. Once the end effector reaches the target, **close the gripper** to grasp the object.
4. Move to a position **15cm above** the Place location.
5. Finally, **open the gripper** to complete the Pick and Place process.

