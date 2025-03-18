# mujoco-pick-and-place

### introduction
In this study, we propose an **interactive robotic control framework** that integrates iOS devices with the MuJoCo simulator while utilizing YOLO-based object recognition. Conventional vision-based YOLO object detection methods often struggle with accurate detection, even within simulation environments, presenting a significant challenge. To address this issue, we developed a User-Interactive System that enhances the success rate of Pick-and-Place tasks by incorporating user intervention.
Users can transmit hand motions via a mobile device, allowing real-time adjustment of the robot's end effector position, enabling more intuitive control. This approach provides a more natural way to operate the robot’s end-effector. By using an easily accessible device like a smartphone, it also allows for remote operation of the robot. Furthermore, the system combines vision-based detection with user intervention, creating a more intuitive and successful Pick-and-Place system compared to vision-only methods. Finally, experimental evaluations demonstrated that the proposed system achieves a higher success rate than conventional Vision-Only approaches. These results verify that our approach improves task performance by integrating vision-based detection and user intervention, demonstrating its potential for application in various unstructured environments.

### Key Features
**iOS ARKit Integration**: Control the robot using the MuJoCo Controller application on an iOS device. This feature is based on **github - omarrayyan/MujocoAR**

**YOLO-Based Object Recognition**: Automatically detects objects and performs Pick-and-Place tasks.

**User Interactive System**: Allows manual adjustments when object detection fails.

**Automated Pick-and-Place Execution**: Moves objects to predefined locations automatically.
