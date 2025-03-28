# mujoco-pick-and-place
![image](https://github.com/user-attachments/assets/2b444ecb-6f6a-4975-8f30-26a33785d1e6)


## **introduction**
In this study, we propose an **interactive robotic control framework** that integrates iOS devices with the MuJoCo simulator while utilizing YOLO-based object recognition. Conventional vision-based YOLO object detection methods often struggle with accurate detection, even within simulation environments, presenting a significant challenge. To address this issue, we developed a User-Interactive System that enhances the success rate of Pick-and-Place tasks by incorporating user intervention.

![image](https://github.com/user-attachments/assets/97ac48af-a86e-4e51-89a9-2b7a3fd6ee2f)

Users can transmit hand motions via a mobile device, allowing real-time adjustment of the robot's end effector position, enabling more intuitive control. This approach provides a more natural way to operate the robot’s end-effector. By using an easily accessible device like a smartphone, it also allows for remote operation of the robot. Furthermore, the system combines vision-based detection with user intervention, creating a more intuitive and successful Pick-and-Place system compared to vision-only methods. Finally, experimental evaluations demonstrated that the proposed system achieves a higher success rate than conventional Vision-Only approaches. These results verify that our approach improves task performance by integrating vision-based detection and user intervention, demonstrating its potential for application in various unstructured environments.

## **Key Features**
**iOS ARKit Integration**: Control the robot using the MuJoCo Controller application on an iOS device. This feature is based on **github - omarrayyan/MujocoAR**
![image](https://github.com/user-attachments/assets/b64a84e9-c592-4717-83d1-65b919e01293)


**YOLO-Based Object Recognition**: Automatically detects objects and performs Pick-and-Place tasks.
![image](https://github.com/user-attachments/assets/fc9ce791-6dc6-45bb-90c4-2afcc12a8184)
![image](https://github.com/user-attachments/assets/38c73d54-e621-4354-a4dd-9a03afaf622a)


**User Interactive System**: Allows manual adjustments when object detection fails.

**Automated Pick-and-Place Execution**: Moves objects to predefined locations automatically.

## **execution module**
![image](https://github.com/user-attachments/assets/22141ce5-3212-49f4-9b1c-29812b70a873)


