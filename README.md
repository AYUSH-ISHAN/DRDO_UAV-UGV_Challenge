# DRDO UAV-UGV Challenge

# Introduction : 
<p align="center"><img src="https://github.com/AYUSH-ISHAN/DRDO_UAV-UGV_Challenge/blob/main/_media_/drdo.png" height="400" width="450"/></p>
This is our solution of our cotingent for the <a href="https://interiit-tech.org/events/DRDO's%20UAV-Guided%20UGV%20Navigation%20Challenge">DRDO UAV-UGV Challenge</a> of mid prep event of 10th Inter IIT 2022.

# Problem Statement : 
The problem statement is to map an area via UAV aerial imagery and aid a UGV in navigating a complex static environment. This requires the team to map the mountain road in the worlds using a UAV and guide a UGV through the area, navigating across various turns, altitudes, and depth of terrain.
- The major components include vehicle consideration (sensors), computer vision development, path planning, vehicle control, integration of multiple systems, and finally the validation and testing of the algorithms in standardized environments.
- The UAV is tasked to map the area following which the original image overlay of the terrain would be replaced with a plain overlay. The UGV then has to traverse the mountain road with aid from the UAV.
- The UGV should traverse along the center of the road.
- The UAV should not fly higher than 20m. The UAV is allowed the use of a downward- facing RGBD-camera, IMU sensor, and GPS.
- No sensors are allowed on the UGV.
- The speed of the UGV should be limited to 35 km/hr.
- The integration of all the algorithms is to be ensured for the program to efficiently gather and process data to map and give the path planning information.
- The task is considered to be completed when the UGV successfully navigates across the terrain to the end of the road with aid from the UAV.
# Approach Used : 

For our approach to solve to solve different segments. Please refer to this <a href="https://github.com/AYUSH-ISHAN/DRDO_UAV-UGV_Challenge/blob/main/MP_DR_Final_T14.pdf">PDF</a><br>
If you are found of PPT format. Please visit this <a href="https://docs.google.com/presentation/d/1wgCw-pj4WYZ1QgIoMBwp1pqgJasY5RpXskZN9Sr_edU/edit#slide=id.g11ef2c89eb0_2_12">LINK</a>
