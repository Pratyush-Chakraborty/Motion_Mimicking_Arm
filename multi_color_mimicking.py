import numpy as np
import cv2
import pybullet as p
import time
import pybullet_data
import math

# Capturing video through webcam
webcam = cv2.VideoCapture(0)

red_x_coord = []
green_x_coord = []
blue_x_coord = []
red_y_coord = []
green_y_coord = []
blue_y_coord = []
vel_matrix= []
alpha = 0
kp = 1000
ki = 0
kd = 200

# Start a while loop
while(1):
	
	# Reading the video from the
	# webcam in image frames
	_, imageFrame = webcam.read()

	# Convert the imageFrame in
	# BGR(RGB color space) to
	# HSV(hue-saturation-value)
	# color space
	hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

	# Set range for red color and
	# define mask
	red_lower = np.array([150, 87, 111], np.uint8)
	red_upper = np.array([180, 255, 255], np.uint8)
	red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

	# Set range for green color and
	# define mask
	green_lower = np.array([40, 41, 82], np.uint8)
	green_upper = np.array([45, 255, 255], np.uint8)
	green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

	# Set range for blue color and
	# define mask
	blue_lower = np.array([94, 80, 2], np.uint8)
	blue_upper = np.array([120, 255, 255], np.uint8)
	blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
	
	# Morphological Transform, Dilation
	# for each color and bitwise_and operator
	# between imageFrame and mask determines
	# to detect only that particular color
	kernel = np.ones((5, 5), "uint8")
	
	# For red color
	red_mask = cv2.dilate(red_mask, kernel)
	res_red = cv2.bitwise_and(imageFrame, imageFrame,
							mask = red_mask)
	
	# For green color
	green_mask = cv2.dilate(green_mask, kernel)
	res_green = cv2.bitwise_and(imageFrame, imageFrame,
								mask = green_mask)
	
	# For blue color
	blue_mask = cv2.dilate(blue_mask, kernel)
	res_blue = cv2.bitwise_and(imageFrame, imageFrame,
							mask = blue_mask)

	# Creating contour to track red color
	contours, hierarchy = cv2.findContours(red_mask,
										cv2.RETR_TREE,
										cv2.CHAIN_APPROX_SIMPLE)
	
	for pic, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		if(area > 300):
			x, y, w, h = cv2.boundingRect(contour)
			imageFrame = cv2.rectangle(imageFrame, (x, y),
									(x + w, y + h),
									(0, 0, 255), 2)
			
            
			
			cv2.putText(imageFrame, "Red Colour", (x, y),
						cv2.FONT_HERSHEY_SIMPLEX, 1.0,
						(0, 0, 255))	
			cv2.circle(imageFrame,((x+(w//2)),(y+(h//2))), 5, (0,0,255), -1)
			red_x_coord.append(x+(w//2))
			red_y_coord.append(y+(h//2))

	# Creating contour to track green color
	contours, hierarchy = cv2.findContours(green_mask,
										cv2.RETR_TREE,
										cv2.CHAIN_APPROX_SIMPLE)
	
	for pic, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		if(area > 300):
			x, y, w, h = cv2.boundingRect(contour)
			imageFrame = cv2.rectangle(imageFrame, (x, y),
									(x + w, y + h),
									(0, 255, 0), 2)
			
			cv2.putText(imageFrame, "Green Colour", (x, y),
						cv2.FONT_HERSHEY_SIMPLEX,
						1.0, (0, 255, 0))
			cv2.circle(imageFrame,((x+(w//2)),(y+(h//2))), 5, (0,255,0), -1)
			green_x_coord.append(x+(w//2))
			green_y_coord.append(y+(h//2))

	# Creating contour to track blue color
	contours, hierarchy = cv2.findContours(blue_mask,
										cv2.RETR_TREE,
										cv2.CHAIN_APPROX_SIMPLE)
	for pic, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		if(area > 300):
			x, y, w, h = cv2.boundingRect(contour)
			imageFrame = cv2.rectangle(imageFrame, (x, y),
									(x + w, y + h),
									(255, 0, 0), 2)
			
			cv2.putText(imageFrame, "Blue Colour", (x, y),
						cv2.FONT_HERSHEY_SIMPLEX,
						1.0, (255, 0, 0))
			cv2.circle(imageFrame,((x+(w//2)),(y+(h//2))), 5, (255,0,0), -1)
			blue_x_coord.append(x+(w//2))
			blue_y_coord.append(y+(h//2))
    		
	# Program Termination
	cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
	if cv2.waitKey(10) & 0xFF == ord('q'):
		webcam.release()
		cv2.destroyAllWindows()
		break

print(red_x_coord)
print(red_y_coord)
print(len(red_x_coord))
print(len(red_y_coord))
for l in range(1,len(red_x_coord)):
	vel_matrix.append(red_x_coord[l] - red_x_coord[l-1])

#connecting to the physics server and setting the additional files from the pybullet data
physicsclient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# setting the gravity for realistic force effects
p.setGravity(0, 0, -9.81)

# loading the URDF
planeID = p.loadURDF("plane.urdf")

# initializing the position and orientation
startpose = [0, 0, 0]
startorientation = p.getQuaternionFromEuler([0,0,0])

# loading the urdf files form desktop
urdfID = p.loadURDF("E:/VNIT/IvLabs/Reference materials/3R/urdf/planar_3dof.urdf", startpose, startorientation, useFixedBase=True)

# getting the information related to the joint of URDF 
joints = p.getNumJoints(urdfID)

joint_indices = []

for i in range(3):
    p.setJointMotorControl2(urdfID, i, p.VELOCITY_CONTROL, force = 0)
    joint_indices.append(i)

print(joint_indices)

joint_states = p.getJointStates(urdfID, joint_indices)
	
for b in range(len(red_x_coord)):
	ep = np.array([[p.getLinkState(urdfID, 2, computeForwardKinematics=True)[4][0]],
                   [p.getLinkState(urdfID, 2, computeForwardKinematics=True)[4][1]],
                   [p.getLinkState(urdfID, 2, computeForwardKinematics=True)[4][2]]])
	eint = np.array([0, 0, 0])
	edot = np.array([0, 0, 0])
	e = np.array([0, 0, 0])
	theta = np.array([0,0,0])
	vel = np.array([0,0,0])
	for i in range(3):
		s = p.getJointState(urdfID, i)
		theta[i] = s[0]
		vel[i] = s[1]
	theta_desired = np.array(p.calculateInverseKinematics(urdfID, 2, [red_x_coord[b], red_y_coord[b], 0]))
	vel_desired = np.array([0.2]*3)
	acc_desired = np.array([0]*3)
	e = theta_desired - theta
	edot = vel_desired - vel
	eint = 0.0001
	mass_matrix = np.array(p.calculateMassMatrix(urdfID,list(theta_desired)))
	acc = acc_desired + kp*e + kd*edot + kp*eint
	gravity_matrix = np.array(p.calculateInverseDynamics(urdfID,list(theta_desired),[0,0,0],[0,0,0]),dtype=np.float64)
	coriolis_matrix = np.array(p.calculateInverseDynamics(urdfID, list(theta_desired), list(vel_desired), [0, 0, 0]),dtype=np.float64)-gravity_matrix
	sum = gravity_matrix + coriolis_matrix
	torque = np.dot(acc, mass_matrix) + sum
	print(torque)

	for k in range(len(torque)):
		p.setJointMotorControl2(urdfID, k, p.TORQUE_CONTROL, force = torque[k])
	ep = np.array([[p.getLinkState(urdfID, 2, computeForwardKinematics=True)[4][0]],
                   [p.getLinkState(urdfID, 2, computeForwardKinematics=True)[4][1]],
                   [p.getLinkState(urdfID, 2, computeForwardKinematics=True)[4][2]]])
	print(ep)
	p.stepSimulation()
	time.sleep(1/240)