# import the necessary packages
from __future__ import print_function
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import argparse
import imutils
import cv2

import minimalmodbus

from time import time, sleep
from math import tan
import numpy as np

from circle_class import Circle

#create font
font = cv2.FONT_HERSHEY_SIMPLEX

#--------------------------------------------
#			USER PARAMETERS
#--------------------------------------------

#image resolution (unit: px)
res_width = 368
res_length = 480

#distance between camera and conveyor (unit: mm)
cam_height = 250

#distance between robot origin and image corner (unit: mm)
robot_offset_x = 600.0
robot_offset_y = 50.0

#robot range from origin (unit: mm)
robot_range_x = 300
robot_range_y = 475

#mm/rot X
mmrotx = 151

#mm/rot Y
mmroty = 113

#servo pulses/rot
prot = 10000

#robot speed (mm/s)
rs = 500

#servo accel/decel time (ms)
adt = 100

#part follow time (unit: ms)
ft = 300

#part speed (unit: mm/s)
ps = 150

#--------------------------------------------
#		END OF USER PARAMETERS
#--------------------------------------------

#calculate ratio between pixels and mm based on camera positioning and settings
px_mm_ratio = (2 * cam_height * tan(0.543)) / res_length

def calculate_pos(img_x, img_y, robot_offset_x, robot_offset_y):

	pos_x = robot_offset_x + img_x * px_mm_ratio
	pos_y = robot_offset_y + img_y * px_mm_ratio

	return (pos_x, pos_y)

#initialize PLC and change communication settings
plc = minimalmodbus.Instrument('/dev/ttyUSB0', 1)
plc.serial.baudrate = 9600
plc.serial.bytesize = 8
plc.serial.parity = minimalmodbus.serial.PARITY_EVEN
plc.serial.stopbits = 1
plc.serial.timeout = 0.1
plc.mode = minimalmodbus.MODE_RTU

#wait for robot to be ready before writing settings
print('waiting for robot ready...')
while True:
	try:
		ready_bit = plc.read_bit(0x0900)
	
		if ready_bit == 1:
			print('robot ready')
			break
		else:
			print('robot not ready')
			sleep(0.1)
	except:
		print('no communication with PLC')
		sleep(0.1)

print('writing settings')
#----------------------------------------
#	START WRITE ROBOT SETTINGS TO PLC
# ---------------------------------------

#D150, D151 - part speed (mm/s)
plc.write_register(0x1096, ps)

#D152, D153 - robot speed (mm/s)
plc.write_register(0x1098, rs)

#D154, D155 - accel/decel time (ms)
plc.write_register(0x109A, adt)

#D158, D159 - mm/rot X
plc.write_register(0x109E, mmrotx)

#D160, D161 - mm/rot Y
plc.write_register(0x10A0, mmroty)

#D162, D163 - pulses/rot
plc.write_register(0x10A2, prot)

#D164, D165 - robot range X
plc.write_register(0x10A4, robot_range_x)

#D166, D167 - robot range Y
plc.write_register(0x10A6, robot_range_y)

#D168, D169 - part follow time (ms)
plc.write_register(0x10A8, ft)

#M7 - tells plc that settings have been written
plc.write_bit(0x0807, 1)

#----------------------------------------
#	 END WRITE ROBOT SETTINGS TO PLC
# ---------------------------------------

#start video stream and begin fps recording
vs = WebcamVideoStream(src=0).start()
fps = FPS().start()

#prepare for circle detection
circle_list = []
 
# loop over frames with threaded stream
key = 255
while key == 255:

	#capture and resize frame, then convert to grayscale
	frame = vs.read()
	frame = imutils.resize(frame, width=res_length)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

	#detect circles with openCV algorithm
	circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 100)

	#add circles to list and draw them on screen
	detected_circle_list = []
	if circles is not None:
		# convert the (x, y) coordinates and radius of the circles to integers
		circles = np.round(circles[0, :]).astype("int")
	
		# loop over the (x, y) coordinates and radius of the circles
		for (x, y, r) in circles:

			coordinates = calculate_pos(
				img_x=x,
				img_y=y,
				robot_offset_x=robot_offset_x,
				robot_offset_y=robot_offset_y
			)

			detected_circle_list.append(Circle(coordinates[0], coordinates[1], ps))

			# draw the circle in the output image, then draw a rectangle
			# corresponding to the center of the circle
			cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
			cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
			cv2.putText(
				img=frame,
				text='(%s, %s)'%(x, y),
				org=(x, y),
				fontFace=font,
				fontScale=0.7,
				color=(0, 0, 255),
				thickness=2,
				lineType=cv2.LINE_AA
			)

	

	#update circle positions
	for circle in circle_list:

		circle.update_pos()

		if circle.current_x < 0:
			circle_list.remove(circle)
	
	#signal if parts are incoming
	if len(circle_list) == 0:
		plc.write_bit(0x0806, 0)
	else:
		plc.write_bit(0x0806, 1)

	#check if robot is ready
	robot_ready_bit = plc.read_bit(0x0900)

	#if ready, send closest circle coordinates
	if robot_ready_bit == 1 and len(circle_list) > 0:
		plc.write_register(0x1100, circle_list[-1].current_x)
		plc.write_register(0x1102, circle_list[-1].current_y)

	#check if robot left, and delete circle if so
	robot_left_bit = plc.read_bit(0x0901)
	if robot_left_bit:
		circle_list.remove(circle)
		plc.write_bit(0x0901, 0)
			


	#check for new circles by comparing expected position data
	for detected_circle in detected_circle_list:

		new_circle = True
		for circle in circle_list:

			if circle.is_equal(detected_circle):
				new_circle = False
				break

		if new_circle:
			circle_list.append(detected_circle)

	#print circle coordinates to terminal
	circle_list_string = 'robot ready: %s\t'%robot_ready_bit
	for circle in circle_list:
		circle_list_string += str(circle) + ', '
	print(circle_list_string)

	#display frame
	cv2.imshow('Frame', frame)
	key = cv2.waitKey(1) & 0xFF

	#update fps object
	fps.update()

# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
 
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
