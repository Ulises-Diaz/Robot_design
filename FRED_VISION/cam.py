
import pyrealsense2 as rs
import numpy as np
import cv2
import time

#to connect camera
pipe = rs.pipeline() 
cfg = rs.config()

cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)

pipe.start(cfg)

while True: 
    frame = pipe.wait_for_frames() 
    color_frame = frame.get_color_frame () 

    #we need numpy to read
    color_image = np.asanyarray(color_frame.get_data())

    roi = cv2.selectROI("select the area", color_image)
    
    time.sleep(100)

    cropped_image = color_image[int(roi[1]):int(roi[1]+roi[3]), 
                      int(roi[0]):int(roi[0]+roi[2])]


    cv2.imshow("cropped img", cropped_image)

    cv2.imshow('rgb', color_image)

    if cv2.waitKey(1) == ord('q'):
        break

pipe.stop()