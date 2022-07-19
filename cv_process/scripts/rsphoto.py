## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

#显示RGB colormap和保存图像

import pyrealsense2 as rs
import numpy as np
import cv2

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)
i=0
def callback(event,x,y,flags,param):
 if event == cv2.EVENT_LBUTTONDOWN:
  cv2.imwrite('/home/cyh/jaka_robot/jaka_robot/src/cv_process/scripts/teris.png', color_image)
  print('success')
 if event == cv2.EVENT_RBUTTONDOWN:
  exit()

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
      
        color_frame = frames.get_color_frame()
        align_to_color=rs.align(rs.stream.color)
      

     
        color_image = np.asanyarray(color_frame.get_data())

   
        color_colormap_dim = color_image.shape

        
        
          

        # Show images
        images=color_image
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback("RealSense", callback)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)

finally:

    # Stop streaming
    pipeline.stop()


