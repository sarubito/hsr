#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
#from darknet_ros_msgs.msg import BoundingBoxes

import rospy
#import hsrb_interface
#from hsrb_interface import geometry
import tf2_ros
import numpy as np
import geometry_msgs.msg
from tf.transformations import *

cvbridge=CvBridge()
tf_br = tf2_ros.TransformBroadcaster()
camera_info=None
invK = None
gcnt=0

MIN_OBJ_AREA = 60

def draw_rect(contours,target_img):
    size_min = 7
    size_max = 150 
    aspect_ratio_min = 1/9. 
    aspect_ratio_max = 9
    rect_list=[]
    target_img = cv2.cvtColor(target_img, cv2.COLOR_GRAY2RGB)
    for i in range(len(contours)):
        posx, posy, width, height = cv2.boundingRect(contours[i])
        fwidth = float(width)
#        if aspect_ratio_min < fwidth/height and fwidth / height < aspect_ratio_max and width > size_min and height > size_min and width < size_max and height < size_max and posx<540 and posx>100 and posy>300: # --
#        if aspect_ratio_min < fwidth/height and fwidth / height < aspect_ratio_max and width > size_min and height > size_min and width < size_max and height < size_max: # --
        if width > size_min and height > size_min and width < size_max and height < size_max and posx<580 and posx>10 and posy>140: 
            area = cv2.contourArea(contours[i])
#            if area > 100 and area <10000:
            if area > MIN_OBJ_AREA:
                target_img = cv2.fillConvexPoly(target_img, contours[i], (255,0,0))
                ellipse = cv2.fitEllipse(contours[i])
######                ma_im = cv2.ellipse(target_img,ellipse,255*256,3)
######                el_box = cv2.boxPoints(ellipse)
######                el_box = np.int0(el_box)
######                ma_rect = cv2.minAreaRect(contours[i])
######                ma_box = cv2.boxPoints(ma_rect)
######                ma_box = np.int0(ma_box)
#                ma_im = cv2.drawContours(ma_im,[ma_box],0,(255,0,0),5)
###                print(ma_rect[0], ma_rect[1], ma_rect[2], ma_box[0][0], ma_box[0][1], ma_box[1][0], ma_box[1][1], ma_box[2][0], ma_box[2][1], ma_box[3][0], ma_box[3][1])
        #if True:
            #cv2.rectangle(target_img, (posx, posy), (posx + width, posy + height), 255*256, 2)
######                rect_list.append((posx + width/2,posy+height/2,ellipse[2],el_box[0][0],el_box[0][1],el_box[1][0],el_box[1][1],el_box[2][0],el_box[2][1]))

                rect_list.append((posx + width/2,posy+height/2,ellipse[2]))

    np.savetxt('test_npsave.txt',rect_list)
#    cv2.imshow('minAreaRect', ma_im*300)
    return rect_list


def hsr_yolo_callback(msg):
#print('hello')
#    yolo_bound_box = msg.bounding_boxes
#    print(msg.bounding_boxes)
    for i, contour in enumerate(yolo_bound_box):
        print(yolo_bound_box[i].Class)

def depth_callback(newimage):
    global invK
    global camera_info
    global global_i
    global gcnt
    #print("callback1")
    gcnt = gcnt + 1
    global_i = newimage
    img = np.array(cvbridge.imgmsg_to_cv2(global_i , "passthrough"),dtype=np.float32)
    depth_array= np.array(img, dtype=np.float32)
#    print(img)
    #print('called'+str(img.shape[0]))

 #   rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, hsr_yolo_callback)
 
#    cv2.imshow('a',img*600)
#    cv2.waitKey(1)
#    return
#
    bg_mx=np.zeros([img.shape[0],img.shape[1]/32])
    bg_mx[1,1]=0
    wd=64
    for i in range(0,img.shape[0]):
      for j in range(0,img.shape[1],wd):
        ii = i 
        jj = int(j/wd) 
        bg_mx[i,jj]=img[i,jj*wd:jj*wd+wd].max()
    

    for i in range(20,480):
      for j in range(0,img.shape[1]):
        ii = i 
        jj = int(j/wd) 
        img[i,j] = bg_mx[i,jj]- img[i,j] 
    
    img[:,0:50]=0
    img[:,590:640]=0
    img[0:100,:]=0
#    print(img)
#    print(type(img[0][0]))
#    _a,_b = cv2.threshold(img,30,255,cv2.THRESH_BINARY)

 #   return
    _,mask = cv2.threshold(img,37,65535,cv2.THRESH_BINARY)
#    _,mask = cv2.threshold(img,30,32767,cv2.THRESH_BINARY)
    cv2.imshow("a",mask)
    cv2.waitKey(1)
    umask = mask.astype(np.uint8)
    print(type(umask[0][0]))
    #print(umask)
    im, obj_cont, hierarchy = cv2.findContours(umask,cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    rect_list = draw_rect(obj_cont,img)

####    img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
#    img = cv2.drawContours(img, obj_cont, -1, (255,0,0), 2)
#    cv2.imshow('drawContours', img*600)
    '''
    if len(obj_cont) > 0:
		for i, contour in enumerate(obj_cont):
			#print("{}, {}".format(i, len(contour)))
			#輪郭を塗りつぶす
			#img = cv2.fillConvexPoly(img, obj_cont[i], (255,255-(i+1)*40,255-(i+1)*40))# 色を徐々に変えて塗りつぶし
			img = cv2.fillConvexPoly(img, obj_cont[i], (255,0,0))
			# area:面積
			#area = cv2.contourArea(contour)
			area = cv2.contourArea(obj_cont[i])
			#if area < 100 and area > 90:
			#	continue
			#else:
			#break

			# moments:重心
			M = cv2.moments(contour)
			if M["m00"] != 0:
				cx = int(M["m10"] / M["m00"])
				cy = int(M["m01"] / M["m00"])
			else:
				cx, cy = 0, 0

			hull = cv2.convexHull(contour)
			#img = cv2.drawContours(img,[hull],0,(255,255,0),2)

			# 楕円のフィッティング
			if area > MIN_OBJ_AREA:
			#if True:
				ellipse2 = cv2.fitEllipse(contour)
				img = cv2.ellipse(img,ellipse2,255*256,3)
#				print(ellipse2)
#				print(ellipse2[2])
#				cv2.imshow('ellipse', img*600)
    '''
    invK = np.linalg.inv(np.array(camera_info.K).reshape(3, 3))
    cnt = 1

#    corner_point_lt=np.zeros((3,3))
#    obj_corner_points=np.zeros((3,3))
    obj_floor_minimum = 5.0
######    for u,v,theta, cp1x, cp1y, cp2x, cp2y, cp3x, cp3y in rect_list:
    for u,v,theta in rect_list:
        z = depth_array[v][u] * 0.001 * 0.99
        if z >  0.00:
            image_point = np.array([u,v,1]) 
            obj_point = np.dot(invK,image_point) * z
            #np.sqrt(z*z-obj_point[0]*obj_point[0]-obj_point[1]*obj_point[1])
#            tilt_angle = whole_body.joint_positions.values()[1]
            tilt_angle = -1.0
            obj_floorpoint_y = np.absolute(obj_point[2]*np.cos(tilt_angle- np.deg2rad(4.0)))
            obj_floorpoint_x = np.absolute(obj_point[0])
            print(obj_floorpoint_x, obj_floorpoint_y)
            if obj_floorpoint_x < 0.5 and obj_floorpoint_y < 1.2 :
                obj_floor_norm = np.sqrt(obj_floorpoint_x*obj_floorpoint_x+obj_floorpoint_y*obj_floorpoint_y)
                if obj_floor_norm < obj_floor_minimum:
                    obj_floor_minimum = obj_floor_norm
#                    t = geometry_msgs.msg.TransformStamped()
                    ofs=  -0.7
#                    ofs=  -0.9
                    """
                    if ellipse[2] < 90:
            	        hand_rotate = ellipse[2]/180*np.pi
                    else:
            	        hand_rotate = (ellipse[2]-180)/180*np.pi
                    """
                    if theta < 90:
                        hand_rotate = theta/180*np.pi
                    else:
                        hand_rotate = (theta-180)/180*np.pi

                    #ofs= -1.57+gcnt*0.1
                    qa = quaternion_from_euler(0,0,-np.pi/2,'rxyz')
                    qb = quaternion_from_euler(0,ofs,0,'rxyz')
                    qc = quaternion_from_euler(0,0,hand_rotate,'rxyz')
                    qb = quaternion_multiply(qa,qb)
                    qb = quaternion_multiply(qb,qc)
#                    t.header = newimage.header
                    # inoue
#                    t.header.stamp = rospy.Time.now() - rospy.Duration(0.0)
                    #inoue end
#                    t.child_frame_id = 'floor_object'+ str(cnt)
#                    t.child_frame_id = 'floor_object1'
                    translation_x = obj_point[0]
                    translation_y = obj_point[1]
                    translation_z = obj_point[2]
                    #t.transform.rotation.z = np.sin(np.pi/2)
                    #t.transform.rotation.w = np.cos(np.pi/2)
                    rotation_x =  qb[0]
                    rotation_y =  qb[1]
                    rotation_z =  qb[2]
                    rotation_w =  qb[3]
                    #t.transform.rotation.z = 0
                    #t.transform.rotation.w = 1

    t = geometry_msgs.msg.TransformStamped()
                #ofs= -1.57+gcnt*0.1
#                 qa = quaternion_from_euler(0,0,-np.pi/2,'rxyz')
#                 qb = quaternion_from_euler(0,ofs,0,'rxyz')
#                 qc = quaternion_from_euler(0,0,hand_rotate,'rxyz')
#                 qb = quaternion_multiply(qa,qb)
#                 qb = quaternion_multiply(qb,qc)
    t.header = newimage.header
    # inoue
 #   t.header.stamp = rospy.Time.now() - rospy.Duration(0.0)
    #inoue end
#   t.child_frame_id = 'floor_object'+ str(cnt)
    t.child_frame_id = 'floor_object1'
    t.transform.translation.x = translation_x 
    t.transform.translation.y = translation_y
    t.transform.translation.z = translation_z
    #t.transform.rotation.z = np.sin(np.pi/2)
    #t.transform.rotation.w = np.cos(np.pi/2)
    t.transform.rotation.x =  rotation_x
    t.transform.rotation.y =  rotation_y
    t.transform.rotation.z =  rotation_z
    t.transform.rotation.w =  rotation_w
    #t.transform.rotation.z = 0
    #t.transform.rotation.w = 1
    tf_br.sendTransform([t])
    print('bload-cast!!')
    cnt = cnt + 1

    cv2.imshow('b',mask)
    #cv2.imshow('a',img*600)
    #cv2.imshow('a',or_img*600)

    cv2.waitKey(1)
    

def get_pixels_depth():
    global invK
    global camera_info

#    print(tf_br.header.stamp)

    #file = open('test.txt', 'w')
    rospy.Subscriber('/hsrb/head_rgbd_sensor/depth_registered/image_raw',Image,depth_callback)
    camera_info = rospy.wait_for_message(
            "/hsrb/head_rgbd_sensor/depth_registered/camera_info", CameraInfo)


rospy.init_node('heihei')
get_pixels_depth()
rospy.spin()
