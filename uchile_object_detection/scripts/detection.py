#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

import tf
import cv2
import tf2_ros
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2, Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
import tf2_sensor_msgs

from yolov5.detect import YoloV5
import matplotlib.pyplot as plt

#np.set_printoptions(threshold=sys.maxsize)

class RGBD():
    u"""RGB-"""

    def __init__(self):
        self._br = tf.TransformBroadcaster()
        # 
        self.model = YoloV5(weights='ycb_v8.pt')
        self.names = self.model.names
        self.bridge = CvBridge()
        # self._cloud_sub = rospy.Subscriber(
        #    "/hsrb/head_rgbd_sensor/depth_registered/rectified_points",
        #    PointCloud2, self._cloud_cb)
        self.pcloud_pub = rospy.Publisher("/bender/sensors/rgbd_head/depth/points_mask", PointCloud2, queue_size=1, latch=True)
        self.pcloud_pub_rot = rospy.Publisher("/bender/sensors/rgbd_head/depth/points_maskrot", PointCloud2, queue_size=1, latch=True)
        self.pcloud_env_mask = rospy.Publisher("/move_group/filtered_cloud", PointCloud2, queue_size=1, latch=True)
        self._pc = None
        self._points_data = None
        self._image_data = None
        self.xyz = []
        self.labels = []
        self._w_image = 640
        self._h_image = 480
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def detect(self, sort=True):
        """Funcion que retorna 2 listas, una con los xyz de la nube de puntos (ordenadas del mas cercano al mas lejano si 
        sort = True y una lista con sus labels respectivos.
        """
        msg = rospy.wait_for_message(
            "/bender/sensors/rgbd_head/depth/points", PointCloud2)
        self._points_data = ros_numpy.numpify(msg)
        self._pc = ros_numpy.numpify(msg)
        self._image_data = self._points_data['rgb'].view(
            (np.uint8, 4))[..., [0, 1, 2]]
        self.xyz = []
        self.labels = []
        _xy = []
        if not self._image_data is None:
            detections = self.model.detect(self._image_data)
            if not detections == []:
                for i, (x1, y1, x2, y2, cls_conf, i_label) in enumerate(detections):
                    _x = int(round(x2.item() - x1.item())/2 + x1.item())
                    _y = int(round(y2.item() - y1.item())/2 + y1.item())
                    _w = int(round(x2.item() - x1.item()))
                    _h = int(round(y2.item() - y1.item()))
                    conf = cls_conf
                    label = str(self.names[int(i_label.item())])
                    _xy.append((_x, _y, label))

                if sort:
                    # ordenar los objetos si es necesario
                    _xy = self.sort_objects(_xy)

                # transformacion coordenada de imagen a nube de puntos
                for i, (_x, _y, label) in enumerate(_xy):
                    x = self._points_data['x'][_y, _x]
                    y = self._points_data['y'][_y, _x]
                    z = self._points_data['z'][_y, _x]
                    if not np.isnan(z):
                        # agregar a self._xyz las coordenadas con respecto a la nube
                        self.xyz.append([x, y, z])
                        self.labels.append(label)
                

            return self.xyz, self.labels

        else:
            return None, None

    def segmentation(self, selected_object=None, sort=True):

        msg = rospy.wait_for_message(
            "/bender/sensors/rgbd_head/depth/points", PointCloud2)
        self._points_data = ros_numpy.numpify(msg)
        self._pc = ros_numpy.numpify(msg)
        self._image_data = self._points_data['rgb'].view(
            (np.uint8, 4))[..., [0, 1, 2]]
        _xywh = []
        if not self._image_data is None:
            detections = self.model.detect(self._image_data)
            if not detections == []:
                for i, (x1, y1, x2, y2, cls_conf, i_label) in enumerate(detections):
                    _x = int(round(x2.item() - x1.item())/2 + x1.item())
                    _y = int(round(y2.item() - y1.item())/2 + y1.item())
                    _w = int(round(x2.item() - x1.item()))
                    _h = int(round(y2.item() - y1.item()))
                    conf = cls_conf
                    label = str(self.names[int(i_label.item())])
                    _xywh.append((_x, _y, _w, _h, label))

                if sort:
                    # ordenar los objetos si es necesario
                    _xywh = self.sort_objects(_xywh)
                
                obj = None
                if selected_object is None:
                    obj = _xywh[0]  # object nearest
                else: # selected object
                    found_object = False
                    for i, d in enumerate(_xywh):
                        if d[4] == selected_object:
                            obj = d
                            found_object = True
                        if found_object:
                            print('Se encontro el objeto seleccionado')
                        else:
                            print('NO se encontro objecto seleccionado')
                    if obj is None:
                        return "retry"
                mask_obj = self.segmentation_object(obj)
                return mask_obj
            return None
        return None

    def segmentation_object(self, obj):
        _point_data = self._points_data.copy()
        _env_data = self._pc.copy()

        frame_rgb = self._image_data.copy()
        frame_mask = np.zeros((480, 640))
        anti_mask = np.ones((480, 640))
        x, y, w, h, label = obj
        frame_obj = frame_rgb[int(y-h/2):int(y+h/2), int(x-w/2):int(x+w/2)]

        frame_hsv = cv2.cvtColor(
            frame_obj, cv2.COLOR_RGB2HSV_FULL)  # convert to hsv
        # valores mesa RGB
        lower_rgb = np.array([234, 229, 234])
        upper_rgb = np.array([255, 255, 255])
        mask_mesa = 255-cv2.inRange(frame_obj, lower_rgb, upper_rgb)

        # valores para piso HSV
        lower_hsv = np.array([144, 109, 110])
        upper_hsv = np.array([150, 255, 255])
        mask_piso = 255-cv2.inRange(frame_hsv, lower_hsv, upper_hsv)
        kernel = np.ones((5,5), np.uint8)
        mask_piso = cv2.erode(mask_piso, kernel)

        # valores para pared RGB
        lower_rgb = np.array([29,29,29])
        upper_rgb = np.array([32,32,32])
        mask_pared = 255-cv2.inRange(frame_obj, lower_rgb, upper_rgb)

        result = cv2.bitwise_and(mask_piso, mask_mesa)
        result = cv2.bitwise_and(result, mask_pared)
        not_result = cv2.bitwise_not(result)

        frame_mask[int(y-h/2):int(y+h/2), int(x-w/2):int(x+w/2)] = result
        anti_mask[int(y-h/2):int(y+h/2), int(x-w/2):int(x+w/2)] = not_result
        not_mask = cv2.bitwise_not(frame_mask)

        result_x = (self._points_data['x']*frame_mask/255)
        result_y = (self._points_data['y']*frame_mask/255)
        result_z = (self._points_data['z']*frame_mask/255)

        env_x = (self._pc['x']*anti_mask)
        env_y = (self._pc['y']*anti_mask)
        env_z = (self._pc['z']*anti_mask)

        for i in range(self._w_image):
            for j in range(self._h_image):
                if abs(result_x[j,i]) < 0.001: 
                    result_x[j,i] = np.nan
                if abs(result_y[j,i]) < 0.001:
                    result_y[j,i] = np.nan
                if abs(result_z[j,i]) < 0.001: 
                    result_z[j,i] = np.nan

        for i in range(self._w_image):
            for j in range(self._h_image):
                if abs(env_x[j,i]) < 0.001: 
                    env_x[j,i] = np.nan
                if abs(env_y[j,i]) < 0.001:
                    env_y[j,i] = np.nan
                if abs(env_z[j,i]) < 0.001: 
                    env_z[j,i] = np.nan

        _point_data['x'] = result_x
        _point_data['y'] = result_y
        _point_data['z'] = result_z

        _env_data['x'] = env_x
        _env_data['y'] = env_y
        _env_data['z'] = env_z

        msg = ros_numpy.msgify(PointCloud2, _point_data)
        env = ros_numpy.msgify(PointCloud2, _env_data)

        msg.header.frame_id = "bender/sensors/rgbd_head_depth_optical_frame"
        env.header.frame_id = "bender/sensors/rgbd_head_depth_optical_frame"
        
        self.pcloud_pub.publish(msg)
        #self.pcloud_env_mask.publish(env)



        try:
            trans = self.tf_buffer.lookup_transform('bender/base_link', msg.header.frame_id,
                                           msg.header.stamp,
                                           rospy.Duration(1))
        except tf2_ros.LookupException as ex:
            rospy.logwarn(ex)
            return
        except tf2_ros.ExtrapolationException as ex:
            rospy.logwarn(ex)
            return

        cloud_out = tf2_sensor_msgs.tf2_sensor_msgs.do_transform_cloud(msg, trans)

        self.pcloud_pub_rot.publish(cloud_out)

        return frame_mask, env

    def sort_objects(self, xy):
        x_ref, y_ref = self._w_image/2, self._h_image
        # ordenar puntos por distancia
        xy = sorted(xy, key=lambda x: (x[0]-x_ref)**2 + (x[1]-y_ref)**2)
        return xy

    def get_image(self):
        return self._image_data

    def get_points(self):
        return self._points_data

    def get_xyz(self):
        return self.xyz

    def get_labels(self):
        return self.labels

    def publish_env(self, env):
        self.pcloud_env_mask.publish(env)
        return

    def get_relative_coordinate(self, parent, child):

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        print(tfBuffer)
        trans = TransformStamped()
        while not rospy.is_shutdown():
            try:
                # 4
                trans = tfBuffer.lookup_transform(parent, child,
                                                  rospy.Time().now(),
                                                  rospy.Duration(4.0))
                break
            except (tf2_ros.ExtrapolationException):
                pass

        return trans.transform

    def _cloud_cb(self, msg):
        self._points_data = ros_numpy.numpify(msg)
        self._image_data = self._points_data['rgb'].view(
            (np.uint8, 4))[..., [2, 1, 0]]
        if not self._image_data is None:
            detections = self.model.detect(self._image_data)
            if not detections == []:
                for i, (x1, y1, x2, y2, cls_conf, i_label) in enumerate(detections):
                    _x = int(round(x1.item()))
                    _y = int(round(y1.item()))
                    _w = int(round(x2.item() - x1.item()))
                    _h = int(round(y2.item() - y1.item()))
                    conf = cls_conf
                    label = self.names[int(i_label.item())]
                    x = self._points_data['x'][_y, _x]
                    y = self._points_data['y'][_y, _x]
                    z = self._points_data['z'][_y, _x]
                    if not np.isnan(z):
                        self._xyz.append([x, y, z])
                        self._frame_name.append(label)
                        # print('{},{},{},{}'.format(x,y,z,label))

            if i in range(len(self._frame_name)):
                (x, y, z) = self._xyz[i]
                self._br.sendTransform(
                    (x, y, z), tf.transformations.quaternion_from_euler(0, 0, 0),
                    rospy.Time(msg.header.stamp.secs, msg.header.stamp.nsecs),
                    self._frame_name[i],
                    msg.header.frame_id)

        else:
            return

if __name__ == '__main__':
    rospy.init_node("detection")
    vision_model = RGBD()
    objects = vision_model.detect()
    print(objects)
    rate = rospy.Rate(0.5) # 10hz

    #while not rospy.is_shutdown():
    #    obj_mask = vision_model.segmentation("mustard_bottle")
    obj_mask, env = vision_model.segmentation("mustard_bottle")

    #while not rospy.is_shutdown():
    #   vision_model.publish_env(env)
    #   rate.sleep()
    