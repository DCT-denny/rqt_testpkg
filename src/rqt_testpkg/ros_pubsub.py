#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Version PYQT5

import os
import rospy
import rospkg
import math

from PyQt5 import QtCore
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget,QApplication
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from std_msgs.msg import Float64,Int16MultiArray,Int16,String,UInt8MultiArray,Float64MultiArray
from sensor_msgs.msg import LaserScan,Imu
import tf
import numpy as np
import cv2 as cv

class ROSdata(QWidget):

    Mode_Start_AUTO = 1
    Mode_Start_Remote = 2
    Mode_Stop = -1
    updata_sensor = QtCore.pyqtSignal()
    updata_wallfuzzy = QtCore.pyqtSignal()
    #updata_arm = QtCore.pyqtSignal()
    def __init__(self,context):
        super(ROSdata, self).__init__()
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_testpkg'), 'resource', 'MyPlugin.ui')
        loadUi(ui_file, self)

        self.progressBarList = [self.progressBar,
                                   self.progressBar_2,
                                   self.progressBar_3,
                                   self.progressBar_4,
                                   self.progressBar_5,
                                   self.progressBar_6]
        """
        self.progressBasrArmList=[self.progressBar_arm1,
                                    self.progressBar_arm2,
                                    self.progressBar_arm3,
                                    self.progressBar_arm4,
                                    self.progressBar_arm5,
                                    self.progressBar_arm6,
                                    self.progressBar_arm7,
                                    self.progressBar_arm8,
                                    self.progressBar_arm9]

        self.lcdNumberArmList=[self.lcdNumber_arm1,
                                self.lcdNumber_arm2,
                                self.lcdNumber_arm3,
                                self.lcdNumber_arm4,
                                self.lcdNumber_arm5,
                                self.lcdNumber_arm6,
                                self.lcdNumber_arm7,
                                self.lcdNumber_arm8,
                                self.lcdNumber_arm9]
        """
        self.lcdNumberWallfuzzyList=[self.lcdNumber_r_NB,
                                   self.lcdNumber_r_NS,
                                   self.lcdNumber_r_ZO,
                                   self.lcdNumber_r_PS,
                                   self.lcdNumber_r_PB,
                                   self.lcdNumber_angle_NB,
                                   self.lcdNumber_angle_NS,
                                   self.lcdNumber_angle_ZO,
                                   self.lcdNumber_angle_PS,
                                   self.lcdNumber_angle_PB]



        self.pushButtonList=[self.pushButton_num0,
                                self.pushButton_num1,
                                self.pushButton_num2,
                                self.pushButton_num3,
                                self.pushButton_num4,
                                self.pushButton_num5,
                                self.pushButton_num6,
                                self.pushButton_num7,
                                self.pushButton_num8,
                                self.pushButton_num9]

        self._sensor_value = [0, 0, 0, 0, 0, 0]
        self._arm_value=[0,0,0,0,0,0,0,0,0]
        self._wallfuzzy_value =[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

        self.subscriber_group()
        self.publisher_group()
        self.pushButton_start.clicked.connect(self.robot_start)
        self.pushButton_alarm.clicked.connect(self.alarm)

        for i in range(0,10):
            self.pushButtonList[i].clicked.connect(self.num_clicked)

        self.pushButton_clear.clicked.connect(self.num_clear)
        self.pushButton_enter.clicked.connect(self.num_enter)

        self.updata_sensor.connect(self.slot_line)
        self.updata_wallfuzzy.connect(self.slot_wall)
        #self.updata_arm.connect(self.slot_arm)


    def publisher_group(self):
        self.task_pub = rospy.Publisher('/task_num', Int16, queue_size=10)
        self.start_pub = rospy.Publisher('/robot_start', Int16, queue_size=10)
        self.alarm_pub = rospy.Publisher('/alarm', Int16, queue_size=10)

    def subscriber_group(self):
        rospy.Subscriber("/scan",LaserScan, self.laser_callback)
        rospy.Subscriber("/light_err",Float64, self.callback)
        rospy.Subscriber("/track_line_sensor",UInt8MultiArray, self.line_callback)
        rospy.Subscriber("/wall_msg",Float64MultiArray, self.wall_callback)
        rospy.Subscriber("/wall_fuzzy",Float64MultiArray, self.wallfuzzy_callback)
        #rospy.Subscriber("/arm_msg",Int16MultiArray, self.arm_callback) #暫時不用
        rospy.Subscriber("/task_msg",Int16, self.task_callback)
        rospy.Subscriber("/imu/data",Imu, self.imu_callback)
        #rospy.Subscriber("/arm_status",String, self.arm_status_callback) #暫時不用
        rospy.Subscriber("/move_it",Int16MultiArray, self.move_callback)
        rospy.Subscriber("/linesss",Int16MultiArray, self.view3wall_callback) #test view 3 wall

    def robot_start(self):
        if self.radioButton_AUTO.isChecked():
            self.start_pub.publish(self.Mode_Start_AUTO)
        elif self.radioButton_Remote.isChecked():
            self.start_pub.publish(self.Mode_Start_Remote)
        elif self.radioButton_Stop.isChecked():
            self.start_pub.publish(self.Mode_Stop)

    def alarm(self):
        self.alarm_pub.publish(1)


    def callback(self,err):
        #print("err" + str(err.data))
        #self.lcdNumber.display(err.data)
        pass

    def view3wall_callback(self,point):
        img = np.zeros((400,400,3), np.uint8)
        _x1=np.zeros(16,dtype=np.int)
        _x2=np.zeros(16,dtype=np.int)
        _y1=np.zeros(16,dtype=np.int)
        _y2=np.zeros(16,dtype=np.int)
        for i in range(0,16):
            _x1[i] = point.data[4*i]
            _y1[i] = point.data[4*i+1]
            _x2[i] = point.data[4*i+2]
            _y2[i] = point.data[4*i+3]
        for i in range(0,16):
            cv.line(img,(_x1[i],_y1[i]),(_x2[i],_y2[i]),(255,0+i*10,0),3)
        cv.line(img,(200,200),(210,200),(0,255,0),3)
        cv.line(img,(200,200),(200,190),(0,255,0),3)

        self.height, self.width, self.bytesPerComponent = img.shape
        self.bytesPerLine = 3 *self.width
        QImg = QImage(img.data, self.width, self.height, self.bytesPerLine,QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(QImg)
        self.label_qimage.setPixmap(pixmap)


    def move_callback(self,move_msg):
        self.lcdNumber_Lspeed.display(move_msg.data[0])
        self.lcdNumber_Rspeed.display(move_msg.data[1])

    def laser_callback(self,msg):
        if math.isnan(msg.ranges[85]):
            pass
        else:
            self.lcdNumber_right_scan.display(msg.ranges[85]*100)
        if math.isnan(msg.ranges[341]):
            pass
        else:
            self.lcdNumber_forward_scan.display(msg.ranges[341]*100)
        if math.isnan(msg.ranges[597]):
            pass
        else:
            self.lcdNumber_left_scan.display(msg.ranges[597]*100)




    def wall_callback(self,wall_msg):
        self.lcdNumber_final_r.display(wall_msg.data[0])
        self.lcdNumber_final_angle.display(wall_msg.data[1])
        self.lcdNumber_r_err.display(wall_msg.data[2])
        self.lcdNumber_angle_err.display(wall_msg.data[3])
        self.lcdNumber_sum.display(wall_msg.data[4])

    def wallfuzzy_callback(self,msg):
        for i in range(0,10):
            self._wallfuzzy_value[i] = msg.data[i]
        self.updata_wallfuzzy.emit()

    def slot_wall(self):
        for i in range(0,10):
            self.lcdNumberWallfuzzyList[i].display(self._wallfuzzy_value[i])


    def line_callback(self,line_msg):
        """
        self.progressBar.connect(self.slot_line)
        self.progressBar_2.connect(self.slot_line)
        self.progressBar_3.connect(self.slot_line)
        self.progressBar_4.connect(self.slot_line)
        self.progressBar_5.connect(self.slot_line)
        self.progressBar_6.connect(self.slot_line)
        """
        for i in range(0,6):
            self._sensor_value[i] = line_msg.data[i]

        #self.emit(SIGNAL("updateSensor"))
        self.updata_sensor.emit()

        """after 6 ,error,error_dot,L_speed,R_speed"""
        #self.lcdNumber_line_err.display(line_msg.data[6])
        #self.lcdNumber_line_errdot.display(line_msg.data[7])
        #self.lcdNumber_line_Lspeed.display(line_msg.data[8])
        #self.lcdNumber_line_Rspeed.display(line_msg.data[9])

    def slot_line(self):
        for i in range(0,6):
            self.progressBarList[i].setValue(ord(self._sensor_value[i]))

    """暫時應該不用
    def arm_callback(self,arm_msg):
        for i in range(0,9):
            self._arm_value[i] = arm_msg.data[i]

        self.updata_arm.emit()


    def slot_arm(self):
        for i in range(0,9):
            self.progressBarArmList[i].setValue(self._arm_value[i])
            self.lcdNumberArmList[i].dispaly(self._arm_value[i])
    #暫時應該不用
    """
    def num_clicked(self):
        sender = self.sender()
        old_value=self.lcdNumber_task_num.intValue()

        for i in range(0,10):
            if sender == self.pushButtonList[i] :
                if old_value != 0 :
                    self.lcdNumber_task_num.display(old_value *10 + i)
                else:
                    self.lcdNumber_task_num.display(i)



    def num_clear(self):
        self.lcdNumber_task_num.display(0)

    def num_enter(self):
        current_num = self.lcdNumber_task_num.intValue()
        self.task_pub.publish(current_num)


    def task_callback(self,task_msg):
        self.lcdNumber_task_num.display(task_msg.data)

    def imu_callback(self,imu_msg):
        (r, p, y) = tf.transformations.euler_from_quaternion( \
                    [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, \
                     imu_msg.orientation.w])

        angle=y*180/3.1415926
        self.lcdNumber_imu.display(angle)

    def arm_status_callback(self,status_msg):
        self.textEdit.setText(status_msg.data)


