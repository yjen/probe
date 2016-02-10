#!/usr/bin/python
from std_msgs.msg import String, Float64
from geometry_msgs.msg import PoseStamped
import rospy

import numpy as np
import matplotlib.pyplot as plt
import serial, time
import pickle
import palpation_probe as pp

import tfx


########

#CHANGE CONSTANTS TO CLASS VARS

#PLOTTING ONLY WORKS WITH MATPLOTLIB 1.1.1
#ros hydro
#numpy 1.9.2
#ubuntu 12.04

########


START = "start"
STOP = "stop"
WINDOW = 700
REFRESH_RATE = 10
RECORD_RATE = 2
SERIAL_DEVICE = '/dev/ttyACM1'



class Probe:
    

    def __init__(self):
        print('init')
        self.palpation_data = []
        self.pose_data = []
        self.status = None
        self.recording_pose = False
        self.publisher_start = None
        self.publisher_end = None
        self.ser = serial.Serial(SERIAL_DEVICE, 9600)


    def read1(self):
        data = self.ser.readline()
        while True:
            while data.strip() == '':
                data = self.ser.readline()
                print 'no data'
            data = data.strip().split(',')[1]
            data = data.strip().split('\r')
            result = [int(x) for x in data if x != '']
            if len(result) != 0:
                return result
        return None

    def read(self):
        try:
            return self.read1()
        except (ValueError, IndexError) as e:
            return self.read()

    def record_status_callback(self, data):
        self.status = data.data
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        
        if self.status == START:
            self.palpation_data = []
            self.pose_data = []
        elif self.status == STOP:
            print(self.palpation_data)
            print(self.pose_data)
            print(str(len(self.palpation_data)) + " palp")
            print(str(len(self.pose_data)) + " pose")
            if len(self.palpation_data) != len(self.pose_data):
                print("WARNING: lengths of data arrays do not match")
            
            while len(self.palpation_data) > len(self.pose_data):
                self.palpation_data = self.palpation_data[:len(self.palpation_data)-1]
                print(str(len(self.palpation_data)) + " palp")
            while len(self.pose_data) > len(self.palpation_data):
                self.pose_data = self.pose_data[:len(self.palpation_data)-1]
                print(str(len(self.pose_data)) + " pose")

            palp_data_file = open('palpation_data_.csv', 'w')
            pose_data_file = open('pose_data_.p', 'wb+')
            for d in self.palpation_data:
                palp_data_file.write(str(d) + ',')
            pickle.dump(self.pose_data, pose_data_file)
            pose_data_file.close()
            palp_data_file.close()
            
            combined_data = [(self.palpation_data[i], self.pose_data[i]) for i in range(len(self.palpation_data))]

            combined_data_file = open('combined_data_.p', 'wb+')
            pickle.dump(combined_data, combined_data_file)
            combined_data_file.close()


            # palp_data_file = open('palpation_data', 'r')
            # pose_data_file = open('pose_data', 'rb+')
            
            # self.palpation_data = palp_data_file.read().split(',')
            # self.palpation_data = [float(d) for d in self.palpation_data]

            # self.pose_data = pickle.load(pose_data_file)


            start_end_points = pp.estimate_vein_location(self.palpation_data, self.pose_data, False)
            print(start_end_points)
            start_pose = tfx.pose(start_end_points[0]) # x,y,z
            end_pose = tfx.pose(start_end_points[1])
            self.publisher_start.publish(start_pose.msg.PoseStamped())
            self.publisher_end.publish(end_pose.msg.PoseStamped())


    def record_pose_callback(self, data):
        if self.recording_pose:
            self.pose_data.append(data)
            #print(str(time.clock()) + 'pose')
            self.recording_pose = False



    def listener(self):
        self.publisher_start = rospy.Publisher("/palpation/cut_start_point", PoseStamped)
        self.publisher_end = rospy.Publisher("/palpation/cut_end_point", PoseStamped)
        self.measurement_pub = rospy.Publisher("/palpation/measurement", Float64)
        rospy.init_node('arduino_in', anonymous=True)
        rospy.Subscriber("palpation_record", String, self.record_status_callback)
        rospy.Subscriber("dvrk_psm1/joint_position_cartesian", PoseStamped, self.record_pose_callback)   
        
        x = np.r_[:WINDOW]
        z = np.array([0]*WINDOW) #raw data
        w = np.array([0]*WINDOW) #smoothed data
        y = np.array([0]*WINDOW) #fake

        z[0] = 2000 # used to be 15000
        w[0] = 2000


        # print('graph')


       

        # stuff to comment out
        # fig = plt.figure(figsize=(12,12))


        # ax = fig.add_subplot(511)
        # aw = fig.add_subplot(512)
        # a1 = fig.add_subplot(513)
        # a2 = fig.add_subplot(514)
        # a3 = fig.add_subplot(515)
        # ly, = ax.plot(x, y)
        # lz, = ax.plot(x, z)
        # lw, = aw.plot(x, w)
        # la1, = a1.plot(x, w)
        # la2, = a2.plot(x, w)
        # la3, = a3.plot(x, w)
        # aw.set_ylim([9000, 12000])
        # a3.set_ylim([0, 3000])
        # a2.set_ylim([3000, 6000])
        # a1.set_ylim([6000, 9000])
        # # draw and show it

        # fig.canvas.draw()

        # plt.show(block=False)

        # 


        i = 0
        j = 0
        k = 0
        time_count = 0
        start_time = -1
        iterations = 0
        time_counts = []
        prev = 0.0

        while not rospy.is_shutdown():
            try:
                data = self.read()
                z = np.append(z, data)
                elem = z[-1]

                w = np.append(w, prev)
                prev = 0.9*prev + 0.1*elem
                
                # stuff to comment out
                # if i == 0:
                #     lz.set_ydata(z[-WINDOW:])
                #     ly.set_ydata(y[-WINDOW:])
                #     lw.set_ydata(w[-WINDOW:])
                #     la1.set_ydata(w[-WINDOW:])
                #     la2.set_ydata(w[-WINDOW:])
                #     la3.set_ydata(w[-WINDOW:])
                #     fig.canvas.draw()
                # i += 1
                # i = i % REFRESH_RATE
                


                # cur_time = time.time()
                # if start_time == -1:
                #     start_time = cur_time
                # if cur_time - start_time > 1:
                #     print(time_count)
                #     time_counts.append(time_count)
                #     time_count = 0
                #     start_time = cur_time
                #     iterations += 1
                # else:
                #     time_count += 1
                # if iterations >= 5:
                #     print(time_counts)
                #     return



                if j == 0:
                    if self.status == START: # status == START
                        # print(str(time.clock()) + 'palp')
                        self.palpation_data.append(w[-1])
                        self.measurement_pub.publish(w[-1])
                        self.recording_pose = True
                        # used to find how fast probe is sending data
                        # cur_time = time.time()
                        # if start_time == -1:
                        #     start_time = cur_time
                        # if cur_time - start_time > 1:
                        #     print(time_count)
                        #     time_counts.append(time_count)
                        #     time_count = 0
                        #     start_time = cur_time
                        #     iterations += 1
                        # else:
                        #     time_count += 1
                        # if iterations >= 5:
                        #     print(time_counts)
                        #     return
                j += 1
                j = j % RECORD_RATE

            except KeyboardInterrupt:
                print('noooo')
                break
        print(rospy.is_shutdown())
        rospy.spin()

if __name__ == '__main__':
    p = Probe()
    p.listener()
