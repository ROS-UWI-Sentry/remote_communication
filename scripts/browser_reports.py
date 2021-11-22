#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

                        ###note###
# ____________________________________________________
#|This is a ROS node that listens to a message from   |
#|the remote.                                         |
#|It is for creating and saving report files and      |
#|sending the data in one string variable.            |
#|See accompanying flowchart for overall operation    |
#|This node is a combination of a subscriber          |
#|and publisher.                                      |
#|You must alter the path to the reports.txt for      |
#|different users                                     |
#|____________________________________________________|


import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from multiprocessing import Process, Pipe
import thread
#variable to tell when to start sending the data from the file to the remote
#it is unused
startSending = 0





#this callback function gets called whenever a message is recieved
#it pushes the message to the terminal for us to confirm what happened
#and it checks the value of the message and performs actions based on it
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' I heard: %s', data.data)
    global startSending
    #if store data, append it to the file, else if read data publish the reporss
    if (data.data=="Initial Data"):
        rospy.logwarn("Initialized.")
        startSending=0
    elif (data.data=="Send Report"):
        #startSending=1
        #open file and read the file into temp variable
        with open("/home/uwi/catkin_ws/src/remote_communication/scripts/reports.txt",'r') as f:
            temp=f.read().splitlines()
        #publish the variable to the remote
        pub.publish(str(temp))


        rospy.loginfo("sent table")        
    elif (data.data=="Recieved Report"):
        startSending=0
    elif (data.data=="Clear Report"):
        #open the file such that it clears everything then close it
        f=open("/home/uwi/catkin_ws/src/remote_communication/scripts/reports.txt","w")
        f.close()
    else:
        #if the data is not recognized, this means that user data is coming in
        #open the file and write to it the incoming data
        with open("/home/uwi/catkin_ws/src/remote_communication/scripts/reports.txt", 'a') as f:
            f.write(data.data+"\n")
        rospy.loginfo(rospy.get_caller_id() + 'this is a test')


#    else:
#        rospy.logwarn("Incrorrect data received.")
        
# def publishData():
#     rate = rospy.Rate(100) #100Hz
#     n = 0
#     if (startSending==1):
#         with open("reports.txt",'r') as f:
#         temp=f.read().splitlines()
#         while not rospy.is_shutdown():
#             rospy.loginfo("The line is: " + temp[n])
#             pub.publish(temp[n])
#             n +=1
#             rate.sleep()


#this function is for subscribing to messages
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #check for message on Topic

    

    global startSending


    #this call creates a subscriber and defines message type and which topic it publishes to
    #whenever a message is received it calls the callback function
    rospy.Subscriber('brwsrButtonsData', String, callback)

    #this value is a sleep value
    rate = rospy.Rate(100) #100Hz

   
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #create a unique node
    rospy.init_node('reportsHandler', anonymous=True)
    #create a publisher object and defines which topic it subscribes to
    pub = rospy.Publisher('reportData', String, queue_size=100)
    
    #start the subscribing and publishing process
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
