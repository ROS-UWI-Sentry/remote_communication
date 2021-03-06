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

## This code is to publish a message to the broswer so that it can respond
#and the ROS on the jetson would know that it is connected 

#It counts up to 30 seconds then publishes a request to the remote
#if the remote does not respond within another 30 seconds the
#code will stop counting and continuously output "turn_off_sentry"
#on the "sentry_control_topic"

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int32
from multiprocessing import Process, Pipe
import thread

                        ###note###
# ____________________________________________________
#|This is a heartbeat to the Remote using a simple    |
#|counter.                                            |
#|See accompanying flowchart for overall operation    |
#|This node is a combination of a subscriber          |
#|and publisher.                                      |
#|____________________________________________________|



##########GLOBAL VARIABLES##########

reset = False
#awaiting_response is the "has a connection request been sent?" variable,
#if it is true, and a response message is not received after 30s,
#then the connection to the remote is lost
awaiting_response = False

#this variable tells the node if the user is on the sanitization page,
#so that it can only check for connection while sanitation is happening
heartbeat = False
#this variable is for if within 30s a connected message was received or not
no_connection_confirmed = False


#this callback function gets called whenever a message is received
#it pushes the message to the terminal for us to confirm what happened
#and it checks the value of the message and performs actions based on it
def callback_heartbeat(data):
    global reset, awaiting_response, heartbeat #percent 

    rospy.loginfo(rospy.get_caller_id() + ' I heard: %s', data.data)
    
    #check if the message received is "connected" so we know that the remote 
    #is connected to rosbridge
    if (data.data=="connected"): #this also continues timing
        rospy.loginfo(rospy.get_caller_id() + " connected to remote")
        #if the browser returns that it is connected within 30s of requesting, 
        #awaiting_response is set to false
        #else if it remains true after 30s then the remote is not connected
        awaiting_response = False
        #we cannot set heartbeat=True here because whenever it is false 
        #the system will be shutting down
    elif (data.data=="on_sanitization_page"):
        rospy.loginfo(rospy.get_caller_id() + " received that the user is on the sanitzation page")
        #start the process of heartbeating
        heartbeat=True 
        #in the event of reconnection we have to stop what happens while connection lost
        awaiting_response = False  
    else:
        rospy.logwarn("Incorrect data received.")


def callback_sentry_control_topic(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard: %s', data.data)
    global heartbeat, no_connection_confirmed
    #if the start message was received set this variable to true
    if (data.data=="turn_off_sanitization"):
        #this node is alive even when the user isn't sanitizing, 
        #so this ensures this node doesn't send the shutdown message 
        #while the user is on another page on the remote
        heartbeat=False
        no_connection_confirmed=False
        rospy.loginfo(rospy.get_caller_id() + " sanitization was stopped by the user, no more checking communication")
        
#this function is for subscribing to messages
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #check for message on Topic

    global reset, awaiting_response, heartbeat, no_connection_confirmed #percent

    #this call creates a subscriber and defines message type and which topic it publishes to
    #whenever a message is received it calls the callback function
    rospy.Subscriber('/heartbeat_rx', String, callback_heartbeat)

    rospy.Subscriber('/sentry_control_topic', String, callback_sentry_control_topic)

    #this value is a sleep value
    rate = rospy.Rate(1) #1Hz
    
    ######listener variables ######

    #counting variable
    i = 0
    #how much to count up to
    t = 30

    #variable to check how many times "t" seconds has passed
    #checks how many times a connection request was not replied to 
    c = 0

    #while ROS is not shutdown via terminal etc, if the conditions are met:
    while not rospy.is_shutdown() :
        #only if the user starts sanitization, start counting
        if heartbeat:
            #if i is less than t
            if (i<t):
                now = rospy.get_time()  
                #rospy.loginfo(now)
                #rospy.loginfo(i)
                #increment i
                i = i + 1
            #if i is the value of t(t seconds passed) and this is the first run or 
            #the connection was confirmed(response received)
            elif (i==t and awaiting_response==False):
                #reset i to restart the count
                i = 0
                #if the connection is restored, reset this variable to 
                #be able to count how many times a connection
                #request was not replied to 
                c = 0
                #request the remote to confrim its connection
                pub_heartbeat_ros_remote.publish("connection_test")
                rospy.loginfo(rospy.get_caller_id() + " Connection request sent.")                   
                #set awaiting_response to true, so if in 30s no confirmation is received
                #this condition will be skipped and the other one will be carried out
                awaiting_response = True
            #if i equals the value of t(t seconds passed) 
            #and the connection was not confirmed(no response was received)
            #within that time to set awaiting_response to False:        
            elif (i==t and awaiting_response==True):
                #publish pause sanitization
                #if no response from the remote is obtained, this should
                #this should ensure that the UV lights are off.
                pub_heartbeat_state_machine.publish("pause_sanitization")
                #reset i so we can keep asking for connection
                i = 0
                #increment how many times this condition was entered
                c = c + 1
                rospy.loginfo(rospy.get_caller_id() + " Connection request sent %s times.", c)
                #has 6*t seconds (3 minutes) passed with no response from remote?
                #or have we entered this condition 6 times?
                if (c==6):
                    #to break the loop and keep publishing at a set rate
                    heartbeat=False
                    no_connection_confirmed=True

        #if no conenction was seen within c*t seconds after requesting it keep 
        #publishing to turn off sentry
        elif no_connection_confirmed:
            #publish turn off sentry
            pub_heartbeat_state_machine.publish("turn_off_sentry")
            rospy.logfatal("No communication with remote after %s seconds, shutting down entire system", c*t)
 
        #wait for 1s
        rate.sleep()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    #create a unique node
    rospy.init_node('heartbeat_ros_remote', anonymous=True)
    #create a publisher object and defines which topic it subscribes to
    pub_heartbeat_ros_remote = rospy.Publisher('heartbeat_tx', String, queue_size=10)

    pub_heartbeat_state_machine = rospy.Publisher('sentry_control_topic', String, queue_size=10)
    
    #start the subscribing and publishing process
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
    

