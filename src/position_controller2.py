#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from robotiq_s_model_control.msg import _SModel_robot_input  as inputMsg
from time import sleep
from math import pi

# normalizes values from an arbitrary range to another new arbitary range
def normalize(value, oldRangeMin, oldRangeMax, newRangeMin, newRangeMax):
    result = (value - oldRangeMin) * ((newRangeMax - newRangeMin) / (oldRangeMax - oldRangeMin)) + (newRangeMin)
    return result


# calculates an approximation of a single joint state of the gripper, assuming that no collison during the grasp occured (-> no object is grasped / empty hand model)
def calculateAngle(finger, joint, fingerPosition, mode):
    angle = 0.0
    # finger offset for joint 0 (why is this needed???)
    finger_joint0_offset = 30.0
    finger_joint2_offset = -30.0

    if (joint==0):#alpha
        if (fingerPosition>140):
            angle = 35.0+finger_joint0_offset
        else: 
            angle = normalize(fingerPosition, 0.0, 140.0, -25.0+finger_joint0_offset, 33+finger_joint0_offset)

    elif (joint==1):
        if (fingerPosition<140):
            angle = 0.0;
        elif(fingerPosition>240):
            angle = 90.0
        else:
            angle = normalize(fingerPosition, 140.0, 240.0, 0.0, 90.0)

    elif(joint==2):
        if (fingerPosition>115):
            angle = -20.0+finger_joint2_offset
        else:
            angle = normalize(fingerPosition, 0.0, 115.0, 35+finger_joint2_offset, -20+finger_joint2_offset)

    return (angle/180)*pi


# calculates the joint state of the additional degree of freedom in 'scissor' direcection
def calulatePalmAngle(fingerPosition, mode):
    outside = -15.0
    inside = 10.0
    angle = 0.0

    if (mode==0): #basic
        angle = normalize(fingerPosition, 15.0, 232.0, outside, inside)
    elif (mode==1): #pinch
        angle = normalize(fingerPosition, 0.0, 219.0, outside, inside)
    elif (mode==2): #wide
        angle = normalize(fingerPosition, 6.0, 232.0, outside, inside)
    elif (mode==3): #scissor
        angle = normalize(fingerPosition, 15.0, 232.0, outside, inside)

    return (angle/180)*pi


# updates the global joint state message object
def updateLocalJointState(inputRegisterMessage):
    for finger in range(0, 3): 
        position = inputRegisterMessage.gPOC
        if (finger==1):
            position = inputRegisterMessage.gPOB
        elif(finger==2):
            position = inputRegisterMessage.gPOA
        for joint in range(0, 3):            
            currentJointState.position[finger*3+joint] = calculateAngle(finger, joint, position, inputRegisterMessage.gMOD)

    currentJointState.position[9] = -calulatePalmAngle(inputRegisterMessage.gPOS, inputRegisterMessage.gMOD)
    currentJointState.position[10] = -currentJointState.position[9]
    

# calculates, generates and publishes the joint states of the gripper from low-level register state messages of the /SModelRobotInput topic
def generateJointStatesFromRegisterStateAndPublish():
    rospy.Subscriber("SModelRobotInput", inputMsg.SModel_robot_input, updateLocalJointState)

    global currentJointState 
    currentJointState = JointState()
    currentJointState.header.frame_id = "robotiq_s_model from real-time state data"
    currentJointState.name = ["finger_1_joint_1", 
                                "finger_1_joint_2", 
                                "finger_1_joint_3", 
                                "finger_2_joint_1", 
                                "finger_2_joint_2", 
                                "finger_2_joint_3", 
                                "finger_middle_joint_1", 
                                "finger_middle_joint_2", 
                                "finger_middle_joint_3", 
                                "palm_finger_1_joint", 
                                "palm_finger_2_joint"]

    currentJointState.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    currentJointState.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    currentJointState.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    pub = rospy.Publisher('joint_states', JointState, queue_size=1) #no buffering?
    rospy.init_node('robotiq_s_model_joint_state_publisher', anonymous=False)
    rate = rospy.Rate(100) # publishes at 100Hz

    while not rospy.is_shutdown():
        currentJointState.header.stamp = rospy.Time.now()
        pub.publish(currentJointState)
        rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    try:
        generateJointStatesFromRegisterStateAndPublish()
    except rospy.ROSInterruptException:
        pass
