#!/usr/bin/env python
# license removed for brevity
import rospy
#import smbus
#import time
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from math import pi
free_joints = {}
dependent_joints={}
joint_list = []


def enviarComandoMotores(data):
    # enviamos informacion a los motores
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
def crearJoints():
	global free_joints
	global joint_list
	minval=-pi
	maxval=pi
	zeroval=0
	name='nombre'
	joint_list.append(name)
 	joint = {'min': minval, 'max': maxval, 'zero': zeroval}
        joint['position'] = zeroval
        joint['velocity'] = 0.0
        joint['effort'] = 0.0
	joint['continuous'] = True
	free_joints[name] = joint
'''
def writeNumber(adress,action,position,vel):
    security=adress+action
    position1=position&0xff
    position2=(position>>8)&0xff
    vel1=vel&0xff
    vel2=(vel>>8)&0xff 
    bus.write_i2c_block_data(adress,cmd,    action,position1,position2,vel1,vel2,security])
    #time.sleep(1)
    #flag=bus.read_byte(adress)
    #flag=bus.read_i2c_block_data(adress,cmd)
    print flag
    return -1
'''

def leyendoencoder():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub_joints = rospy.Publisher('scorbot', JointState, queue_size=10)
    rospy.init_node('leyendoencoder', anonymous=True)
    rospy.Subscriber("chatter", String, enviarComandoMotores)
    rate = rospy.Rate(1) # 10hz
    crearJoints()
    count=0
    while not rospy.is_shutdown():
        #leer los datos de los arduinos
    #esto debe estar en un for donde se le pide a cada motor   
    #flag=bus.read_byte(adress)
    #flag=bus.read_i2c_block_data(adress,cmd)
        # organizar los datos para enviar por el topico sensors?msgs/JointStates
        #publicar Jointstates
	msg = JointState()
	msg.header.stamp = rospy.Time.now()
	num_joints = (len(free_joints.items()) +
		len(dependent_joints.items()))
        msg.position = num_joints * [0.0]
        msg.velocity = num_joints * [0.0]
        msg.effort = num_joints * [0.0]
	
        for i, name in enumerate(joint_list):
           msg.name.append(str(name))
           joint = None

           # Add Free Joint
           if name in free_joints:
               joint = free_joints[name]
               factor = 1
               offset = 0
           # Add Dependent Joint
           elif name in dependent_joints:
               param = dependent_joints[name]
               parent = param['parent']
               factor = param.get('factor', 1)
               offset = param.get('offset', 0)
               # Handle recursive mimic chain
               recursive_mimic_chain_joints = [name]
               while parent in dependent_joints:
                   if parent in recursive_mimic_chain_joints:
                      error_message = "Found an infinite recursive mimic chain"
                      rospy.logerr("%s: [%s, %s]", error_message, ', '.join(recursive_mimic_chain_joints), parent)
                      sys.exit(-1)
                   recursive_mimic_chain_joints.append(parent)
                   param = dependent_joints[parent]
                   parent = param['parent']
                   offset += factor * param.get('offset', 0)
                   factor *= param.get('factor', 1)
               joint = free_joints[parent]
           # hacer update con los datos que vienen del arduino
           joint['position']=count
	   joint['velocity']=count-1

           if 'position' in joint:
                msg.position[i] = joint['position'] * factor + offset
           if  'velocity' in joint:
                msg.velocity[i] = joint['velocity'] * factor
           if  'effort' in joint:
                msg.effort[i] = joint['effort']

        pub_joints.publish(msg)
        count=count+1
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        leyendoencoder()
    except rospy.ROSInterruptException:
        pass
