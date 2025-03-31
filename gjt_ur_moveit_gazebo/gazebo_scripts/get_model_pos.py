#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from gazebo_msgs.msg import ModelStates
import tf

def model_states_callback(data):
    # Get the name and location information for all models
    model_names = data.name
    model_positions = data.pose
    model_list = []
    for name, pos in zip(model_names, model_positions):
        x = pos.position.x
        y = pos.position.y
        z = pos.position.z
        model_list.append({'name': name, 'position': [x, y, z]})
    for model in model_list:
        print("Model Name:", model['name'])
        print("Position:", model['position'])
    rospy.signal_shutdown("Got model positions")
    
def model_on_desk_states_callback(data):
    model_names = data.name
    model_positions = data.pose
    model_list = []
    with open('src/gjt_ur_moveit_gazebo/env_info/obj_on_desk.txt', 'r') as f:
        model_names_on_desk = f.read()
    print(model_names_on_desk)
    for name, pos in zip(model_names, model_positions):
        # if name in model_names_on_desk:
        x = pos.position.x
        y = pos.position.y
        z = pos.position.z
        qx = pos.orientation.x
        qy = pos.orientation.y
        qz = pos.orientation.z
        qw = pos.orientation.w
        roll, pitch, yaw = tf.transformations.euler_from_quaternion([qx, qy, qz, qw])
        model_list.append("{} {:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f}".format(name, x, y, z, roll, pitch, yaw))
    for model in model_list:
        print(model)

    rospy.signal_shutdown("Got model positions")


if __name__ == '__main__':
    rospy.init_node('model_positions_node', anonymous=True)
    rospy.Subscriber('/gazebo/model_states', ModelStates, model_on_desk_states_callback)
    rospy.spin()
