#!/usr/bin/env python3

import random

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
import uuid
import argparse


def main():

    # -------------------------------
    # Initialization
    # -------------------------------
    parser = argparse.ArgumentParser(description='Script to compute perfect numbers.')
    parser.add_argument('-l', '--location', type=str, help='', required=False,
                        default='on_bed')
    parser.add_argument('-o', '--object', type=str, help='', required=False,
                        default='Box_B')

    args = vars(parser.parse_args())  # creates a dictionary
    #print(args['location'])

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('robutler_description_T3') + '/models/'


    ###########################################################################
    # Defines poses where to put objects
    ###########################################################################
    poses = {}

    # on bed pose #
    p = Pose()
    p.position = Point(x=-6.033466, y=1.971232, z=0.644345)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed'] = {'pose': p}

    # on bed-side-table pose left #
    p = Pose()
    p.position = Point(x=-7.516057, y=2.733027, z=0.681438)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed_side_table_left'] = {'pose': p}

    # on bed-side-table pose right #
    p = Pose()
    p.position = Point(x=-4.434177, y=2.860802, z=0.681438)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bed_side_table_right'] = {'pose': p}

    # table bedroom #
    p = Pose()
    p.position = Point(x=-8.967117, y=1.559003, z=0.739959)
    q = quaternion_from_euler(0, 0, 0.787498)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['on_bedroom_table'] = {'pose': p}

    # strange room on chair #
    p = Pose()
    p.position = Point(x=-8.204075, y=-4.446789, z=0.363266)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['chair_strange_room'] = {'pose': p}

    # living_room_table #
    p = Pose()
    p.position = Point(x=1.281256, y=-1.638220, z=0.361972)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['living_room_table'] = {'pose': p}

    # living_room_sofa #
    p = Pose()
    p.position = Point(x=-0.207158, y=-1.265936, z=0.495777)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['living_room_sofa'] = {'pose': p}

    # table_gym #
    p = Pose()
    p.position = Point(x=-0.534364, y=4.018290, z=0.399243)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['table_gym'] = {'pose': p}

    # table_kitchen #
    p = Pose()
    p.position = Point(x=6.644286, y=0.742703, z=0.801420)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['table_kitchen'] = {'pose': p}

    # living_room #
    p = Pose()
    p.position = Point(x=-0.033629, y=0.411562, z=0.000284)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['living_room'] = {'pose': p}

    # kitchen #
    p = Pose()
    p.position = Point(x=7.616100, y=-4.093740, z=0.000000)
    q = quaternion_from_euler(0, 0, 0)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['kitchen'] = {'pose': p}

    # bedroom #
    p = Pose()
    p.position = Point(x=-5.178481, y=0.496342, z=0.00)
    q = quaternion_from_euler(0, 0, 3.116592)  # From euler angles (rpy) to quaternion
    p.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    poses['bedroom'] = {'pose': p}

    ############################################################################
    # Defines poses where to put objects (end)
    ############################################################################




    ########################### define objects ################################
    objects = {}

    # add object sphere_v
    f = open(package_path + 'sphere_v/model.sdf', 'r')
    objects['sphere_v'] = {'name': 'sphere_v', 'sdf': f.read()}

    #add object person_standing
    f = open(package_path + 'person_standing/model.sdf', 'r')
    objects['person_standing'] = {'name': 'person_standing', 'sdf': f.read()}

    #add object person_standing
    f = open(package_path + 'Box_B/model.sdf', 'r')
    objects['Box_B'] = {'name': 'Box_B', 'sdf': f.read()}

    #add object computer
    f = open(package_path + 'labtop_mac_1/model.sdf', 'r')
    objects['labtop_mac_1'] = {'name': 'laptop_mac_1', 'sdf': f.read()}

    #add object bottle red wine
    f = open(package_path + 'bottle_red_wine/model.sdf', 'r')
    objects['bottle_red_wine'] = {'name': 'bottle_red_wine', 'sdf': f.read()}

    #add object bottle white wines
    f = open(package_path + 'bottle_white_wine/model.sdf', 'r')
    objects['bottle_white_wine'] = {'name': 'bottle_white_wine', 'sdf': f.read()}

    #add object bottle white wines
    f = open(package_path + 'can_coke/model.sdf', 'r')
    objects['can_coke'] = {'name': 'can_coke', 'sdf': f.read()}

    ########################### define objects (end) ###########################



    #Check if given object and location are valid

    if not args['location'] in poses.keys():
        print('Location ' + args['location'] +
              ' is unknown. Available locations are ' + str(list(poses.keys())))

    if not args['object'] in objects.keys():
        print('Object ' + args['object'] +
              ' is unknown. Available objects are ' + str(list(objects.keys())))

    # -------------------------------
    # ROS
    # -------------------------------

    rospy.init_node('insert_object', log_level=rospy.INFO)

    service_name = 'gazebo/spawn_sdf_model'
    print('waiting for service ' + service_name + ' ... ', end='')
    rospy.wait_for_service(service_name)
    print('Found')

    service_client = rospy.ServiceProxy(service_name, SpawnModel)

    print('Spawning an object ...')
    uuid_str = str(uuid.uuid4())
    service_client(objects[args['object']]['name'] + '_' + uuid_str,
                   objects[args['object']]['sdf'],
                   objects[args['object']]['name'] + '_' + uuid_str,
                   poses[args['location']]['pose'],
                   'world')

    print('Done')


if __name__ == '__main__':
    main()
