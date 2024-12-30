import h5py
import rosbag
import scipy.spatial.transform as st
import cv2
import numpy as np
from cv_bridge import CvBridge
import rospy
import sys
import os

def current_data_missing(data):
    return any([v is None for v in data['obs'].values()])

def current_data_history(data, minimum_len):
    for k, v in data['obs'].items():
        if 'forcetorque' in k:
            if len(v) == 0:
                return False
            if len(v) < minimum_len:
                padding = minimum_len-len(v)
                pre =  []
                for i in range(padding):
                    pre.append(v[-1])
                # print(pre)
                # print(len(pre))
                # print(v[-1])
                pre.extend(v)
                data['obs'][k] = pre
                # print(len(data['obs'][k]))
                # for i in data['obs'][k]:
                #     print(i)
    return True


def bag_h5(input_file, output_file, video_HD=False):
    data_h5 = {
        'actions': [],
        'actions_abs': [],
        'obs': {
            'robot0_rgb': [],
            'robot1_rgb': [],
            'robot0_eef_pos': [],
            'robot0_eef_quat': [],
            'robot0_gripper_qpos': [],
            'robot0_forcetorque': [],
            'robot1_eef_pos': [],
            'robot1_eef_quat': [],
            'robot1_gripper_qpos': [],
            'robot1_forcetorque': [],
        }
    }

    data_overall = {
        'obs': {
            'robot0_rgb': [],
            'robot1_rgb': [],
            'robot0_eef_pos': [],
            'robot0_eef_quat': [],
            'robot0_gripper_qpos': [],
            'robot0_forcetorque': [],
            'robot1_eef_pos': [],
            'robot1_eef_quat': [],
            'robot1_gripper_qpos': [],
            'robot1_forcetorque': [],
        }
    }

    data_latest = {
        'obs': {
            'robot0_rgb': None,
            'robot1_rgb': None,
            'robot0_eef_pos': None,
            'robot0_eef_quat': None,
            'robot0_gripper_qpos': None,
            'robot0_forcetorque': None,
            'robot1_eef_pos': None,
            'robot1_eef_quat': None,
            'robot1_gripper_qpos': None,
            'robot1_forcetorque': None,
        }
    }


    # Lightning = [robot0, Thunder = robot1
    len_history = 32
    resolution = (84, 84)

    bridge = CvBridge()
    missed, added = 0, 0
    time_last = None
    freq = 1/20.0
    image_crop, width, hieght = (120,700), 500, 500
    # for topic, msg, t in rosbag.Bag('force_ryan.bag').read_messages():
    # for topic, msg, t in rosbag.Bag('../rerun/test.bag').read_messages():
    for topic, msg, t in rosbag.Bag(input_file).read_messages():

        if 'lightning' in topic:
            arm='robot0'
        elif 'thunder' in topic:
            arm='robot1'

        if 'cartesian_eef' in topic: # msg.data = x,y,z,rx,ry,rz
            data_latest['obs'][f'{arm}_eef_pos'] = msg.data[:3]
            data_latest['obs'][f'{arm}_eef_quat'] = st.Rotation.from_euler("xyz", msg.data[3:]).as_quat()
        elif 'gripper' in topic:
            data_latest['obs'][f'{arm}_gripper_qpos'] = msg.data/255
        elif 'ft'in topic and not 'raw' in topic:
            data_latest['obs'][f'{arm}_forcetorque'] = msg.data

        # if 'cameras/top/depth' in topic:
        #     # image = bridge.imgmsg_to_cv2(msg)
        #     raw = np.frombuffer(msg.data, dtype=np.uint16)
        #     image = raw.reshape(msg.height, msg.width)
        #     image = image[image_crop[0]:image_crop[0]+width, image_crop[1]:image_crop[1]+hieght]
        #     image = cv2.resize(image, resolution, interpolation=cv2.INTER_NEAREST)
        #     image = image.astype(np.float32)
        #     data_latest['obs']['overhead_depth'] = image
        # elif 'cameras/top/rgb' in topic:
        #     image = bridge.imgmsg_to_cv2(msg, 'rgb8')
        #     image = image[image_crop[0]:image_crop[0]+width, image_crop[1]:image_crop[1]+hieght]
        #     image = cv2.resize(image, resolution, interpolation=cv2.INTER_NEAREST)
        #     data_latest['obs']['overhead_image'] = image
        if '/cameras/rgb/lightning/wrist' in topic:
            image = bridge.imgmsg_to_cv2(msg, 'rgb8')
            if not video_HD:
                image = cv2.resize(image, resolution, interpolation=cv2.INTER_NEAREST)
            data_latest['obs']['robot0_rgb'] = image
        elif '/cameras/rgb/thunder/wrist' in topic:
            image = bridge.imgmsg_to_cv2(msg, 'rgb8')
            if not video_HD:
                image = cv2.resize(image, resolution, interpolation=cv2.INTER_NEAREST)
            data_latest['obs']['robot1_rgb'] = image


        if time_last is None:
            if current_data_missing(data_latest):
                missed += 1
            else:
                time_last = t
            continue
        added += 1

        time_diff = t - time_last
        if time_diff.to_sec() > freq:
            time_last += rospy.Duration.from_sec(freq)

            if current_data_history(data_overall, len_history):
                action = tuple(data_latest['obs']['robot1_eef_pos'][i] - data_overall['obs']['robot1_eef_pos'][-1][i] for i in range(3))
                data_h5['actions'].append(action)
                data_h5['actions_abs'].append(data_latest['obs']['robot1_eef_pos'])
                for k, v in data_overall['obs'].items():
                    if 'forcetorque' in k:
                        data_h5['obs'][k].append(v.copy())
                        # print(f"{k}: {v}")
                    else:
                        data_h5['obs'][k].append(v[-1])
                
            for k, v in data_latest['obs'].items():
                data_overall['obs'][k].append(data_latest['obs'][k])
                while len(data_overall['obs'][k]) > len_history:
                    data_overall['obs'][k].pop(0)


    # start_step = 27
    # for k, v in data_h5['obs'].items():
    #     data_h5['obs'][k] = v[start_step:]
    # data_h5['actions'] = data_h5['actions'][start_step:]
    # data_h5['actions_abs'] = data_h5['actions_abs'][start_step:]

    # plot_actions(data_h5['actions'])

    data_h5_file = h5py.File(output_file, 'w')
    data_h5_file.create_dataset('actions', data=data_h5['actions'])
    data_h5_file.create_group('obs')
    for k, v in data_h5['obs'].items():
        data = np.array(v)
        data = np.expand_dims(data, axis=1)
        data_h5_file.create_dataset(f'obs/{k}', data=data)
    data_h5_file.close()




def h5_tree(val, pre=''):
    items = len(val)
    for key, val in val.items():
        items -= 1
        if items == 0:
            # the last item
            if type(val) == h5py._hl.group.Group:
                print(pre + '└── ' + key)
                h5_tree(val, pre+'    ')
            else:
                try:
                    print(pre + '└── ' + key + ' %s' % str(val.shape) + ' (%s)' % str(val.dtype))
                except TypeError:
                    print(pre + '└── ' + key + ' (scalar)')
        else:
            if type(val) == h5py._hl.group.Group:
                print(pre + '├── ' + key)
                h5_tree(val, pre+'│   ')
            else:
                try:
                    print(pre + '├── ' + key + ' %s' % str(val.shape) + ' (%s)' % str(val.dtype))
                except TypeError:
                    print(pre + '├── ' + key + ' (scalar)')

def plot_actions(actions):
    import matplotlib.pyplot as plt
    import numpy as np
    actions = np.array(actions)
    plt.plot(actions[:, 0], label='x')
    plt.plot(actions[:, 1], label='y')
    plt.plot(actions[:, 2], label='z')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    input_dir = sys.argv[1]
    output_dir = sys.argv[2]

    for file in os.listdir(input_dir):
        if file.endswith(".bag"):
            print(f"Processing {input_dir}/{file}")
            bag_h5(f"{input_dir}/{file}", f"{output_dir}/{file}.h5", video_HD=False)
            h5_tree(h5py.File(f"{output_dir}/{file}.h5", 'r'))
            print(f"Converted {file} to {file}.h5")
