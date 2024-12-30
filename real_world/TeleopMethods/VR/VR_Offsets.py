import triad_openvr
import os
import pickle
import time


def main():
    vr = triad_openvr.triad_openvr()
    print("Connected to VR")
    current_dir = os.path.dirname(os.path.realpath(__file__))

    while(True):
        for controller in ("controller_1", "controller_2"):
            if controller in vr.devices:
                    devices = vr.devices[controller]
                    buttons = devices.get_controller_inputs()
                    pos = devices.get_pose_euler()
                    # print(pos)
                    print(buttons)
                    if buttons['trigger'] == 1.0:
                        print(f"Trigger pressed on {controller}")
                        pos_xyz = pos[:3]
                        pos_rpy = [angle*3.14159/180 for angle in pos[3:]]
                        pickle.dump(pos_xyz+pos_rpy, open(f"{current_dir}/VR_offsets.pkl", "wb"))
                        print(f"Position: {pos_xyz}, Rotation: {pos_rpy}")
                        print(f"Saved {controller}.pkl")
                        exit()
                    # time.sleep(1)
                    break

                        

if __name__ == '__main__':
    main()