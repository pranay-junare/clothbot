import pyspacemouse
import time
import os

def main():
    dev = None
    files = os.listdir('/dev/')
    # print(files)
    print("Plug in the SpaceMouse now...", end="", flush=True)
    while dev is None:
        new_files = os.listdir('/dev/')
        print(".", end="", flush=True)
        # print(new_files)
        time.sleep(.5)
        for file in new_files:
            if file not in files:
                if 'hidraw' in file:
                    dev = file
    print("\nDevice found: " + dev)
    
    # success = pyspacemouse.open(dof_callback=pyspacemouse.print_state, button_callback=pyspacemouse.print_buttons, path='/dev/'+dev)
    success = pyspacemouse.open(path='/dev/'+dev)
    if success:
        while 1:
            state = pyspacemouse.read()
            print(state)
            time.sleep(0.001)

if __name__ == '__main__':
    main()