import json
import serial
import time
import pickle
import os

if __name__ == '__main__':
    for i in range(10):
        print(f"{10-i} seconds remaining")
        time.sleep(1)
    con = serial.Serial(
        '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        921600)
    print("Connection established, preparing to read data...")
    con.reset_input_buffer()
    con.reset_output_buffer()
    con.read_until(b'\x00')[:-1] 

    data = con.read_until(b'\x00')[:-1]
    data = json.loads(data.decode('utf-8'))
    ID = data['ID']
    raw_angles = data['values'][:7]
    print(f"ID: {ID}")
    print(f"Raw angles: {raw_angles}")
    if ID == 'thunder':
        inverted = [-1, -1, 1, -1, -1, -1, 1]
    elif ID == 'lightning':
        inverted = [-1, -1, 1, -1, -1, -1, -1]
    print((raw_angles, inverted))
    path = os.path.dirname(os.path.abspath(__file__))
    pickle_path = os.path.join(path, "offsets_"+ID+".pickle")
    pickle.dump((raw_angles, inverted), open(pickle_path, "wb"))
    print("Done")