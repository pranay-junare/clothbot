import json
import serial
import time
import pickle

if __name__ == '__main__':
    con = serial.Serial(
        '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
        921600)
    print("Connection established, preparing to read data...")
    time.sleep(0.5) # Ignore bootloader messages
    con.read_all() # Clear buffer
    con.read_until(b'\x00')[:-1] 


    data = con.read_until(b'\x00')[:-1]
    data = json.loads(data.decode('utf-8'))
    raw_angles = data['values'][:7]
    print(raw_angles)
    pickle.dump(raw_angles, open("offsets_thunder.pickle", "wb"))
    print("Done")
