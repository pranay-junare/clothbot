import serial
import json
import time

if __name__ == '__main__':
    con = serial.Serial(
        '/dev/ttyACM0',
        921600,
        timeout=10)
    print("Connection established")

    messagePWM = {
        "ID": 0,
        "PWM": [0.0]*6,
    }

    # Send PWM message
    con.write(json.dumps(messagePWM).encode('utf-8')+b'\x00')
    data = con.read_until(b'\x00')[0:-1]
    data = json.loads(data.decode('utf-8'))
    print(data)


    # THUNDER
    # meassageSetInfo = {
    #     'ID': 0,
    #     'SAVE': {
    #         'NAME': "Thunder",
    #         'PWM_ORDER': [
                # 0,
                # 1,
                # 2,
                # 3,
                # 4,
                # 5,
    #         ]
    #     }
    # }

    # LIGHTNING
    meassageSetInfo = {
        'ID': 1,
        'SAVE': {
            'NAME': "Thunder",
            'PWM_ORDER': [
                0,
                1,
                2,
                3,
                4,
                5,
            ]
        }
    }

    con.write(json.dumps(meassageSetInfo).encode('utf-8')+b'\x00')
    data = con.read_until(b'\x00')[0:-1]
    data = json.loads(data.decode('utf-8'))
    print(data)

    names = [
        "Front",
        "Back",
        "Left",
        "Right",
        "Top",
        "Bottom",
    ]
    PWM_pins = [0, 1, 2, 3, 4, 5]
    PWM_order = []

    for i in range(len(names)):
        print(f"Enter a value when the {names[i]} motor is active:")
        for j in range(len(PWM_pins)):
            messagePWM['PWM'] = [0.0]*6
            messagePWM['PWM'][PWM_pins[j]] = 1.0
            con.write(json.dumps(messagePWM).encode('utf-8')+b'\x00')
            data = con.read_until(b'\x00')[0:-1]
            data = json.loads(data.decode('utf-8'))
            print(data)
            active = input("Enter a value when the motor is active:")
            if active is not "":
                PWM_order.append(j)
                break
                
    
    meassageSetInfo['SAVE']['PWM_ORDER']=PWM_order
    con.write(json.dumps(meassageSetInfo).encode('utf-8')+b'\x00')
    data = con.read_until(b'\x00')[0:-1]
    data = json.loads(data.decode('utf-8'))
    print(data)


    con.close()
