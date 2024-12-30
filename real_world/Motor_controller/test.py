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
        "ID": None,
        "PWM": [0.0]*6,
    }

    messageInfo = {
        "ID": None,
        "Info": "GET",
    }
    messageNo = 0

    messageInfo["ID"] = messageNo
    messageNo += 1
    con.write(json.dumps(messageInfo).encode('utf-8')+b'\x00')
    data = con.read_until(b'\x00')[0:-1]
    data = json.loads(data.decode('utf-8'))
    print(data)
    if "ERROR" in data:
        print("ERROR")
        print(data["ERROR"])
        
    # Send PWM message

    for i in range(6):
        messagePWM["PWM"] = [0.0]*6
        messagePWM["PWM"][i] = 1.0
        con.write(json.dumps(messagePWM).encode('utf-8')+b'\x00')
        data = con.read_until(b'\x00')[0:-1]
        data = json.loads(data.decode('utf-8'))
        print(data)
        time.sleep(2)

    messagePWM["PWM"] = [0.0]*6
    con.write(json.dumps(messagePWM).encode('utf-8')+b'\x00')
    data = con.read_until(b'\x00')[0:-1]
    data = json.loads(data.decode('utf-8'))
    print(data)


    con.close()
