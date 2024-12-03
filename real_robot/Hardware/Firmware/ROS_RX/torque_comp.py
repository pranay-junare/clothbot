import serial
import time
import rospy
from geometry_msgs.msg import WrenchStamped

class Servo_driver:
    def __init__(self, port):
        self.port = port
        self.ser = serial.Serial(port, 9600)
        self.angle = 0

    def setAngle(self, angle):
        self.angle =  angle + 128
        self.angle = max(min(self.angle, 245), 15)
        print(angle,int(self.angle))
        self.ser.write(bytes([int(self.angle)]))

    def getAngle(self):
        return self.angle
    
    def close(self):
        self.ser.close()


def callback(data):
    global y_torque
    global init
    if not init:
        init = data.wrench.torque.y
    y_torque = init -data.wrench.torque.y

if __name__ == '__main__':
    global y_torque
    global init
    y_torque = 0
    init = False
    # servo = Servo_driver('/dev/ttyUSB4')
    servo = Servo_driver('/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
    # for i in range(-90, 90):
    #     servo.setAngle(i)
    #     time.sleep(0.05)
    # servo.setAngle(0)

    rospy.init_node('servo_driver')
    rospy.Subscriber('/left/wrench', WrenchStamped, callback)
    try:
        while(True):
            servo.setAngle(-y_torque*50)
            time.sleep(0.1)
    except KeyboardInterrupt:
        servo.close()
        exit()