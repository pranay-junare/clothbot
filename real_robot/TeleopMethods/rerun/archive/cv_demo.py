import cv2
import rerun as rr
from rtde_receive import RTDEReceiveInterface

rr.init("rerun_example_image", spawn=True)
cap = cv2.VideoCapture(6)

# thunder_ip = "192.168.0.101"
# lightning_ip = "192.168.0.102"
# arms = ["Thunder", "Lightning"]
# ips = [thunder_ip, lightning_ip]
# rtde = RTD


while True:
    ret, frame = cap.read()
    rr.log("image", rr.Image(frame))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close the window
cap.release()
cv2.destroyAllWindows()