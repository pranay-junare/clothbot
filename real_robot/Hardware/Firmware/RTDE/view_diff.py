import rospy
from std_msgs.msg import Float32MultiArray, Bool
import numpy as np
import matplotlib.pyplot as plt
import queue

q1 = queue.Queue()
q2 = queue.Queue()
q1_mod = 0
q2_mod = 0
mod = 5

def init_plot(name, color):
    plt.ion()  # Turn on interactive mode
    fig, axs = plt.subplots(2, figsize=(10, 8))
    bars1 = axs[0].bar(range(1, 7), [0]*6, color='green')  # Initialize with zeros and green color
    bars2 = axs[1].bar(range(1, 7), [0]*6, color='green')  # Initialize with zeros and green color
    scale = 1.0*np.pi
    for ax in axs:
        ax.set_ylabel('Difference (rad)')
        ax.set_title(name.pop(0))
        ax.set_ylim([-scale, scale])  # Set the limits of y-axis
        ax.axhline(0, color='black')
        ax.set_facecolor(color.pop(0))
    return fig, axs, bars1, bars2

def update_plot(bars, differences):
    for bar, new_height in zip(bars, differences):
        bar.set_height(new_height)
        if abs(new_height) < 0.2:  # If the joint is very close
            bar.set_color('green')
        elif abs(new_height) < 0.5:  # If the joint is somewhat close
            bar.set_color('yellow')
        else:
            bar.set_color('red')

def thunder_callback(data):
    global q1_mod
    q1_mod += 1
    if q1_mod == mod:
        q1.put(data.data[:6])  # Put the data in the queue
        q1_mod = 0

def lightning_callback(data):
    global q2_mod
    q2_mod += 1
    if q2_mod == mod:
        q2.put(data.data[:6])  # Put the data in the queue
        q2_mod = 0

def thunder_en_callback(data):
    print("Thunder enable:", data.data)
    global thunder_color
    if data.data:
        thunder_color = 'lightgreen'
    else:
        thunder_color = 'lightgrey'

def lightning_en_callback(data):
    print("Lightning enable:", data.data)
    global lightning_color
    if data.data:
        lightning_color = 'lightgreen'
    else:
        lightning_color = 'lightgrey'

def main():
    global thunder_color
    global lightning_color
    thunder_color = 'lightgrey'
    lightning_color = 'lightgrey'
    fig, axs, bars1, bars2 = init_plot(["Thunder", "Lightning"], ["lightgrey", "lightgrey"])
    rospy.init_node('Diff')
    rospy.Subscriber("/thunder_diff", Float32MultiArray, thunder_callback)
    rospy.Subscriber("/lightning_diff", Float32MultiArray, lightning_callback)
    rospy.Subscriber("/lightning_enable", Bool, lightning_en_callback)
    rospy.Subscriber("/thunder_enable", Bool, thunder_en_callback)
    while not rospy.is_shutdown():
        if not q1.empty():
            differences = q1.get()  # Get the data from the queue
            update_plot(bars1, differences)
        if not q2.empty():
            differences = q2.get()  # Get the data from the queue
            update_plot(bars2, differences)
        if thunder_color is not None:
            axs[0].set_facecolor(thunder_color)
            thunder_color = None
        if lightning_color is not None:
            axs[1].set_facecolor(lightning_color)
            lightning_color = None
        fig.canvas.draw()
        fig.canvas.flush_events()
        rospy.sleep(0.01)

if __name__ == "__main__":
    main()