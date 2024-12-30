#https://makersportal.com/blog/2018/8/14/real-time-graphing-in-python

import numpy as np
import matplotlib.pyplot as plt

class LivePlot:
    def __init__(self, num_points):
        self.num_points = num_points
        self.lines = []
        plt.style.use('ggplot')
        self.size = 200
        self.x_vec = np.linspace(-self.size/1000,0,self.size+1)[0:-1]
        self.y_vec = np.zeros((len(self.x_vec),self.num_points))

        
    def live_plotter(self, x_vec,y_data):
        if self.lines==[]:
            self.lines = [None]*y_data.shape[1]
            plt.ion()
            fig = plt.figure(figsize=(13,6))
            # fig.canvas.mpl_connect('close_event', lambda evt: exit(0))
            ax = fig.add_subplot(111)
            for i in range(y_data.shape[1]):
                self.lines[i] = ax.plot(x_vec,y_data[:,i],'-o',alpha=0.8)[0]
                self.lines[i].axes.set_ylim(-2*np.pi, 2*np.pi) 
            self.lines.append(fig)
            plt.ylabel('Rotation (-2Pi to 2Pi)')
            plt.title('Arm State')
            plt.show()
        # Check if plot has been closed
        if self.lines[-1].canvas.manager.window is None:
            exit(0)
            
        for i in range(y_data.shape[1]):
            self.lines[i].set_ydata(y_data[:,i])
        self.lines[-1].canvas.flush_events()

    def update(self, data):
        self.y_vec[-1] = data
        self.live_plotter(self.x_vec, self.y_vec)
        self.y_vec = np.append(self.y_vec[1:],np.zeros([1,self.num_points]),axis=0)