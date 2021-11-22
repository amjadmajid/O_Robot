
import matplotlib.pyplot as plt
import random
import numpy as np
# import scipy as sc
from scipy.interpolate import make_interp_spline

with open("EEG_participant_1.txt", "r") as tf:
    data = tf.read().split(',')

data_x = []
delta = []
theta = []
lowAlpha = []
highAlpha = []
lowBeta = []
highBeta = []
lowGamma = []
midGamma = []

# for i in range(len(data)):
    # data_x.append(i)
# for i in range(len(data)):
#     delta.append(data[i][0])
# print(delta)
# for i in range(len(data)):
#     theta.append(data[i][1])
# print(theta)
# for i in range(len(data)):
#     lowAlpha.append(data[i][2])
# print(lowAlpha)
# for i in range(len(data)):
#     highAlpha.append(data[i][3])
# print(highAlpha)
# for i in range(len(data)):
#     lowBeta.append(data[i][4])
# print(lowBeta)
# for i in range(len(data)):
#     highBeta.append(data[i][5])
# print(highBeta)
# for i in range(len(data)):
#     lowGamma.append(data[i][6])
# print(lowGamma)
# for i in range(len(data)):
#     midGamma.append(data[i][7])
# print(midGamma)

### SMOOTHED ###
# x = np.asarray(data_x)
# y = np.asarray(data)
# # Initialise the subplot function using number of rows and columns
# # figure, axis = plt.subplots(8, 1)
# xnew = np.linspace(x.min(), x.max(), 500)
# spl = make_interp_spline(x, y, k=3)
# y_smooth = spl(xnew)
# plt.plot(xnew, y_smooth)
# plt.show()

### RAW ###
np_datapointsList = np.asarray(data).astype(int)
# normalize all values in array
# result = (np_datapointsList - np.min(np_datapointsList))/np.ptp(np_datapointsList)
# dataprint = list(result)
plt.plot(np_datapointsList, '#2d2d2d', marker='',mfc='pink' ) #plot the data
# plt.xlim(10250,11600)
# plt.ylim(-650,650)
# plt.xticks([]) #set the tick frequency on x-axis
# plt.yticks([])
plt.ylabel('Amplitude') #set the label for y axis
plt.xlabel('Datapoint') #set the label for x-axis
# plt.title("Plotting a list") #set the title of the graph
plt.show() #display the graph

# For Sine Function
# axis[0].plot(delta)
# axis[0].set_title("delta")

# # For Sine Function
# axis[1].plot(theta)
# axis[1].set_title("theta")

# # For Sine Function
# axis[2].plot(lowAlpha)
# axis[2].set_title("lowAlpha")

# # For Sine Function
# axis[3].plot(highAlpha)
# axis[3].set_title("highAlpha")

# # For Sine Function
# axis[4].plot(lowBeta)
# axis[4].set_title("lowBeta")

# # For Sine Function
# axis[5].plot(highBeta)
# axis[5].set_title("highBeta")

# # For Sine Function
# axis[6].plot(lowGamma)
# axis[6].set_title("lowGamma")

# # For Sine Function
# axis[7].plot(midGamma)
# axis[7].set_title("midGamma")
  
# Combine all the operations and display
plt.show()