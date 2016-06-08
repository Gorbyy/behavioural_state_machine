#! /usr/bin/env python

import numpy as np # to work with numerical data efficiently
import matplotlib.pyplot as plt

amplitude = 200
limitmax = amplitude/2
limitmin = -amplitude/2

position = 50




fs = 500 # sample rate 
f = 3 # the frequency of the signal

x = np.arange(fs) # the points on the x axis for plotting
# compute the value (amplitude) of the sin wave at the for each sample
y = np.sin(2*np.pi*f * x / fs)*(amplitude/2) + position

for i in range(len(y)):
	if y[i] > limitmax:
		y[i] = limitmax
	elif y[i] < limitmin:
		y[i] = limitmin


plt.plot(x, y)
plt.plot(x, -y)
plt.xlabel('sample(n)')
plt.ylabel('angle')
plt.show()