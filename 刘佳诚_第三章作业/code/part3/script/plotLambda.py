#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

# Data for plotting
data = np.loadtxt("/home/null/Documents/VIO-cource/刘佳诚_第三章作业/code/part3/build/lambda_set.txt")
data = data.T
fig, ax = plt.subplots()
ax.plot(data[0], data[1])

ax.set(xlabel='iteration', ylabel='lambda',
       title='lambda change by iterations')
ax.grid()

##fig.savefig("../../lambda.png")
plt.show()
