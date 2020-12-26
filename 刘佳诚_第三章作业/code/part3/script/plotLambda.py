#!/usr/bin/env python
# -*- coding: utf-8 -*-
import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

# Data for plottingt
data0 = np.loadtxt("/home/null/Documents/VIO-cource/刘佳诚_第三章作业/code/part3/build/lambda_set.txt")
data1 = np.loadtxt("/home/null/Documents/VIO-cource/刘佳诚_第三章作业/code/part3/build/lambda_set0.txt")
data2 = np.loadtxt("/home/null/Documents/VIO-cource/刘佳诚_第三章作业/code/part3/build/lambda_set1.txt")

data0 = data0.T
data1 = data1.T
data2 = data2.T
# fig, ax = plt.subplots()
# ax.plot(data[0], data[1])

# ax.set(xlabel='iteration', ylabel='lambda',
#        title='lambda change by iterations')
# ax.grid()

# fig.savefig("../../lambda012.png")
# plt.show()


plt.plot(data0[0], data0[1],  color='grey', alpha=0.3,label='original')
plt.plot(data1[0], data1[1],  color='blue', alpha=0.3,label='marquardt')
plt.plot(data2[0], data2[1], color='red', alpha=0.3,label='method1 from paper')

plt.grid(axis='x', color='0.95')
plt.legend(title='algorithms')
plt.title('compare three LM algorithms')
plt.show()