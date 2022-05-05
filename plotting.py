import time
import numpy as np
import copy
import csv
import matplotlib.pyplot as plt
import json


def shift_plt():
    data_name = 'exp3/exp3_log.csv'
    ns_code = []
    s_code = []

    with open(data_name, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=' ', quotechar='|')

        n=0
        for row in reader:
            #print(', '.join(row))
            #print(row)
            if n >= 1 and n < 101:
                row = row[0].split(',')
                ns_code.append(list(np.float_(row[0:2])))
            elif n >= 101:
                if len(row) == 1:
                    row = row[0].split(',')
                else:
                    row = row[0].split(',')+row[-1].split(',')[1:-1]
                #print(len(row))
                s_code.append(list(np.float_(row[0:2]+row[3:5])))
            n+=1

    #print(ns_code)
    #print(s_code)
    for s in s_code:
        print(s)

    r = []
    theta = []
    color = []
    size = []
    for i in range(0,100):
        r.append(np.hypot(s_code[i][2], s_code[i][3])*1000)
        theta.append(np.arctan2(s_code[i][3], s_code[i][2]))
        size.append(50)
        if s_code[i][1]==0:
            color.append(0.30)
            #color.append(0.25)
        elif s_code[i][1]==1:
            color.append(0.0)
            #color.append(0.125)
        elif s_code[i][1]==2:
            color.append(0.08)
            #color.append(0.125)
        elif s_code[i][1]==3:
            color.append(0.92)
            #color.append(0.75)
        elif s_code[i][1]==4:
            color.append(0.2)
            #color.append(0.125)
        elif s_code[i][1]==5:
            color.append(0.45)
            #color.append(0.5)
        print(r[i], theta[i], color[i])


    print(plt.style.available)
    #plt.style.use('classic')

    fig = plt.figure()
    ax = fig.add_subplot(projection='polar')
    c = ax.scatter(theta, r, c=color, s=size, cmap='hsv', vmin=0, vmax=1, alpha=0.5, label=['hi','data'])
    ax.legend()
    ax.set_rmax(5)
    ax.set_rticks(np.arange(1,6,1))  # Less radial ticks
    ax.set_rlabel_position(0)
    ax.set_xticklabels([])
    ax.set_yticklabels([])

    #x = np.linspace(0, len(sensor_list)-1, len(sensor_list))
    #y = sensor_list
    #yp = [pose_list[i][0:3] for i in range(0,len(pose_list))]
    #yo = [pose_list[i][3:6] for i in range(0,len(pose_list))]

    ## plot
    #fig, ax = plt.subplots(3, 1)

    #ax[0].plot(x, y, linewidth=2.0)
    #ax[1].plot(x, yp, linewidth=2.0)
    #ax[2].plot(x, yo, linewidth=2.0)

    ##ax.set(xlim=(0, 8), xticks=np.arange(1, 8),
    ##       ylim=(0, 8), yticks=np.arange(1, 8))
    #ax[0].set(xlim=(0, 200),ylim=(6000, 7000))
    #ax[1].set(xlim=(0, 200))
    #ax[2].set(xlim=(0, 200))

    plt.show()

def shift_scatter():
    data_name = 'exp1/log.csv'
    ns_code = []
    s_code = []

    with open(data_name, newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=' ', quotechar='|')

        n=0
        for row in reader:
            #print(', '.join(row))
            #print(row)
            if n >= 1 and n < 101:
                row = row[0].split(',')
                ns_code.append(list(np.float_(row[0:2])))
            elif n >= 101:
                if len(row) == 1:
                    row = row[0].split(',')
                else:
                    row = row[0].split(',')+row[-1].split(',')[1:-1]
                #print(len(row))
                s_code.append(list(np.float_(row[0:2]+row[3:5])))
            n+=1

    x = []
    y = []
    color = []
    size = []
    for i in range(0,100):
        x.append(s_code[i][2])
        y.append(s_code[i][3])
        if s_code[i][1]==0:
            #color.append(100*np.pi/180)
            color.append(0.25)
        elif s_code[i][1]==1:
            #color.append(0*np.pi/180)
            color.append(0)
        elif s_code[i][1]==2:
            #color.append(180*np.pi/180)
            color.append(0.5)
        elif s_code[i][1]==3:
            #color.append(45*np.pi/180)
            color.append(0.75)
        elif s_code[i][1]==4:
            #color.append(225*np.pi/180)
            color.append(0.125)

    print(plt.style.available)
    plt.style.use('classic')

    # plot
    fig, ax = plt.subplots()

    ax.scatter(x, y, c=color, vmin=0, vmax=1, alpha=0.75)

    ax.set(xlim=(-0.005, 0.005), xticks=np.arange(-0.005, 0.005,0.001),
           ylim=(-0.005, 0.005), yticks=np.arange(-0.005, 0.005,0.001))

    plt.show()


def f(x, y):
    return np.sin(x) ** 10 + np.cos(10 + y * x) * np.cos(x)

def pctest():
    x = np.linspace(0, 8, 50)
    y = np.linspace(0, 8, 40)

    X, Y = np.meshgrid(x, y)
    Z = f(X, Y)

    fig, ax = plt.subplots(subplot_kw = dict(projection = 'polar'))
    plt.axis('off')
    ax.contourf(X, Y, Z)

    new_axis = fig.add_axes(ax.get_position(), frameon = False)
    new_axis.plot()
    plt.show()