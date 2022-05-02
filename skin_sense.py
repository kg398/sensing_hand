import time
import numpy as np
import nidaqmx
import _thread
import copy
import scipy.optimize as optimize
import csv
import cv2
import json
import matplotlib.pyplot as plt
import random

import kg_robot as kgr

thread_flag = False
tm_thread_flag = False
ft_thread_flag = False
pos_thread_flag = False
skin_thread_flag = False
cam_thread_flag = False
timer_flag = 0

ft_data = []
pos_data = []
skin_data = []

filename = 'video.avi'
frames_per_second = 24.0
res = '720p'

class skin_sense():
    def __init__(self,robot):
        self.robot = robot

    def record(self,name='test',ft=False,pos=True,skin=True,cam=True):
        start_time = int(time.time())
        read_traj(self.robot,name,start_time,ft,pos,skin,cam)
        return '{}_{}_traj.json'.format(name,start_time)

    def play(self, name, ft=False, pos=False, skin=True, cam=True, rp=0, noise=0):
        play_and_read(self.robot, name, ft, pos, skin, cam, rp)
        return

    def generate(self, name, kp=0, noise=0, rp=0):
        #name = 'exp2/test_1650898610_traj.json'
        #kp = [0,30,60,70,74,92,96,108]
        sequence = json.load(open(name))
        label = ''

        if noise==1: # add 2d random gaussian shift
            label = '_shift'
            xshift = random.gauss(0, 0.002)
            yshift = random.gauss(0, 0.002)
            print('xshift: ',xshift)
            print('yshift: ',yshift)
            for i in range(0,len(sequence)):
                sequence[i][0][0] += xshift
                sequence[i][0][1] += yshift
        elif noise==2: # add 2d noise to each keypoint
            label = '_kpnoise'
            for i in range(0,len(kp)):
                sequence[kp[i]][0][0] += random.gauss(0, 0.002)
                sequence[kp[i]][0][1] += random.gauss(0, 0.002)


        j = 0
        for i in range(kp[0]+1,kp[-1]):
            if i == kp[j+1]:
                j+=1
                continue
            lin_start = sequence[kp[j]][0]
            lin_end = sequence[kp[j+1]][0]
            for k in range(0,6):
                sequence[i][0][k] = (kp[j+1]-i)/(kp[j+1]-kp[j])*lin_start[k] + (i-kp[j])/(kp[j+1]-kp[j])*lin_end[k]

        if noise==3:# add 2d random gaussian noise
            label = '_noise'
            for i in range(0,len(sequence)):
                sequence[i][0][0] += random.gauss(0, 0.002)
                sequence[i][0][1] += random.gauss(0, 0.002)

        name = '{}_kp{}_replay{}.json'.format(name[:-5],label,rp)
        open(name, "w").write(json.dumps(sequence))
            
        #x = np.linspace(0, len(sequence)-1, len(sequence))
        #yp = [sequence[i][0][0:3] for i in range(0,len(sequence))]
        #yo = [sequence[i][0][3:6] for i in range(0,len(sequence))]

        ## plot
        #fig, ax = plt.subplots(2, 1)

        #ax[0].plot(x, yp, linewidth=2.0)
        #ax[1].plot(x, yo, linewidth=2.0)
        #plt.show()

        return name



def wait_for_enter():
    global thread_flag
    thread_flag = True
    input()
    thread_flag = False
    return

def read_data(burt,name='test',start_time=0,ft=False,pos=True,skin=True,cam=True):
    global thread_flag
    thread_flag = True
    global ft_data
    ft_data = [0,0,0,0,0,0]
    global pos_data
    pos_data = [0,0,0,0,0,0]
    global skin_data
    skin_data = []

    with open('{}_data.csv'.format(name), 'w', newline='') as csvfile:
        names = ['n']+['t']+['x','y','z','rx','ry','rz']+['fx','fy','fz','tx','ty','tz']+['skin']
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(names)
       
        n=0
        if cam == True:
            global cam_thread_flag
            _thread.start_new_thread(read_cam,(burt,name,))
        if ft == True:
            global ft_thread_flag
            _thread.start_new_thread(read_ft,(burt,start_time,))
        #if pos == True:
        #    global pos_thread_flag
        #    _thread.start_new_thread(read_pos,(burt,start_time,False))
        if skin == True:
            global skin_thread_flag
            _thread.start_new_thread(read_skin,(burt,start_time,))

        tic = time.time()
        while thread_flag == True:
            current_time = time.time()-tic
            if pos == True:
                pos_data = burt.getl()
            csvwriter.writerow([n,current_time]+pos_data+ft_data+skin_data)
            n+=1
            time.sleep(0.1)

        print(n)
        ft_thread_flag = False
        pos_thread_flag = False
        skin_thread_flag = False
        cam_thread_flag = False
        time.sleep(0.5)
        burt.stream_data_stop(wait=False)
        burt.ee_force_stop()
        burt.stream_data_stop()
        burt.ee_force_stop()
        time.sleep(0.1)
        burt.ee_stop_streaming()
        burt.ping()
    return

def read_traj(burt,name='test',start_time=0,ft=False,pos=True,skin=True,cam=True):
    global thread_flag
    thread_flag = True
    global ft_data
    ft_data = [0,0,0,0,0,0]
    global pos_data
    pos_data = [0,0,0,0,0,0]
    global skin_data
    skin_data = []

    input("press enter to start and stop recording")
    _thread.start_new_thread(wait_for_enter,())

    with open('{}_{}_data.csv'.format(name,start_time), 'w', newline='') as csvfile:
        names = ['n']+['t']+['x','y','z','rx','ry','rz']+['fx','fy','fz','tx','ty','tz']+['skin']
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(names)
       
        n=0
        if cam == True:
            global cam_thread_flag
            _thread.start_new_thread(read_cam,(burt,'{}_{}'.format(name,start_time),))
        if ft == True:
            global ft_thread_flag
            _thread.start_new_thread(read_ft,(burt,start_time,))
        if pos == True:
            global pos_thread_flag
            _thread.start_new_thread(read_pos,(burt,start_time,True,))
        if skin == True:
            global skin_thread_flag
            _thread.start_new_thread(read_skin,(burt,start_time,))

        sequence = []
        time.sleep(0.1)
        sequence.append([pos_data,0,0])
        tic = time.time()
        prev_time = 0
        time.sleep(0.1)
        while thread_flag == True:
            current_time = time.time()-tic
            timestep = current_time-prev_time
            csvwriter.writerow([n,current_time]+pos_data+ft_data+skin_data)
            sequence.append([pos_data,timestep,0])
            n+=1
            prev_time = current_time
            time.sleep(0.1)

        print(n)
        ft_thread_flag = False
        pos_thread_flag = False
        skin_thread_flag = False
        cam_thread_flag = False
        sequence.append([pos_data,time.time()-tic,0])
        time.sleep(1)
        open('{}_{}_traj.json'.format(name,start_time), "w").write(json.dumps(sequence))
        burt.socket_send(burt.format_prog(31))
        burt.ee_force_stop()
        burt.stream_data_stop()
        burt.ee_force_stop()
        time.sleep(0.1)
        burt.ee_stop_streaming()
        burt.ping()
    return 

def play_and_read(burt, name, ft=False, pos=True, skin=True, cam=True, rp=0):
    sequence = json.load(open(name))
    print(len(sequence))
    print("average timestep: ",sequence[-1][1]/(len(sequence)-2))
    
    burt.movel(sequence[0][0])

    start_time = int(time.time())
    #name = name.split('.')[0]+'_replay{}'.format(rp)
    name = name.split('.')[0]
    global thread_flag
    _thread.start_new_thread(read_data,(burt,name,start_time,ft,pos,skin,cam))
    
    toc = time.time()
    for i in range(1,len(sequence)-1):
        burt.set_digital_out(0,sequence[i][2])
        burt.servoj(sequence[i][0],control_time=sequence[i][1],lookahead_time=0.008,gain=300)

    burt.stopl(0.5)
    thread_flag = False
    tic = time.time()
    time.sleep(0.5)
    burt.ping()
    #burt.socket_flush()
    print("recorded ",sequence[-1][1],"secs")
    print("executed in ",tic-toc,"secs")
    print("recorded end_pos: ",sequence[-1][0])
    print("actual end_pos:",burt.getl())

def read_ft(burt,start_time):
    global ft_thread_flag
    ft_thread_flag = True
    global ft_data
    ft_data = [0,0,0,0,0,0]

    print('ft data start')

    while ft_thread_flag == True:
        ft_data = burt.ft.read()
        time.sleep(0.01)
    return

def read_pos(burt,start_time,fd):
    global pos_thread_flag
    pos_thread_flag = True
    global pos_data
    pos_data = [0,0,0,0,0,0]

    print('pos data start')
    burt.ping()
    if fd==True:
        burt.socket_send(burt.format_prog(30))
    burt.stream_data_start(0.01)
    while pos_thread_flag == True:
        raw = burt.read_msg()
        if raw != []:
            pos_data = raw
    return

def read_skin(burt,start_time):
    global skin_thread_flag
    skin_thread_flag = True
    global skin_data
    skin_data = []

    print('skin data start')

    burt.ee_start_streaming()
    while skin_thread_flag == True:
        raw = bytes.decode(burt.ee.readline())
        skin_data = []
        #print(raw)

        n = 0
        for item in raw[0:-1].split("\t"):
            if item == 'S':
                n = 1
            elif n == 1:
                #time = float(item)
                n = 0
            else:
                skin_data.append(float(item))
    return

def read_cam(burt,name):
    global cam_thread_flag
    cam_thread_flag = True

    filename = "{}_vid.avi".format(name)
    cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_AUTOFOCUS,0)
    out = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'XVID'), 25, (640, 480))# [1920, 1080])
    ret, frame = cap.read()
    cv2.imwrite('{}_start.jpg'.format(name), frame)

    while cam_thread_flag == True:
        ret, frame = cap.read()
        out.write(frame)

    ret, frame = cap.read()
    cv2.imwrite('{}_end.jpg'.format(name), frame)

    cap.release()
    out.release()
    return


