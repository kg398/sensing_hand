import time
import numpy as np
import nidaqmx
import _thread
import copy
import scipy.optimize as optimize
import csv
import cv2
import json

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

    def demonstrate(self,name='test',ft=True,pos=True,skin=True,cam=True):
        start_time = int(time.time())
        read_data(self.robot,name,start_time,ft,pos,skin,cam)
        return '{}_{}_traj.json'.format(name,start_time)

    def play(self, name, cam=True, rp=0):
        sequence = json.load(open(name))
        print(len(sequence))
        print("average timestep: ",sequence[-1][1]/(len(sequence)-2))

        if cam == True:
            start_time = int(time.time())
            name = name.split('.')[0]+'_replay{}'.format(rp)
            global cam_thread_flag
            _thread.start_new_thread(record_cam,(self.robot,name,start_time,))

        self.robot.movel(sequence[0][0])
        toc = time.time()
        for i in range(1,len(sequence)-1):
            self.robot.set_digital_out(0,sequence[i][2])
            self.robot.servoj(sequence[i][0],control_time=sequence[i][1],lookahead_time=0.008,gain=300)

        self.robot.stopl(0.5)
        cam_thread_flag = False
        tic = time.time()
        self.robot.socket_ping()
        print("recorded ",sequence[-1][1],"secs")
        print("executed in ",tic-toc,"secs")
        print("recorded end_pos: ",sequence[-1][0])
        print("actual end_pos:",self.robot.getl())



    def start_recording(self,name='test',ft=True,pos=True,skin=True,cam=True):
        global thread_flag
        start_time = int(time.time())
        _thread.start_new_thread(read_data,(self.robot,name,start_time,ft,pos,skin,cam))
        return '{}_{}_traj.json'.format(name,start_time)

    def stop_recording(self):
        global thread_flag
        thread_flag = False
        return


def wait_for_enter():
    global thread_flag
    thread_flag = True
    input()
    thread_flag = False
    return

def read_data(burt,name='test',start_time=0,ft=True,pos=True,skin=True,cam=True):
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
            _thread.start_new_thread(record_cam,(burt,name,start_time,))
        if ft == True:
            global ft_thread_flag
            _thread.start_new_thread(read_ft,(burt,start_time,))
        if pos == True:
            global pos_thread_flag
            _thread.start_new_thread(read_pos,(burt,start_time,))
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
        time.sleep(0.5)
        open('{}_{}_traj.json'.format(name,start_time), "w").write(json.dumps(sequence))
        burt.socket_send(burt.format_prog(31))
        burt.stream_data_stop()
        burt.ee_stop_streaming()
        burt.ping()
    return 

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

def read_pos(burt,start_time):
    global pos_thread_flag
    pos_thread_flag = True
    global pos_data
    pos_data = [0,0,0,0,0,0]

    print('pos data start')
    burt.ping()
    burt.socket_send(burt.format_prog(30))
    burt.stream_data_start(0.01)
    while pos_thread_flag == True:
        pos_data = burt.read_msg()
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
                time = float(item)
                n = 0
            else:
                skin_data.append(float(item))
    return

def record_cam(burt,name,start_time):
    global cam_thread_flag
    cam_thread_flag = True

    filename = "{}_{}_vid.avi".format(name,start_time)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_AUTOFOCUS,0)
    out = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'XVID'), 25, (640, 480))# [1920, 1080])
    ret, frame = cap.read()
    cv2.imwrite('{}_{}_start.jpg'.format(name,start_time), frame)

    while cam_thread_flag == True:
        ret, frame = cap.read()
        out.write(frame)

    ret, frame = cap.read()
    cv2.imwrite('{}_{}_end.jpg'.format(name,start_time), frame)

    cap.release()
    out.release()
    return
