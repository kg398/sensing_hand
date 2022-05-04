import waypoints as wp
import time
import serial
import numpy as np
import nidaqmx
import _thread
import copy
import scipy.optimize as optimize
import csv
import cv2
import matplotlib.pyplot as plt
import json

import kg_robot as kgr

def main():
    print("------------Configuring Burt-------------\r\n")
    burt = kgr.kg_robot()
    #burt = kgr.kg_robot(port=30010,ee_port="COM45",db_host="192.168.1.10")
    #burt = kgr.kg_robot(port=30010,db_host="192.168.1.10")
    #burt = kgr.kg_robot(ee_port="COM47")
    #burt = kgr.kg_robot(port=30010,ee_port="COM32",db_host="192.168.1.51")
    print("----------------Hi Burt!-----------------\r\n\r\n")

    name = ''
    rp = 0
    try:
        while 1:
            ipt = input("cmd: ")

            if ipt == 'rs':
                demand_pose = burt.getl()
                demand_pose[2] -= 0.1
                burt.force_move(demand_pose,force=40)
                time.sleep(1)
                burt.translatel_rel([0,0,0.1])

            if ipt == 'flush':
                burt.socket_flush()

            if ipt == 'cam':
                cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
                cap.set(cv2.CAP_PROP_AUTOFOCUS,0)
                ret, frame = cap.read()
                cv2.imwrite('_start.jpg', frame)

            if ipt == 't':
                name = burt.skin.record(ft=False,pos=True,skin=True,cam=True)
                print(name)
            if ipt == 'p':
                if name=='':
                    #name = input('name: ')
                    #name = 'exp1/test_1650898610_traj_kp.json'
                    name = 'exp1/test_1650898610_traj.json'
                    #name = 'exp2/test_1650898610_traj.json'
                    #name = 'exp3/test_1651576480_traj.json'
                if rp == 0:
                    rp = int(input('replay no.: '))
                for i in range(0,int(input('iter: '))):
                    print('iter ',i)
                    kp_name = burt.skin.generate(name,[0,30,60,70,74,92,96,108],noise=1,rp=rp)
                    #kp_name = burt.skin.generate(name,[0,30,62,66,74,83,92,107],noise=1,rp=rp)

                    #sequence = json.load(open(kp_name))
                    #x = np.linspace(0, len(sequence)-1, len(sequence))
                    #yp = [sequence[i][0][0:3] for i in range(0,len(sequence))]
                    #yo = [sequence[i][0][3:6] for i in range(0,len(sequence))]

                    ## plot
                    #fig, ax = plt.subplots(2, 1)

                    #ax[0].plot(x, yp, linewidth=2.0)
                    #ax[1].plot(x, yo, linewidth=2.0)
                    #plt.show()

                    burt.skin.play(kp_name,ft=False,pos=True,skin=True,cam=True,rp=rp)
                    rp+=1
                    time.sleep(13)

            if ipt == 'xy':
                base_seq = json.load(open('exp3/test_1651576480_traj_kp_replay0.json'))
                xshift = []
                yshift = []
                for i in range(0,100):
                    seq = json.load(open('exp3/test_1651576480_traj_kp_shift_replay{}.json'.format(100+i)))
                    xshift.append(seq[0][0][0]-base_seq[0][0][0])
                    yshift.append(seq[0][0][1]-base_seq[0][0][1])

                with open('exp3/shifts.csv', 'w', newline='') as csvfile:
                    names = ['n','x','y']
                    csvwriter = csv.writer(csvfile)
                    csvwriter.writerow(names)
                    for i in range(0,100):
                        csvwriter.writerow([i+100,xshift[i],yshift[i]])

            if ipt == 'kpg':
                name = burt.skin.generate('exp2/test_1650898610_traj.json',[0,30,60,70,74,92,96,108],noise=3)
                #name = 'exp1/test_1650898610_traj.json'
                #kp = [0,30,60,70,74,92,96,108]
                #sequence = json.load(open(name))
                #j = 0
                #for i in range(kp[0]+1,kp[-1]):
                #    if i == kp[j+1]:
                #        j+=1
                #        continue
                #    lin_start = sequence[kp[j]][0]
                #    lin_end = sequence[kp[j+1]][0]
                #    for k in range(0,6):
                #        sequence[i][0][k] = (kp[j+1]-i)/(kp[j+1]-kp[j])*lin_start[k] + (i-kp[j])/(kp[j+1]-kp[j])*lin_end[k]

                #open('{}_kp.json'.format(name[:-5]), "w").write(json.dumps(sequence))
                    
                #x = np.linspace(0, len(sequence)-1, len(sequence))
                #yp = [sequence[i][0][0:3] for i in range(0,len(sequence))]
                #yo = [sequence[i][0][3:6] for i in range(0,len(sequence))]

                ## plot
                #fig, ax = plt.subplots(2, 1)

                #ax[0].plot(x, yp, linewidth=2.0)
                #ax[1].plot(x, yo, linewidth=2.0)
                #plt.show()

            if ipt == 'ws':
                tic = time.time()
                burt.serial_send('S',1)
                for i in range(0,100):
                    print(burt.ee.readline())
                print(100/(time.time()-tic))
                burt.serial_send('S',0)

            if ipt == 'plt':
                print(plt.style.available)
                plt.style.use('classic')

                # make data
                data_name = name[:-5]+'data.csv'
                print(name)
                #data_name = 'exp1/test_1650898610_data.csv'
                #data_name = 'exp1/test_1650898610_traj_kp_replay0_data.csv'
                data_name = 'exp3/test_1651576480_data.csv'

                pose_list = []
                sensor_list = []
                n = 0
                with open(data_name, newline='') as csvfile:
                    reader = csv.reader(csvfile, delimiter=' ', quotechar='|')
                    for row in reader:
                        #print(', '.join(row))
                        if n!= 0:
                            row = row[0].split(',')
                            if len(list(np.float_(row[14:46]))) != 0:
                                pose_list.append(list(np.float_(row[2:8])))
                                sensor_list.append(list(np.float_(row[14:46])))
                        else:
                           n+=1

                x = np.linspace(0, len(sensor_list)-1, len(sensor_list))
                y = sensor_list
                yp = [pose_list[i][0:3] for i in range(0,len(pose_list))]
                yo = [pose_list[i][3:6] for i in range(0,len(pose_list))]

                # plot
                fig, ax = plt.subplots(3, 1)

                ax[0].plot(x, y, linewidth=2.0)
                ax[1].plot(x, yp, linewidth=2.0)
                ax[2].plot(x, yo, linewidth=2.0)

                #ax.set(xlim=(0, 8), xticks=np.arange(1, 8),
                #       ylim=(0, 8), yticks=np.arange(1, 8))
                ax[0].set(xlim=(0, 200),ylim=(6000, 7000))
                ax[1].set(xlim=(0, 200))
                ax[2].set(xlim=(0, 200))

                plt.show()


            if ipt == 's0':
                #burt.ee_stop_streaming()
                burt.serial_send('S',0)
            if ipt == 's1':
                #burt.ee_start_streaming()
                burt.serial_send('S',1)
            if ipt == 'cl':
                burt.ee.reset_input_buffer()
            if ipt == 'r':
                print(bytes.decode(burt.ee.readline()))

            if ipt == 'c':
                with open('test.csv', 'w', newline='') as csvfile:
                    names = ['test1','test2','test3']
                    csvwriter = csv.writer(csvfile)
                    csvwriter.writerow(names)
                    for row in range(1,10):
                        csvwriter.writerow([row,row+1,row+2])

            #if ipt == 't':
            #    burt.teach_mode.record()
            #if ipt == 'p':
            #    burt.teach_mode.play(name)


                


            if ipt == 'close':
                break
            elif ipt == 'home':
                burt.home()

            
            #else:
            #    var = int(input("var: "))
            #    burt.serial_send(ipt,var,True)

        
    finally:
        print("Goodbye")
        burt.close()
if __name__ == '__main__': main()



