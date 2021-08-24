import waypoints as wp
import time
import serial
import numpy as np
import nidaqmx
import _thread
import copy
import scipy.optimize as optimize
import csv

import kg_robot as kgr

def main():
    print("------------Configuring Burt-------------\r\n")
    #burt = kgr.kg_robot()
    burt = kgr.kg_robot(port=30010,ee_port="COM45",db_host="192.168.1.10")
    #burt = kgr.kg_robot(port=30010,ee_port="COM32",db_host="192.168.1.51")
    print("----------------Hi Burt!-----------------\r\n\r\n")

    name = ''
    try:
        while 1:
            ipt = input("cmd: ")

            if ipt == 'demo':
                name = burt.skin.demonstrate(ft=False,pos=True,skin=True,cam=True)
                print(name)
            if ipt == 'rp':
                burt.skin.play(name)

            if ipt == 'rec':
                name = burt.skin.start_recording(ft=False,pos=True,skin=True,cam=True)
                print(name)
            if ipt == 'stop':
                burt.skin.stop_recording()

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

            if ipt == 't':
                burt.teach_mode.record()
            if ipt == 'p':
                burt.teach_mode.play(name)


                


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



