import socket
import sys
import curses
from threading import Thread

from DataPointReader import DataPointReader
### SAVE DATA TO FILE ###--for debugging
# import time
# import numpy as np
# import sys
# np.set_printoptions(threshold=sys.maxsize)

HOST = '192.168.1.129'
PORT = 1234
ROBOT_COUNT = 2
conns = []  
count = 0

def main(scr):
    scr.keypad(True)
    curses.use_default_colors()
    curses.noecho()
    curses.curs_set(0)
    # curses.cbreak()
    scr.nodelay(1)
    scr.refresh()
    
    ## Get screen width/height
    height,width = scr.getmaxyx()

    ## To keep the information of both robots ordered in the terminal 
    padcount = ROBOT_COUNT + 1
    pads = []
    for i in range(padcount):
        pads.append(curses.newwin(height, width//padcount, 0, i * width//padcount))
        pads[i].scrollok(True)
        pads[i].refresh()

    scr.refresh()

    ## Create a socket seerver, bind the socket and start listening to the data on the socket
    s = socket.socket(ROBOT_COUNT)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    pads[0].addstr('Socket server created\n')
    try:
        s.bind((HOST, PORT))
    except socket.error as err:
        pads[0].addstr(err)
        sys.exit()
    pads[0].addstr('Socket server bind complete\n')
    s.listen(0)
    pads[0].addstr('Socket server now listening\n')

    pads[0].refresh()
    scr.refresh()

    dataPointReader = DataPointReader()
    dataPointReader.start() ## Start the Mindwave Mobile V1 and start reading data points
    blinkcounter = [] ## Keeps track of the datapoints needed to determine the amount of blinks
    eyeclose = 0 ## Detect whether the eyes closed
    eyeopen = 0 ## Detect whether the eyes opened after closing
    blink = 0 ## Count the number of blinks
    ready = 0 ## Recognize whether the signal strength is good enough to start the data reading

    def recv(threadname, conn, pad):
        while True:
            data = conn.recv(1024).decode('utf-8')
            pad.addstr(data)
            # pad.refresh()

    def listen_to_clients(threadname):
        global count
        global conns
        while True:
            conn, addr = s.accept()
            conns.append((conn, addr))
            pads[0].addstr('Connection from: ' + addr[0] + ':' + str(addr[1]) + '\n')
            Thread(target=recv, args=('thread ' + str(count), conn, pads[count + 1])).start()
            count += 1
            # pads[0].refresh()

    ### SAVE DATA TO FILE ###--for debugging
    # datapointsList = []
    # t_end = time.time() + 30
    # while(time.time() < t_end):

    Thread(target=listen_to_clients, args=("listen_to_clients", )).start()
    
    while(True):
        dataPoint = dataPointReader.readNextDataPoint()
        ############## ATTENTION ##############
        ## Determine when the attention of the user is high enough for the robot to start driving
        if(dataPoint != None and dataPoint[0] == 'p' and int(dataPoint[1]) == 0):
            ready = 1

        if(ready == 1):
            if(dataPoint != None and dataPoint[0] == 'a'):
                if(int(dataPoint[1]) >= 60):
                    pads[0].addstr("Level: " + str(int(dataPoint[1])) + " GO! - 1\n")
                    # pads[0].addstr(b'1')
                    for i in range(count):
                        (conn, addr) = conns[i]
                        conn.sendto(b'1', (addr[0], addr[1]))
                else:
                    pads[0].addstr("Level: " + str(int(dataPoint[1])) + " STOP - 0\n")
                    # pads[0].addstr(b'0')
                    for i in range(count):
                        (conn, addr) = conns[i]
                        conn.sendto(b'0', (addr[0], addr[1]))

            ############## BLINKING ##############
            ## Determine when the user blinks and how often the user blinks
            if(dataPoint != None and dataPoint[0] == 'b'):
                # datapointsList.append(str(dataPoint[1])) #--for debugging
                blinkcounter.append(int(dataPoint[1]))
                if(int(dataPoint[1]) > 400 and eyeclose == 0):
                    eyeclose = 1
                if(eyeclose == 1 and int(dataPoint[1]) < -200):
                    eyeopen = 1
                if(eyeclose == 1 and eyeopen == 1):
                    blink += 1
                    eyeclose = 0
                    eyeopen = 0
                if(blink == 0):
                    blinkcounter.clear()
                if(len(blinkcounter) == 500):
                    if(blink == 1):
                        pads[0].addstr("Blinks: " + str(blink) + " TURN RIGHT! - 2\n")
                        # pads[0].addstr(b'2')
                        for i in range(count):
                            (conn, addr) = conns[i]
                            conn.sendto(b'2', (addr[0], addr[1]))
                    elif(blink == 2):
                        pads[0].addstr("Blinks: " + str(blink) + " TURN LEFT! - 3\n")
                        # pads[0].addstr(b'3')
                        for i in range(count):
                            (conn, addr) = conns[i]
                            conn.sendto(b'3', (addr[0], addr[1]))
                    else:
                        pads[0].addstr("Blinks: " + str(blink) + " TURN BACK! - 4\n")
                        # pads[0].addstr(b'4')
                        for i in range(count):
                            (conn, addr) = conns[i]
                            conn.sendto(b'4', (addr[0], addr[1]))
                    blink = 0
                    blinkcounter.clear()
        
        for i in range(padcount):
            pads[i].refresh()

    ### SAVE DATA TO FILE ###--for debugging
    # np_datapointsList = np.asarray(datapointsList).astype(float)
    # # print(np_datapointsList)
    # #normalize all values in array
    # result = (np_datapointsList - np.min(np_datapointsList))/np.ptp(np_datapointsList)
    # # print(result)
    # dataprint = list(result)
    # # print(datapointsList)
    
    # f = open("demofileRAW_v2.txt", "w")
    # f.write(str(dataprint))
    # f.close()

curses.wrapper(main)
