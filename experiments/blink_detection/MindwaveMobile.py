from DataPointReader import DataPointReader
## SAVE DATA TO FILE ###--for debugging
import time
import numpy as np
import sys
import os
np.set_printoptions(threshold=sys.maxsize)


if __name__ == '__main__':
    mindwaveDataPointReader = DataPointReader()
    mindwaveDataPointReader.start()
    blinkcounter = []
    eyeclose = 0
    eyeopen = 0
    blink = 0
    ready = 0 ## Recognize whether the signal strenght is good enough to start the data reading

    ## SAVE DATA TO FILE ###--for debugging
    datapointsList = []
    blinksList = []
    t_end = time.time() + 30
    while(time.time() < t_end):
    # while(True):
        dataPoint = mindwaveDataPointReader.readNextDataPoint()
        ############## ATTENTION ##############
        ## Determine when the attention of the user is high enough for the robot to start driving
        if(dataPoint != None and dataPoint[0] == 'p' and int(dataPoint[1]) == 0):
            ready = 1

        if(ready == 1):
            print("START")

            ############## BLINKING ##############
            ## Determine when the user blinks and how often the user blinks
            if(dataPoint != None and dataPoint[0] == 'b'):
                datapointsList.append(str(dataPoint[1])) #--for debugging
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
                    blinksList.append(blink)
                    blink = 0
                    blinkcounter.clear()

    print("STOP")
    print(str(blinksList))
    ## SAVE DATA TO FILE ###--for debugging

    f = open("participant.txt", "w")
    f.write(str(datapointsList))
    f.close()
    
    with open("participant.txt", "r") as infile, open("EEG_participant.txt", "w") as outfile:
        datalist = infile.read()
        datalist = datalist.replace("'", "")
        datalist = datalist.replace(" ", "")
        datalist = datalist.replace("[", "")
        datalist = datalist.replace("]", "")
        outfile.write(datalist)
        infile.close()
        outfile.close()
        os.remove("participant.txt")
    
