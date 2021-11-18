from ByteReader import ByteReader
from DataPointParser import DataPointParser
import collections

class DataPointReader:
    def __init__(self):
        self._byteReader = ByteReader() ## Initialize MindwaveMobileRawReader where all byte operations take place
        self._dataPointQueue = collections.deque() ## Deque used for quicker append and pop operations which are needed for fast data point reading



    ### CONNECT TO THE MINDWAVE MOBILE OVER BLUETOOTH TO START RECEIVING DATA ###
    def start(self):
        self._byteReader.connectToMindWaveMobile()
    


    ### READ NEXT DATA POINTS FROM THE MINDWAVE MOBILE ###
    def readNextDataPoint(self):
        if (not (len(self._dataPointQueue) > 0)): ## Check whether there are multiple datapoints in the queue
            ## Put the next datapoint in the queue
            dataPoints = self._readDataPointsFromNextPacket()
            self._dataPointQueue.extend(dataPoints)
        return self._dataPointQueue.pop() ## Get next datapoint from the queue
    


    ### GET DATA POINTS IN THE NEXT PACKET ###
    def _readDataPointsFromNextPacket(self):
        ## Go to the start of the next packet
        while(True):
            byte = self._byteReader.getByte()
            if (byte == 0xaa):  ## We need two 0xaa bytes since these indicate the start of a packet
                byte = self._byteReader.getByte()
                if (byte == 0xaa): ## Check whether we have arrived at the start of the packet
                    break

        ## Read the packet
        payloadLength = self._byteReader.getByte() ## Read the payload length
        payloadBytes = self._byteReader.getBytes(payloadLength)
        checkSum = self._byteReader.getByte()
        sumOfPayload = sum(payloadBytes)
        lastEightBits = sumOfPayload % 256
        invertedLastEightBits = ~lastEightBits + 256 ## Compute 1's complement!

        if (not (invertedLastEightBits == checkSum)): ## Check whether checksum is ok
            ## If not, go to the next packet and try again
            print("checksum of packet not correct, discard packet")
            return self._readDataPointsFromNextPacket() 
        else:
            dataPoints = DataPointParser(payloadBytes).parseDataPoints() ## Get the information that the data point carries with it
        self._byteReader.clearAlreadyReadBuffer()
        return dataPoints
        
    
    
    
    
