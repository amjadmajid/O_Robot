import bluetooth
import sys

class ByteReader:
    def __init__(self):
        self._buffer = []
        self._bufferPosition = 0
        


    ### CONNECT TO THE MINDWAVE MOBILE OVER BLUETOOTH TO START RECEIVING DATA ###
    def connectToMindWaveMobile(self):
        self.mindwaveMobileSocket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        mindwaveMobileAddress = '20:68:9D:C2:26:9C'
        ## Try to connect to the Mindwave Mobile, if it doesn't work exit the program
        while(True):
            try:
                self.mindwaveMobileSocket.connect((mindwaveMobileAddress, 1))
                return
            except bluetooth.btcommon.BluetoothError as error:
                print("Could not connect to Mindwave Mobile: " + str(error))
                sys.exit() 



    ### GET ONE BYTE FROM THE MINDWAVE MOBILE ###
    def getByte(self):
        if (len(self._buffer) <= self._bufferPosition + 100): ## Make sure that more bytes can be read from the Mindwave Mobile
            newBytes = self._readBytesFromMindwaveMobile(100)
            self._buffer += newBytes ## Add bytes read from the Mindwave Mobile into the buffer
        nextByte = self._buffer[self._bufferPosition] ## Get the next byte in the buffer
        self._bufferPosition += 1 ## Update the buffer position
        return nextByte
    


    ### GET THE BYTES FROM THE MINDWAVE MOBILE FOR THE PAYLOAD BYTES ###
    def getBytes(self, amountOfBytes):
        if (len(self._buffer) <= self._bufferPosition + amountOfBytes): ## Make sure that more bytes can be read from the Mindwave Mobile
            newBytes = self._readBytesFromMindwaveMobile(amountOfBytes)
            self._buffer += newBytes ## Add bytes read from the Mindwave Mobile into the buffer
        nextBytes = list(self._buffer[self._bufferPosition: self._bufferPosition + amountOfBytes]) ## Get the next "amountOfBytes" bytes from the buffer
        self._bufferPosition += amountOfBytes ## Update the buffer position by "amountOfBytes"
        return nextBytes
        


    ### READ AVAILABLE DATA FROM THE MINDWAVE MOBILE ###
    def _readBytesFromMindwaveMobile(self, amountOfBytes):
        missingBytes = amountOfBytes
        receivedBytes = b''
        while(missingBytes > 0):
            receivedBytes += self.mindwaveMobileSocket.recv(missingBytes)
            missingBytes = amountOfBytes - len(receivedBytes)
        return receivedBytes
    


    ### CLEAR THE BUFFER THAT'S ALREADY DEALTH WITH TO MAKE ROOM FOR THE NEXT PACKET ###
    def clearAlreadyReadBuffer(self):
        self._buffer = self._buffer[self._bufferPosition : ]
        self._bufferPosition = 0