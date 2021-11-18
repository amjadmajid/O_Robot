class DataPointParser:
    def __init__(self, payloadBytes):
        self._payloadBytes = payloadBytes
        self._payloadIndex = 0
    


    ### PARSING OF DATA POINTS ###
    def parseDataPoints(self):
        dataPoints = []
        ## Stop when the index is at the end of the payload bytes buffer
        while (not (self._payloadIndex == len(self._payloadBytes))):
            dataPoint = self._parseSingleDataPoint()
            dataPoints.append(dataPoint)
        return dataPoints
    


    ### PARSING OF A SINGLE DATA POINT ###
    def _parseSingleDataPoint(self):
        ## Get the row code and ignore the extended code bytes (0x55)
        byte = self._payloadBytes[self._payloadIndex] #Get the next byte
        self._payloadIndex += 1 ## Increment the payloadIndex with 1 when the next byte is used
        while (byte == 0x55):
            byte = self._payloadBytes[self._payloadIndex]
            self._payloadIndex += 1
        dataRowCode = byte

        ## Get the length of the value bytes
        dataRowHasLengthByte = dataRowCode > 0x7f
        if (dataRowHasLengthByte):
            amountOfBytes = self._payloadBytes[self._payloadIndex]
            self._payloadIndex += 1
        else:
            amountOfBytes = 1

        ## Get the valueBytes of the data row
        dataRowValueBytes = self._payloadBytes[self._payloadIndex : self._payloadIndex + amountOfBytes]
        self._payloadIndex += amountOfBytes

        #### WHAT INFORMATION DOES THE DATA POINT CARRY? ####
        if (dataRowCode == 0x02):
            return 'p', dataRowValueBytes[0]
        if (dataRowCode == 0x04): ## Get the attention value from the Mindwave Mobile data point
            return 'a', dataRowValueBytes[0]
        if (dataRowCode == 0x80): ## Get the (raw) data points from the Mindwave Mobile to later determine whether the user blinked
            self.blinkValue = dataRowValueBytes[0] * 256 + dataRowValueBytes[1]
            if self.blinkValue >= 32768:
                self.blinkValue -= 65536
            return 'b', self.blinkValue