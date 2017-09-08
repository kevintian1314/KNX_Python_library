
KNX_TELEGRAM_MAX_SIZE = 23
KNX_TELEGRAM_PAYLOAD_MAX_SIZE = 16
CONTROL_FIELD_DEFAULT_VALUE = 188
CONTROL_FIELD_PRIORITY_MASK = 12
ROUTING_FIELD_DEFAULT_VALUE = 225
COMMAND_FIELD_LOW_COMMAND_MASK = 0xC0
COMMAND_FIELD_HIGH_COMMAND_MASK = 0x03
COMMAND_FIELD_LOW_DATA_MASK = 0x3F
ROUTING_FIELD_PAYLOAD_LENGTH_MASK = 15

KNX_TELEGRAM_HEADER_SIZE = 6
KNX_TELEGRAM_LENGTH_OFFSET = 8


class KnxTelegram:
    _telegram = [None] * KNX_TELEGRAM_MAX_SIZE
    _controlField = 0  # 0
    _sourceAddrH = 1  # 1
    _sourceAddrL = 2  # 2
    _targetAddrH = 3  # 3
    _targetAddrL = 4  # 4
    _routing = 5  # 5
    _commandH = 6  # 6
    _commandL = 7  # 7
    # _payloadChecksum = [None] * (KNX_TELEGRAM_PAYLOAD_MAX_SIZE - 1)

    def __init__(self):
        self.clearTelegram()

    def clearTelegram(self):
        for i in range(0, KNX_TELEGRAM_MAX_SIZE):
            self._telegram[i] = 0
            self._telegram[self._controlField] = CONTROL_FIELD_DEFAULT_VALUE
            self._telegram[self._routing] = ROUTING_FIELD_DEFAULT_VALUE

    def getCommand(self):
        return (((self._telegram[self._commandL] & COMMAND_FIELD_LOW_COMMAND_MASK) >> 6) + ((self._telegram[self._commandH] & COMMAND_FIELD_HIGH_COMMAND_MASK) << 2))

    def getPayloadLength(self):
        return (self._telegram[self._routing] & ROUTING_FIELD_PAYLOAD_LENGTH_MASK)

    def getFirstPayloadByte(self):
        return (self._telegram[self._commandL] & COMMAND_FIELD_LOW_DATA_MASK)

    def getLongPayload(self, destination, nbOfBytes):
        if (nbOfBytes > KNX_TELEGRAM_PAYLOAD_MAX_SIZE - 2):
            nbOfBytes = KNX_TELEGRAM_PAYLOAD_MAX_SIZE - 2
        for i in range(8, nbOfBytes + 8):
            destination[i] = self._telegram[i]

    def getTelegramLength(self):
        return KNX_TELEGRAM_LENGTH_OFFSET + self.getPayloadLength()

    def getChecksum(self):
        return self._telegram[8]

    def getSourceAddress(self):
        addr = self._telegram[self._sourceAddrL] + \
            (self._telegram[self._sourceAddrH] << 8)
        return addr

    def getTargetAddress(self):
        addr = self._telegram[self._targetAddrL] + \
            (self._telegram[self._targetAddrH] << 8)
        return addr

    def setTargetAddress(self, addr):
        self._telegram[self._targetAddrL] = addr & 0xFF
        self._telegram[self._targetAddrH] = (addr >> 8) & 0xFF

    def setPayloadLength(self, length):
        self._telegram[self._routing] &= ~ROUTING_FIELD_PAYLOAD_LENGTH_MASK
        self._telegram[self._routing] |= length & ROUTING_FIELD_PAYLOAD_LENGTH_MASK

    def setCommand(self, command):
        self._telegram[self._commandH] &= ~COMMAND_FIELD_HIGH_COMMAND_MASK
        self._telegram[self._commandH] |= (command >> 2)
        self._telegram[self._commandL] &= ~COMMAND_FIELD_LOW_COMMAND_MASK
        self._telegram[self._commandL] |= (command << 6)

    def setSourceAddress(self, addr):
        self._telegram[self._sourceAddrL] = addr & 0xFF
        self._telegram[self._sourceAddrH] = (addr >> 8) & 0xFF

    def setFirstPayloadByte(self, data):
        self._telegram[self._commandL] &= ~COMMAND_FIELD_LOW_DATA_MASK
        self._telegram[self._commandL] |= data & COMMAND_FIELD_LOW_DATA_MASK

    def setLongPayload(self, origin, nbOfBytes):
        if (nbOfBytes > KNX_TELEGRAM_PAYLOAD_MAX_SIZE - 2):
            nbOfBytes = KNX_TELEGRAM_PAYLOAD_MAX_SIZE - 2
        for i in range(0, nbOfBytes):
            self._telegram[i+8] = origin[i]

    def isChecksumCorrect(self):
        checksum = self.getChecksum()
        if checksum == self.calculateChecksum():
            return True
        else:
            return False

    def calculateChecksum(self):
        xorSum = 0
        indexChecksum = KNX_TELEGRAM_HEADER_SIZE + self.getPayloadLength() + 1
        for i in range(0, indexChecksum):
            xorSum ^= self._telegram[i]
        return (~xorSum & 0xFF)

    def updateChecksum(self):
        xorSum = 0
        indexChecksum = KNX_TELEGRAM_HEADER_SIZE + self.getPayloadLength() + 1
        for i in range(0, indexChecksum):
            xorSum ^= self._telegram[i]
        self._telegram[indexChecksum] = ~xorSum & 0xFF

    def copy(self, dest):
        length = self.getTelegramLength()
        for i in range(0, length):
            dest._telegram[i] = self._telegram[i]

    def writeRawByte(self, data, byteIndex):
        self._telegram[byteIndex] = data

    def readRawByte(self, byteIndex):
        return self._telegram[byteIndex]

    def changePriority(self, priority):
        self._telegram[self._controlField] &= ~CONTROL_FIELD_PRIORITY_MASK
        self._telegram[self._controlField] |= priority & CONTROL_FIELD_PRIORITY_MASK

    def clearLongPayload(self):
        for i in range(8, 23):
            self._telegram[i] = 0

    def clearFirstPayloadByte(self):
        self._telegram[self._commandL] &= ~COMMAND_FIELD_LOW_DATA_MASK
