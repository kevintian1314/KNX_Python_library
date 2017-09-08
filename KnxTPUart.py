"""
This module is the bottom layer for the Knx system, it handles reception and transmissions of the telegrams through the TP UART, by using serial transmissions. 
It notifies the upper layer (KnxDevice) of any detected events on the Knx Bus. 
"""

import serial
import monotonic
import KnxComObject
import KnxTelegram

RX_RESET = 0
RX_STOPPED = 1
RX_INIT = 2
RX_IDLE_WAITING_FOR_CTRL_FIELD = 3
RX_EIB_TELEGRAM_RECEPTION_STARTED = 4
RX_EIB_TELEGRAM_RECEPTION_ADDRESSED = 5
RX_EIB_TELEGRAM_RECEPTION_LENGTH_INVALID = 6
RX_EIB_TELEGRAM_RECEPTION_NOT_ADDRESSED = 7

TX_RESET = 0
TX_STOPPED = 1
TX_INIT = 2
TX_IDLE = 3
TX_TELEGRAM_SENDING_ONGOING = 4
TX_WAITING_ACK = 5

KNX_TPUART_OK = 0
KNX_TPUART_ERROR_NULL_ACK_CALLBACK_FCT = 252
KNX_TPUART_ERROR_NULL_EVT_CALLBACK_FCT = 253
KNX_TPUART_ERROR_NOT_INIT_STATE = 254
KNX_TPUART_ERROR = 255

ACK_RESPONSE = 0

BUS_MONITOR = 1

TPUART_SET_ADDR_REQ = 0x28

# TP UART events
TPUART_EVENT_RESET = 0
TPUART_EVENT_RECEIVED_EIB_TELEGRAM = 1
TPUART_EVENT_EIB_TELEGRAM_RECEPTION_ERROR = 2
TPUART_EVENT_STATE_INDICATION = 3

NACK_RESPONSE = 1
NO_ANSWER_TIMEOUT = 2
TPUART_RESET_RESPONSE = 3

# Services to TPUART (hostcontroller -> TPUART) :
TPUART_RESET_REQ = [0x01]
TPUART_STATE_REQ = [0x02]
TPUART_ACTIVATEBUSMON_REQ = [0x05]
TPUART_DATA_CONFIRM_SUCCESS = [0x8B]
TPUART_RESET_INDICATION = [0x03]
TPUART_STATE_INDICATION = [0x07]
TPUART_DATA_CONFIRM_FAILED = [0x0B]
TPUART_RX_ACK_SERVICE_NOT_ADDRESSED = [0x10]
TPUART_RX_ACK_SERVICE_ADDRESSED = [0x11]
TPUART_DATA_END_REQ = [0x40]
TPUART_DATA_START_CONTINUE_REQ = [0x80]

TPUART_STATE_INDICATION_MASK = 0x07

EIB_CONTROL_FIELD_PATTERN_MASK = 211
EIB_CONTROL_FIELD_VALID_PATTERN = 144

KNX_TELEGRAM_MAX_SIZE = 23

_telegram = KnxTelegram.KnxTelegram()
_addressedComObjectIndex = None
_lastByteRxTimeMicrosec = 0
_readBytesNb = 0
_lastByteRxTimeMicrosec = 0

_nowTime = 0
_txByte = [None] * 2
_sentMessageTimeMillisec = 0


class tx:
    state = None
    sentTelegram = KnxTelegram.KnxTelegram()
    ackFctPtr = None
    nbRemainingBytes = None
    txByteIndex = None


class rx:
    state = None
    receivedTelegram = KnxTelegram.KnxTelegram()
    addressedComObjectIndex = None


class KnxTPUart:
    _serial = None
    _physicalAddr = None
    _mode = None
    _comObjectsList = None
    _assignedComObjectsNb = None
    _tx = tx()
    _rx = rx()
    _evtCallbackFct = None
    _orderedIndexTable = None
    _stateIndication = None

    def __init__(self, cserial, physicalAddr, mode):
        """
        Constructor for the KnxTPUart class. This class handles all the serial transmissions going through the Knx bus. 
        """
        self._serial = serial.Serial()
        self._serial.port = cserial
        self._serial.baudrate = 19200
        self._serial.parity = serial.PARITY_EVEN
        self._serial.stopbits = serial.STOPBITS_ONE
        self._serial.bytesize = serial.EIGHTBITS
        self._serial.timeout = 1

        self._physicalAddr = physicalAddr
        self._mode = mode
        self._rx.state = RX_RESET
        self._rx.addressedComObjectIndex = 0
        self._tx.state = TX_RESET
        self._tx.sentTelegram = None
        self._tx.ackFctPtr = None
        self._tx.nbRemainingBytes = 0
        self._tx.txByteIndex = 0
        self._stateIndication = 0
        self._evtCallbackFct = None
        self._comObjectsList = None
        self._assignedComObjectsNb = 0
        self._orderedIndexTable = None

    def reset(self):
        """
        This function initiates a reset request to the TP UART device. It checks if it's active.
        Returns KNX_TPUART_ERROR in case of failure, KNX_TPUART_OK otherwise
        """
        attempts = 10

        if (self._rx.state > RX_RESET or self._tx.state > TX_RESET):
            self._serial.close()
            self._rx.state = RX_RESET
            self._tx.state = TX_RESET

        self._serial.open()

        while (attempts > 0):
            attempts -= 1
            self._serial.write(bytearray(TPUART_RESET_REQ))

            nowTime = startTime = millis()
            while (nowTime - startTime < 1000):
                nowTime = millis()

                if (self._serial.inWaiting() > 0):
                    c = self._serial.read()
                    for ch in c:
                        if (ord(c) == TPUART_RESET_INDICATION[0]):
                            self._rx.state = RX_INIT
                            self._tx.state = TX_INIT
                            return KNX_TPUART_OK

        self._serial.close()
        return KNX_TPUART_ERROR

    def init(self):
        """
        This function initializes the connection to the Knx bus through the TP UART port. Returns error if the TP Uart is not in INIT state.
        """
        tpuartCmd = [None] * 3

        if ((self._rx.state != RX_INIT) or (self._tx.state != TX_INIT)):
            return KNX_TPUART_ERROR_NOT_INIT_STATE

        # Bus monitoring mode
        if (self._mode == BUS_MONITOR):
            self._serial.write(bytearray(TPUART_ACTIVATEBUSMON_REQ))
        else:  # normal mode
            if (self._evtCallbackFct is None):
                return KNX_TPUART_ERROR_NULL_EVT_CALLBACK_FCT
            if (self._tx.ackFctPtr is None):
                return KNX_TPUART_ERROR_NULL_ACK_CALLBACK_FCT
            tpuartCmd[0] = TPUART_SET_ADDR_REQ
            tpuartCmd[1] = (self._physicalAddr >> 8)
            tpuartCmd[2] = (self._physicalAddr & 0x0F)

            self._serial.write(bytearray(tpuartCmd))

            self._serial.write(bytearray(TPUART_STATE_REQ))

            self._rx.state = RX_IDLE_WAITING_FOR_CTRL_FIELD
            self._tx.state = TX_IDLE

        return KNX_TPUART_OK

    def sendTelegram(self, sentTelegram):
        """
        This function configures the device to prepare for a transmission and save the telegram to be sent in this module.

        Params:
                sentTelegram: the telegram which will be sent in bus.
        """
        if (self._tx.state != TX_IDLE):
            return KNX_TPUART_ERROR
        if (sentTelegram.getSourceAddress() != self._physicalAddr):
            sentTelegram.setSourceAddress(self._physicalAddr)
            sentTelegram.updateChecksum()
        self._tx.sentTelegram = sentTelegram
        self._tx.nbRemainingBytes = sentTelegram.getTelegramLength()
        self._tx.txByteIndex = 0
        self._tx.state = TX_TELEGRAM_SENDING_ONGOING
        return KNX_TPUART_OK

    def RxTask(self):
        """
        Reception task.
        This function handles all the events related to the reception of a message through the TP UART. It notifies the upper layer through the saved callback.
        This should be called periodically to allow telegrams reception.
        """
        global _telegram, _addressedComObjectIndex, _lastByteRxTimeMicrosec, _readBytesNb, _lastByteRxTimeMicrosec
        if (self._rx.state >= RX_EIB_TELEGRAM_RECEPTION_STARTED):
            nowTime = micros()
            if ((nowTime - _lastByteRxTimeMicrosec) > 2000):
                rxState = self._rx.state
                if (rxState == RX_EIB_TELEGRAM_RECEPTION_LENGTH_INVALID):
                    self._evtCallbackFct(
                        TPUART_EVENT_EIB_TELEGRAM_RECEPTION_ERROR)
                elif (rxState == RX_EIB_TELEGRAM_RECEPTION_ADDRESSED):
                    if (_telegram.isChecksumCorrect()):
                        _telegram.copy(self._rx.receivedTelegram)
                        self._rx.addressedComObjectIndex = _addressedComObjectIndex
                        self._evtCallbackFct(
                            TPUART_EVENT_RECEIVED_EIB_TELEGRAM)
                else:
                    self._evtCallbackFct(
                        TPUART_EVENT_EIB_TELEGRAM_RECEPTION_ERROR)

                self._rx.state = RX_IDLE_WAITING_FOR_CTRL_FIELD

        if self._serial.inWaiting() > 0:
            incomingByte = ord(self._serial.read())
            _lastByteRxTimeMicrosec = micros()

            if (self._rx.state == RX_IDLE_WAITING_FOR_CTRL_FIELD):
                if (incomingByte & EIB_CONTROL_FIELD_PATTERN_MASK) == EIB_CONTROL_FIELD_VALID_PATTERN:
                    self._rx.state = RX_EIB_TELEGRAM_RECEPTION_STARTED
                    _readBytesNb = 1
                    _telegram.writeRawByte(incomingByte, 0)

                elif (incomingByte == TPUART_DATA_CONFIRM_SUCCESS[0]):
                    if (self._tx.state == TX_WAITING_ACK):
                        self._tx.ackFctPtr(ACK_RESPONSE)
                        self._tx.state = TX_IDLE

                elif (incomingByte == TPUART_RESET_INDICATION[0]):
                    if ((self._tx.state == TX_TELEGRAM_SENDING_ONGOING) or (self._tx.state == TX_WAITING_ACK)):
                        self._tx.ackFctPtr(TPUART_RESET_RESPONSE)
                    self._tx.state = TX_STOPPED
                    self._rx.state = RX_STOPPED
                    self._evtCallbackFct(TPUART_EVENT_RESET)
                    return

                elif ((incomingByte & TPUART_STATE_INDICATION_MASK) == TPUART_STATE_INDICATION[0]):
                    self._evtCallbackFct(
                        TPUART_EVENT_STATE_INDICATION)
                    self._stateIndication = incomingByte

                elif (incomingByte == TPUART_DATA_CONFIRM_FAILED[0]):
                    if (self._tx.state == TX_WAITING_ACK):
                        self._tx.ackFctPtr(NACK_RESPONSE)
                        self._tx.state = TX_IDLE

            elif (self._rx.state == RX_EIB_TELEGRAM_RECEPTION_STARTED):
                _telegram.writeRawByte(incomingByte, _readBytesNb)
                _readBytesNb += 1

                if (_readBytesNb == 3):
                    if (_telegram.getSourceAddress() == self._physicalAddr):
                        self._rx.state = RX_EIB_TELEGRAM_RECEPTION_NOT_ADDRESSED

                elif (_readBytesNb == 6):
                    if (self.isAddressedAssigned(_telegram.getTargetAddress(), _addressedComObjectIndex)):
                        self._rx.state = RX_EIB_TELEGRAM_RECEPTION_ADDRESSED
                        self._serial.write(
                            bytearray(TPUART_RX_ACK_SERVICE_ADDRESSED))
                    else:
                        self._rx.state = RX_EIB_TELEGRAM_RECEPTION_NOT_ADDRESSED
                        self._serial.write(
                            bytearray(TPUART_RX_ACK_SERVICE_NOT_ADDRESSED))
            elif (self._rx.state == RX_EIB_TELEGRAM_RECEPTION_ADDRESSED):
                if (_readBytesNb == KNX_TELEGRAM_MAX_SIZE):
                    self._rx.state = RX_EIB_TELEGRAM_RECEPTION_LENGTH_INVALID
                else:
                    _telegram.writeRawByte(incomingByte, _readBytesNb)
                    _readBytesNb += 1

    def TxTask(self):
        """
        Transmission task.
        This function handles all the events related to the transmission of a message through the TP UART. It checks the tx state to see if a telegram is waiting to be sent and
        sends it when it does. This should be called periodically to allow telegrams transmission.
        """
        global _nowTime, _txByte, _sentMessageTimeMillisec
        if (self._tx.state == TX_WAITING_ACK):
            _nowTime = millis()
            if (_nowTime - _sentMessageTimeMillisec > 500):
                self._tx.ackFctPtr(NO_ANSWER_TIMEOUT)
                self._tx.state = TX_IDLE
        if (self._tx.state == TX_TELEGRAM_SENDING_ONGOING):
            if (self._rx.state != RX_EIB_TELEGRAM_RECEPTION_STARTED):
                if (self._tx.nbRemainingBytes == 1):
                    _txByte[0] = TPUART_DATA_END_REQ[0] + self._tx.txByteIndex
                    _txByte[1] = self._tx.sentTelegram.readRawByte(
                        self._tx.txByteIndex)

                    self._serial.write(bytearray(_txByte))
                    _sentMessageTimeMillisec = millis()
                    self._tx.state = TX_WAITING_ACK
                else:
                    _txByte[0] = TPUART_DATA_START_CONTINUE_REQ[0] + \
                        self._tx.txByteIndex
                    _txByte[1] = self._tx.sentTelegram.readRawByte(
                        self._tx.txByteIndex)
                    self._serial.write(bytearray(_txByte))
                    self._tx.txByteIndex += 1
                    self._tx.nbRemainingBytes -= 1

    def isAddressedAssigned(self, addr, index):
        """
        This function checks if the target address is an assigned com object one
        if yes, then it updates the adressed com object index with the index of the targeted com object and return true
        else return false
        """
        global _addressedComObjectIndex
        divisionCounter = 0
        if (self._assignedComObjectsNb == 0):
            return False

        i = 4
        while (self._assignedComObjectsNb >> i):
            divisionCounter += 1

        searchIndexStart = 0
        searchIndexStop = self._assignedComObjectsNb - 1
        searchIndexRange = self._assignedComObjectsNb

        while (divisionCounter):
            searchIndexRange >>= 1
            if (addr >= self._comObjectsList[self._orderedIndexTable[searchIndexStart + searchIndexRange]].getAddress()):
                searchIndexStart += searchIndexRange
            else:
                searchIndexStop -= searchIndexRange
            divisionCounter -= 1

        for i in range(searchIndexStart, searchIndexStop):
            if self._comObjectsList[self._orderedIndexTable[i]].getAddress() == addr:
                index = self._orderedIndexTable[i]
                _addressedComObjectIndex = index
                return True
            if (i > searchIndexStop):
                return False

    def attachComObjectsList(self, comObjectsList, listSize):
        """
        Attach a list of com objects (object with the communication attribute)
        This function returns KNX_TPUART_ERROR_NOT_INIT_STATE (254) if the TPUART is not in Init state and KNX_TPUART_OK otherwise.
        """
        if (self._rx.state != RX_INIT or self._tx.state != TX_INIT):
            return KNX_TPUART_ERROR_NOT_INIT_STATE

        if self._orderedIndexTable is not None:
            self._orderedIndexTable = None
            self._comObjectsList = None
            self._assignedComObjectsNb = 0

        if (comObjectsList is None or listSize is None or listSize == 0):
            return KNX_TPUART_OK

        for i in range(0, listSize):
            if (isCom(comObjectsList, i)):
                self._assignedComObjectsNb += 1

        if (self._assignedComObjectsNb is None or self._assignedComObjectsNb == 0):
            return KNX_TPUART_OK

        for i in range(0, listSize):
            if (isCom(comObjectsList, i)):
                continue
            for j in range(0, listSize):
                if ((i != j) and (comObjectsList[j].getAddress() != comObjectsList[i].getAddress()) and (isCom(j))):
                    if (j < i):
                        break
                    else:
                        self._assignedComObjectsNb -= 1

        self._comObjectsList = comObjectsList
        self._orderedIndexTable = [None] * self._assignedComObjectsNb
        minMin = 0x0000
        foundMin = 0xFFFF
        for i in range(0, self._assignedComObjectsNb):
            for j in range(0, listSize):
                if ((isCom(comObjectsList, j) and (comObjectsList[j].getAddress() >= minMin and (comObjectsList[i].getAddress() <= foundMin)))):
                    foundMin = comObjectsList[j].getAddress()
                    self._orderedIndexTable[i] = j

            minMin = foundMin + 1
            foundMin = 0xFF
        return KNX_TPUART_OK

    def setEvtCallback(self, evtCallbackFct):
        """
        This function saves the callback function to the upper layer (KnxDevice) to notify the device that there was a transmission

        Params:
                evtCallbackFct : the saved function used to notify the upper layer.
        """
        if (evtCallbackFct is None):
            return KNX_TPUART_ERROR
        if (self._rx.state != RX_INIT or self._tx.state != TX_INIT):
            return KNX_TPUART_ERROR_NOT_INIT_STATE
        self._evtCallbackFct = evtCallbackFct
        return KNX_TPUART_OK

    def setAckCallback(self, ackFctPtr):
        if ackFctPtr is None:
            return KNX_TPUART_ERROR
        if ((self._rx.state != RX_INIT or self._tx.state != TX_INIT)):
            return KNX_TPUART_ERROR_NOT_INIT_STATE
        self._tx.ackFctPtr = ackFctPtr
        return KNX_TPUART_OK

    def getTargetedComObjectIndex(self):
        return self._rx.addressedComObjectIndex

    def getReceivedTelegram(self):
        return self._rx.receivedTelegram


def millis():
    """
    This function returns the value (in milliseconds) of a clock which never goes backwards. Used for timing purpose.
    """
    return int(round(monotonic.monotonic() * 1000))


def micros():
    """
    This function returns the value (in microseconds) of a clock which never goes backwards. Used for timing purpose.
    """
    return int(round(monotonic.monotonic() * 1000000))


def isCom(objectList, index):
    """
    This function checks if the specified com object contains the "Communication" attribute
    """
    return objectList[index].getIndicator() & KnxComObject.KNX_COM_OBJ_C_INDICATOR
