"""
This python script is used to create an instance for the knx system. It emulates the Raspberri Pi as a knx device connected to the bus by the TPUART.
"""

import KnxTPUart
import KnxTelegram
import KnxComObject
import monotonic
import KnxDPT

NORMAL = 0
BUS_MONITOR = 1
INIT = 0
IDLE = 1
TX_ONGOING = 2

KNX_DEVICE_OK = 0
KNX_DEVICE_ERROR = 255

TPUART_EVENT_RESET = 0
KNX_TPUART_ERROR = 255

ACTIONS_QUEUE_SIZE = 16

EIB_READ_REQUEST = 0
EIB_WRITE_REQUEST = 1
EIB_RESPONSE_REQUEST = 2

KNX_COMMAND_VALUE_READ = 0
KNX_COMMAND_VALUE_RESPONSE = 1
KNX_COMMAND_VALUE_WRITE = 2

_Knx = None
_targetedComObjIndex = 0


class ActionRingBuffer:
    """
    Custom list class used to queue tx actions.
    """
    _head = None
    _tail = None
    _buffer = [None]
    _size = None
    _elementsCurrentNb = None

    def __init__(self, size):
        """
        Constructor for the ActionRingBuffer class.
        """
        self._head = 0
        self._tail = 0
        self._elementsCurrentNb = 0
        self._size = size
        self._buffer = [None] * size

    def append(self, data):
        """
        This function appends the data into the custom list
        """
        if self._elementsCurrentNb == self._size:
            self.incrementHead()
        else:
            self._elementsCurrentNb += 1

        self._buffer[self._tail] = data
        self.incrementTail()

    def pop(self, data):
        """
        This function pops the most recently added element from the list and returns it 
        """
        if (self._elementsCurrentNb == 0):
            return -1
        data = self._buffer[self._head]
        self.incrementHead()
        self._elementsCurrentNb -= 1
        return data

    def incrementHead(self):
        self._head = (self._head + 1) % self._size

    def incrementTail(self):
        self._tail = (self._tail + 1) % self._size


class tx_action:
    """
    Tx action class
    """
    command = None
    index = None
    byteValue = None
    notUsed = None
    valuePtr = None


class KnxDevice:
    _comObjectsList = None
    _comObjectsNb = None
    _state = None
    _tpuart = None
    _initCompleted = False
    _initIndex = None
    _rxTelegram = KnxTelegram.KnxTelegram()
    _txTelegram = KnxTelegram.KnxTelegram()
    _txActionsList = ActionRingBuffer(ACTIONS_QUEUE_SIZE)
    _knxEventsCallback = None
    _lastInitTimeMillis = None
    _lastTXTimeMicros = None
    _lastRXTimeMicros = 0

    def __init__(self):
        """
        Constructor for the KnxDevice class
        """
        self._state = INIT
        self._tpuart = None
        self._initCompleted = False
        self._initIndex = 0
        self._rxTelegram = None

    def begin(self, serial, physicalAddr, Knx, knxEvtCallback):
        """
        Start the Knx device and check if it's active. Return KNX_DEVICE_ERROR (255) if it failed and KNX_DEVICE_OK otherwise.
        The function tries to "ping" the tpuart to see if it's responding.

        Params:
                serial : raspberry pi serial port, /dev/ttyAMA0 for Raspi 2 B
                physicalAddr: physical address associated with the serial port
                Knx : the current knx instance saved for use in callbacks
                knxevtcallback : callback to get notified of knx events from the upper layer (main script)
        """
        global _Knx
        _Knx = Knx
        self._knxEventsCallback = knxEvtCallback
        self._tpuart = KnxTPUart.KnxTPUart(
            serial, physicalAddr, NORMAL)
        self._rxTelegram = self._tpuart.getReceivedTelegram()
        if (self._tpuart.reset() != KnxTPUart.KNX_TPUART_OK):
            self._tpuart = None
            self._rxTelegram = None
            return KNX_DEVICE_ERROR

        self._tpuart.attachComObjectsList(
            self._comObjectsList, self._comObjectsNb)
        self._tpuart.setEvtCallback(self.getTpUartEvents)
        self._tpuart.setAckCallback(self.txTelegramAck)
        self._tpuart.init()
        self._state = IDLE

        self._lastInitTimeMillis = millis()
        self._lastTXTimeMicros = micros()
        return KNX_DEVICE_OK

    def task(self):
        """
        Knx device execution task. It will handle the reception and transmission of knx telegrams through the uart port by using the tp uart module.
        """
        global test, value
        action = tx_action()

        if self._initCompleted == False:
            nowTimeMillis = millis()
            if ((nowTimeMillis - self._lastInitTimeMillis) > 500):
                while ((self._initIndex < self._comObjectsNb) and self._comObjectsList[self._initIndex].getValidity()):
                    self._initIndex += 1

                if (self._initIndex == self._comObjectsNb):
                    self._initCompleted = True

                else:
                    action.command = EIB_READ_REQUEST
                    action.index = self._initIndex
                    self._txActionsList.append(action)
                    self._lastInitTimeMillis = millis()

        nowTimeMicros = micros()
        if ((nowTimeMicros - self._lastRXTimeMicros) > 200):
            self._lastRXTimeMicros = nowTimeMicros
            self._tpuart.RxTask()

        if self._state == IDLE:
            action = self._txActionsList.pop(action)
            if (action != -1):
                if (action.command == EIB_READ_REQUEST):
                    self._comObjectsList[action.index].copyAttributes(
                        self._txTelegram)
                    self._txTelegram.clearLongPayload()
                    self._txTelegram.clearFirstPayloadByte()
                    self._txTelegram.setCommand(KNX_COMMAND_VALUE_READ)
                    self._txTelegram.updateChecksum()
                    self._tpuart.sendTelegram(self._txTelegram)
                    self._state = TX_ONGOING
                if (action.command == EIB_RESPONSE_REQUEST):
                    self._comObjectsList[action.index].copyAttributes(
                        self._txTelegram)
                    self._comObjectsList[action.index].copyValue(
                        self._txTelegram)
                    self._txTelegram.setCommand(KNX_COMMAND_VALUE_RESPONSE)
                    self._txTelegram.updateChecksum()
                    self._tpuart.sendTelegram(self._txTelegram)
                    self._state = TX_ONGOING
                if (action.command == EIB_WRITE_REQUEST):
                    if ((self._comObjectsList[action.index].getLength()) <= 2):
                        self._comObjectsList[action.index].updateValue(
                            action.byteValue)
                    else:
                        self._comObjectsList[action.index].updateValue(
                            action.valuePtr)
                    if ((self._comObjectsList[action.index].getIndicator()) & KnxComObject.KNX_COM_OBJ_T_INDICATOR):
                        self._comObjectsList[action.index].copyAttributes(
                            self._txTelegram)
                        self._comObjectsList[action.index].copyValue(
                            self._txTelegram)
                        self._txTelegram.setCommand(KNX_COMMAND_VALUE_WRITE)
                        self._txTelegram.updateChecksum()
                        self._tpuart.sendTelegram(self._txTelegram)
                        self._state = TX_ONGOING

        nowTimeMicros = micros()
        if (nowTimeMicros - self._lastTXTimeMicros > 800):
            self._lastTXTimeMicros = nowTimeMicros
            self._tpuart.TxTask()

    def getTpUartEvents(self, event):
        """
        Callback from KnxTpUart module. This function is called when an event is detected on the bus. It will then perform the specified action depending on the
        occurring event.

        Params:
                event: the event type detected by the TP uart. 
        """
        global _targetedComObjIndex, _Knx
        action = tx_action()

        if (event == KnxTPUart.TPUART_EVENT_RECEIVED_EIB_TELEGRAM):
            _Knx._state = IDLE
            _targetedComObjIndex = _Knx._tpuart.getTargetedComObjectIndex()

            rxTelegramCommand = _Knx._rxTelegram.getCommand()
            if rxTelegramCommand == KNX_COMMAND_VALUE_READ:
                if ((self._comObjectsList[_targetedComObjIndex].getIndicator()) & KnxComObject.KNX_COM_OBJ_R_INDICATOR):
                    action.command = EIB_RESPONSE_REQUEST
                    action.index = _targetedComObjIndex
                    _Knx._txActionsList.append(action)

            elif rxTelegramCommand == KNX_COMMAND_VALUE_RESPONSE:
                if ((self._comObjectsList[_targetedComObjIndex].getIndicator() & KnxComObject.KNX_COM_OBJ_U_INDICATOR)):
                    self._comObjectsList[_targetedComObjIndex].updateValue(
                        _Knx._rxTelegram)
                    self._knxEventsCallback(_targetedComObjIndex)

            elif rxTelegramCommand == KNX_COMMAND_VALUE_WRITE:
                if ((self._comObjectsList[_targetedComObjIndex].getIndicator()) & KnxComObject.KNX_COM_OBJ_W_INDICATOR):
                    self._comObjectsList[_targetedComObjIndex].updateValue(
                        _Knx._rxTelegram)
                    self._knxEventsCallback(_targetedComObjIndex)

        if event == TPUART_EVENT_RESET:
            while (_Knx._tpuart.reset() == KNX_TPUART_ERROR):
                _Knx._tpuart.init()
                _Knx._state = IDLE

    def txTelegramAck(self, value):
        global _Knx
        _Knx._state = IDLE

    def read(self, index):
        """
        This function reads the short value of the specified com object in the received telegram.

        Params:
                index: the index corresponding to the com object in the defined com list.
        """
        return self._comObjectsList[index].getValue()

    def write(self, objectIndex, value):
        """
        This function sends a telegram to the specified com object and with the specified value.

        Params:
                objectIndex : the index corresponding to the object in the defined com list.
                value : the value you want to send through the telegram.
        """
        action = tx_action()
        length = self._comObjectsList[objectIndex].getLength()

        if (length <= 2):
            action.byteValue = value & 0xFF
        else:
            destValue = [None] * (length - 1)
            action.valuePtr = destValue

        action.command = EIB_WRITE_REQUEST
        action.index = objectIndex
        self._txActionsList.append(action)
        return KNX_DEVICE_OK


def G_ADDR(maingrp, midgrp, subgrp):
    return (((maingrp & 0x1F) << 11) + ((midgrp & 0x7) << 8) + subgrp)


def P_ADDR(area, line, busdevice):
    return (((area & 0xF) << 12) + ((line & 0xF) << 8) + busdevice)


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
