"""
This module is used to define the knx communication objects connected to the device. KnxDevice's objects list must be specified in the main script. 
"""

import KnxDPT
import KnxTelegram

# See "knx.org" for com obj indicators specification
# INDICATOR field : B7  B6  B5  B4  B3  B2  B1  B0
#                   xx  xx   C   R   W   T   U   I
KNX_COM_OBJ_C_INDICATOR = 0x20  # Comuunication (C)
KNX_COM_OBJ_R_INDICATOR = 0x10  # Read (R)
KNX_COM_OBJ_W_INDICATOR = 0x08  # Write (W)
KNX_COM_OBJ_T_INDICATOR = 0x04  # Transmit (T)
KNX_COM_OBJ_U_INDICATOR = 0x02  # Update (U)
KNX_COM_OBJ_I_INDICATOR = 0x01  # Init Read (I)

KNX_COM_OBJECT_OK = 0
KNX_COM_OBJECT_ERROR = 255

KNX_PRIORITY_NORMAL_VALUE = 12


class KnxComObject:
    _addr = None
    dptId = None
    _indicator = None
    _length = None
    _validity = None
    _value = None
    _longValue = None

    def __init__(self, addr, dptId, indicator):
        """
        Constructor for the KnxComObject representing a device on the Knx bus.
        """
        self._addr = addr
        self.dptId = dptId
        self._indicator = indicator
        self._length = self.lengthCalculation(dptId)

        if (self._length == 1):
            self._longValue = None
        else:
            self._longValue = [None] * (self._length - 1)
            for i in range(0, self._length - 1):
                self._longValue[i] = 0

        if (self._indicator & KNX_COM_OBJ_I_INDICATOR):
            self._validity = False
        else:
            self._validity = True

    def updateValue(self, ori):
        """
        This function saves the value received through telegrams in the matching KnxComObject.

        Params:
                ori : the value received either as a telegram or an integer
        """
    
        if isinstance(ori, KnxTelegram.KnxTelegram):
            if ori.getPayloadLength() != self.getLength():
                return KNX_COM_OBJECT_ERROR
            if self._length == 1:
                self._value = ori.getFirstPayloadByte()
            elif self._length == 2:
                ori.getLongPayload(self._value, 1)
            else:
                ori.getLongPayload(self._longValue, self._length - 1)
            self._validity = True
            return KNX_COM_OBJECT_OK
        elif isinstance(ori, int):
            if (self._length > 2):
                return KNX_COM_OBJECT_ERROR
            self._value = ori
            self._validity = True
            return KNX_COM_OBJECT_OK

    def copyAttributes(self, dest):
        """
        This function copies the com object attributes in the specified telegram. This includes the priority, the address and the length of the com object.

        Params:
                dest: the telegram wherein the value will be copied
        """
        dest.changePriority(self.getPriority())
        dest.setTargetAddress(self.getAddress())
        dest.setPayloadLength(self._length)

    def copyValue(self, dest):
        """
        This function copies the com object value in the specified telegram.

        Params:
                dest: the telegram wherein the value will be copied
        """
        if (self._length == 1):
            dest.setFirstPayloadByte(self._value)
        elif self._length == 2:
            dest.setLongPayload(self._value, 1)
        else:
            dest.setLongPayload(self._longValue, self._length - 1)

    def lengthCalculation(self, dptId):
        return ((KnxDPT.KnxDPTFormatToLengthBit[KnxDPT.KnxDPTIdToFormat[dptId]] / 8) + 1)

    def getIndicator(self):
        return self._indicator

    def getAddress(self):
        return self._addr

    def getLength(self):
        return self._length

    def getValidity(self):
        return self._validity

    def getValue(self):
        return self._value

    def getPriority(self):
        return KNX_PRIORITY_NORMAL_VALUE

    def getDptId(self):
        return self.dptId            
