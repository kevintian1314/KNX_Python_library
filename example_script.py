import KnxComObject
import KnxDevice
import KnxDPT
import KnxTelegram

# Knx device instance
Knx = KnxDevice.KnxDevice()

# Definition of the Communication Objects attached to the device
Knx._comObjectsList = [
    KnxComObject.KnxComObject(KnxDevice.G_ADDR(0, 0, 1), 0, 0x2E),
    KnxComObject.KnxComObject(KnxDevice.G_ADDR(0, 0, 1), 0, 0x2C)
]

# Number of communication objects saved in the device.
Knx._comObjectsNb = len(Knx._comObjectsList)

# Function called from the previous layer (KnxDevice) to allow the script to be notified of a telegram event.
def KnxEvents(index):
    if index == 1:
        if (Knx.read(1)):
            print("ON")
        else:
            print("OFF")

try:
    Knx.begin("/dev/ttyAMA0", KnxDevice.P_ADDR(1, 1, 3), Knx, KnxEvents)
    # Knx.write(1, 1)
    while 1:
        Knx.task()
except KeyboardInterrupt:
    print("\nProgram ended")
