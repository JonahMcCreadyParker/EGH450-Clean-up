import depthai as dai
#sup
with dai.Device() as device:
    print(device.getConnectedCameraFeatures())
