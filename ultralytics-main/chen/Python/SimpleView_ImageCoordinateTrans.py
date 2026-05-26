# -- coding: utf-8 --
import threading
import msvcrt
import ctypes
import time
import os
import struct
from ctypes import *
from Mv3dRgbdImport.Mv3dRgbdDefine import *
from Mv3dRgbdImport.Mv3dRgbdApi import *
from Mv3dRgbdImport.Mv3dRgbdDefine import DeviceType_Ethernet, DeviceType_USB, DeviceType_Ethernet_Vir, DeviceType_USB_Vir, MV3D_RGBD_FLOAT_EXPOSURETIME, \
    ParamType_Float, ParamType_Int, ParamType_Enum, CoordinateType_Depth, MV3D_RGBD_FLOAT_Z_UNIT, MV3D_RGBD_OK,ImageType_Depth, \
        MV3D_RGBD_CAMERA_PARAM,MV3D_RGBD_INT_IMAGEALIGN
  
if __name__ == "__main__":
    nDeviceNum=ctypes.c_uint(0)
    nDeviceNum_p=byref(nDeviceNum)
    # ch:获取设备数量 | en:Get device number
    ret=Mv3dRgbd.MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, nDeviceNum_p) 
    if  ret!=0:
        print("MV3D_RGBD_GetDeviceNumber fail! ret[0x%x]" % ret)
        os.system('pause')
        sys.exit()
    if  nDeviceNum==0:
        print("find no device!")
        os.system('pause')
        sys.exit()
    print("Find devices numbers:", nDeviceNum.value)
    
    stDeviceList = MV3D_RGBD_DEVICE_INFO_LIST()
    net = Mv3dRgbd.MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, pointer(stDeviceList.DeviceInfo[0]), 20, nDeviceNum_p)
    
    for i in range(0, nDeviceNum.value):
        print("\ndevice: [%d]" % i)
        strModeName = ""
        for per in stDeviceList.DeviceInfo[i].chModelName:
            strModeName = strModeName + chr(per)
        print("device model name: %s" % strModeName)

        strSerialNumber = ""
        for per in stDeviceList.DeviceInfo[i].chSerialNumber:
            strSerialNumber = strSerialNumber + chr(per)
        print("device SerialNumber: %s" % strSerialNumber)

    # ch:创建相机示例 | en:Create a camera instance
    camera=Mv3dRgbd()
    nConnectionNum = input("please input the number of the device to connect:")
    if int(nConnectionNum) >= nDeviceNum.value:
        print("intput error!")
        os.system('pause')
        sys.exit()

    # ch:打开设备 | en:Open device  
    ret = camera.MV3D_RGBD_OpenDevice(pointer(stDeviceList.DeviceInfo[int(nConnectionNum)]))
    if ret != 0:
        print ("MV3D_RGBD_OpenDevice fail! ret[0x%x]" % ret)
        os.system('pause')
        sys.exit()

    # ch:关闭图像对齐深度图坐标系模式 | en:Close image align depth coordinate mode
    stParam = MV3D_RGBD_PARAM()
    stParam.enParamType = ParamType_Int
    stParam.ParamInfo.stIntParam.nCurValue = 0
    ret = camera.MV3D_RGBD_SetParam(MV3D_RGBD_INT_IMAGEALIGN, pointer(stParam))

    if MV3D_RGBD_OK != ret:
        print ("SetParam fail! ret[0x%x]" % ret)
        camera.MV3D_RGBD_CloseDevice()
        os.system('pause')
        sys.exit()

    print("Close image align success.")

    ret = camera.MV3D_RGBD_GetParam(MV3D_RGBD_FLOAT_Z_UNIT, pointer(stParam))
    if MV3D_RGBD_OK != ret:
        print ("GetParam fail! ret[0x%x]" % ret)
        camera.MV3D_RGBD_CloseDevice()
        os.system('pause')
        sys.exit()

    print("Get z-unit success.")
    fZunit = stParam.ParamInfo.stFloatParam.fCurValue

    # ch:开始取流 | en:Start grabbing
    ret=camera.MV3D_RGBD_Start()
    if ret != 0:
        print ("start fail! ret[0x%x]" % ret)
        camera.MV3D_RGBD_CloseDevice()
        os.system('pause')
        sys.exit()

    stFrameData = MV3D_RGBD_FRAME_DATA()
    # ch:获取图像数据 | en:Get image data
    ret = camera.MV3D_RGBD_FetchFrame(pointer(stFrameData), 5000)
    if MV3D_RGBD_OK == ret:
        if stFrameData.nValidInfo == False:
            print("MV3D_RGBD_FetchFrame success.") 
            for i in range(0, stFrameData.nImageCount):
                if ImageType_Depth == stFrameData.stImageData[i].enImageType:
                    print("depth raw image: framenum (%d) height(%d) width(%d) len(%d) coordinate(%d)!" %(stFrameData.stImageData[i].nFrameNum,
                            stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nDataLen, stFrameData.stImageData[i].enCoordinateType))
                    # ch:获取传感器标定信息 | en:Get sensor calib info
                    stCameraParam = MV3D_RGBD_CAMERA_PARAM()
                    ret = camera.MV3D_RGBD_GetCameraParam(pointer(stCameraParam))
                    if MV3D_RGBD_OK != ret:
                        print ("MV3D_RGBD_GetCameraParam fail! ret[0x%x]" % ret)
                        camera.MV3D_RGBD_CloseDevice()
                        os.system('pause')
                        sys.exit()
                    print("Get camera param success.")
                   
                    # ch:将深度图转换到RGB坐标系下 | en:Convert depth map to rgb coordinate
                    stDepthConvImg = MV3D_RGBD_IMAGE_DATA()
                    ret = camera.MV3D_RGBD_ImageCoordinateTrans(pointer(stFrameData.stImageData[i]), fZunit, pointer(stDepthConvImg),  pointer(stCameraParam))
                    if MV3D_RGBD_OK == ret:
                        print("depth convert image: framenum (%d) height(%d) width(%d) len(%d) coordinate(%d)!" % (stDepthConvImg.nFrameNum,
                            stDepthConvImg.nHeight, stDepthConvImg.nWidth, stDepthConvImg.nDataLen, stFrameData.stImageData[i].enCoordinateType))
                    else:
                        print("MV3D_RGBD_ImageCoordinateTrans failed...sts[%#x]", ret)         
        else:
            print("stFrameData.nValidInfo is true!")
    else:
        print("MV3D_RGBD_FetchFrame lost frame!")
            
    # ch:停止取流 | en:Stop grabbing
    ret=camera.MV3D_RGBD_Stop()
    if ret != 0:
        print ("stop fail! ret[0x%x]" % ret)
        os.system('pause')
        sys.exit()

    # ch:销毁句柄 | en:Destroy the device handle 
    ret=camera.MV3D_RGBD_CloseDevice()
    if ret != 0:
        print ("CloseDevice fail! ret[0x%x]" % ret)
        os.system('pause')
        sys.exit()
    
    sys.exit()
