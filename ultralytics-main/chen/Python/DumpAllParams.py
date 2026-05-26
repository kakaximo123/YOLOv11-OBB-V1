# -- coding: utf-8 --

import threading
import msvcrt

import ctypes
import time
import os

from ctypes import *

from Mv3dRgbdImport.Mv3dRgbdDefine import *

from Mv3dRgbdImport.Mv3dRgbdApi import *

from Mv3dRgbdImport.Mv3dRgbdDefine import DeviceType_Ethernet, DeviceType_USB, DeviceType_Ethernet_Vir, DeviceType_USB_Vir, MV3D_RGBD_FLOAT_EXPOSURETIME, ParamType_Float, ParamType_Int, ParamType_Enum,ParamType_Bool, CoordinateType_Depth,ParamType_String, MV3D_RGBD_FLOAT_Z_UNIT,MV3D_RGBD_OK,MV3D_RGBD_INT_WIDTH, MV3D_RGBD_INT_HEIGHT,MV3D_RGBD_ENUM_WORKINGMODE,MV3D_RGBD_ENUM_PIXELFORMAT,MV3D_RGBD_ENUM_IMAGEMODE,MV3D_RGBD_FLOAT_GAIN,MV3D_RGBD_FLOAT_FRAMERATE,MV3D_RGBD_ENUM_TRIGGERSELECTOR,MV3D_RGBD_ENUM_TRIGGERMODE,MV3D_RGBD_ENUM_TRIGGERSOURCE, MV3D_RGBD_FLOAT_TRIGGERDELAY, MV3D_RGBD_INT_IMAGEALIGN
    

def DumpParam(camera = 0, pParamName =""):

    pstValue = MV3D_RGBD_PARAM()
    pstValue_p= byref (pstValue)
    nRet = MV3D_RGBD_OK

    nRet = camera.MV3D_RGBD_GetParam(pParamName, pstValue_p)

    if nRet!= MV3D_RGBD_OK:
        print("MV3D_RGBD_GetParam  pParamName [%s] fail! ret[0x%x]" %pParamName % nRet)
        return

    if ParamType_Int == pstValue.enParamType:

        print("ParamName : %s, Current Value: %d ,Max Value: %d ,Min Value: %d \r\n"
            % (pParamName , pstValue.ParamInfo.stIntParam.nCurValue , pstValue.ParamInfo.stIntParam.nMax ,
            pstValue.ParamInfo.stIntParam.nMin))

    elif ParamType_Float == pstValue.enParamType:

        print("ParamName : %s, Current Value: %f ,Max Value: %f ,Min Value: %f \r\n"
        %(pParamName ,pstValue.ParamInfo.stFloatParam.fCurValue ,pstValue.ParamInfo.stFloatParam.fMax ,pstValue.ParamInfo.stFloatParam.fMin))

    elif ParamType_Enum == pstValue.enParamType:

        print("ParamName : %s, Current Value: %d ,Supported Number: %d \r\n" 
        %(pParamName ,pstValue.ParamInfo.stEnumParam.nCurValue ,pstValue.ParamInfo.stEnumParam.nSupportedNum))
        print("            %s Enum options :\r\n" %pParamName)
        for  i in range(0,pstValue.ParamInfo.stEnumParam.nSupportedNum):

            print("            Support Value is [%d] \r\n" %pstValue.ParamInfo.stEnumParam.nSupportValue[i])

    elif ParamType_Bool == pstValue.enParamType:

        print("ParamName : %s, Current BoolValue: %d \r\n" %pParamName %pstValue.ParamInfo.bBoolParam)

    elif ParamType_String == pstValue.enParamType:

        print("ParamName : %s, Current String MaxLength: %d,Current String Value: %s\r\n" %(pParamName ,pstValue.ParamInfo.stStringParam.nMaxLength ,str(pstValue.ParamInfo.stStringParam.chCurValue,encoding = 'ascii')))
    return


def DumpAllParams(pHandle = 0):

    DumpParam(pHandle, "DeviceModelName")

    DumpParam(pHandle, MV3D_RGBD_INT_WIDTH)

    DumpParam(pHandle, MV3D_RGBD_INT_HEIGHT)

    DumpParam(pHandle, MV3D_RGBD_ENUM_WORKINGMODE)

    DumpParam(pHandle, MV3D_RGBD_ENUM_PIXELFORMAT)

    DumpParam(pHandle, MV3D_RGBD_ENUM_IMAGEMODE)

    DumpParam(pHandle, MV3D_RGBD_FLOAT_GAIN)

    DumpParam(pHandle, MV3D_RGBD_FLOAT_EXPOSURETIME)

    DumpParam(pHandle, MV3D_RGBD_FLOAT_FRAMERATE)

    DumpParam(pHandle, MV3D_RGBD_ENUM_TRIGGERSELECTOR)

    DumpParam(pHandle, MV3D_RGBD_ENUM_TRIGGERMODE)

    DumpParam(pHandle, MV3D_RGBD_ENUM_TRIGGERSOURCE)

    DumpParam(pHandle, MV3D_RGBD_FLOAT_TRIGGERDELAY)

    DumpParam(pHandle, MV3D_RGBD_INT_IMAGEALIGN)

    return MV3D_RGBD_OK
    


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

    ret = DumpAllParams(camera)


    # ch:销毁句柄 | en:Destroy the device handle 

    ret=camera.MV3D_RGBD_CloseDevice()

    if ret != 0:

        print ("CloseDevice fail! ret[0x%x]" % ret)

        os.system('pause')

        sys.exit()
    

    sys.exit()

