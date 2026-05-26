# -- coding: utf-8 --
import threading
import msvcrt
import ctypes
import time
import os
from ctypes import *
from Mv3dRgbdImport.Mv3dRgbdDefine import *
from Mv3dRgbdImport.Mv3dRgbdApi import *
from Mv3dRgbdImport.Mv3dRgbdDefine import DeviceType_Ethernet, DeviceType_USB, DeviceType_Ethernet_Vir, DeviceType_USB_Vir, MV3D_RGBD_FLOAT_EXPOSURETIME, \
    ParamType_Float, ParamType_Int, ParamType_Enum, CoordinateType_Depth, MV3D_RGBD_FLOAT_Z_UNIT, Mv3dRgbdIpCfgMode
    
def GetDestIP():
    stIpCfgInfo = MV3D_RGBD_IP_CONFIG()
    print("Please enter the network mode of the camera to be configure \n")
    nIPCfgMode  = int(input("[1] STATIC [2] DHCP [3] LLA :"))
 
    if (1 == nIPCfgMode):
        pNewIP = input("Please enter the camera IP to be set: ").encode("ascii")
        for i in range(0, len(pNewIP)):
            stIpCfgInfo.chDestIp[i] = pNewIP[i]

        pNewNetMask = input("Please enter the camera Netmask to be set: ").encode("ascii")
        for i in range(0, len(pNewNetMask)):
            stIpCfgInfo.chDestNetMask[i] = pNewNetMask[i]
        
        pNewDestGateWay = input("Please enter the camera Gateway to be set: ").encode("ascii")
        for i in range(0, len(pNewDestGateWay)):
            stIpCfgInfo.chDestGateWay[i] = pNewDestGateWay[i]

    elif (3 == nIPCfgMode):
        nIPCfgMode = IpCfgMode_LLA

    elif ((0 >=  nIPCfgMode)  or (3 < nIPCfgMode)):
        print("Enter error ! Get it error!")
        return 0, stIpCfgInfo

    stIpCfgInfo.enIPCfgMode = Mv3dRgbdIpCfgMode(nIPCfgMode)
    return 1, stIpCfgInfo

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
    stIpCfgInfo = MV3D_RGBD_IP_CONFIG()

    ret, stIpCfgInfo = GetDestIP()
    if (0 == ret ):
        print("GetDestIP fail!")
        sys.exit()
    else:
        ret  = camera.MV3D_RGBD_SetIpConfig(stDeviceList.DeviceInfo[1].chSerialNumber,stIpCfgInfo)
        if ret  == 0:
            print("MV3D_RGBD_SetIpConfig success!")
        else:
            print("MV3D_RGBD_SetIpConfig fail! ret[0x%x]" % ret)
            sys.exit()

    print("Main done !")
    os.system('pause')
    sys.exit()

