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
    ParamType_Float, ParamType_Int, ParamType_Enum, CoordinateType_Depth, MV3D_RGBD_FLOAT_Z_UNIT,IpCfgMode_Static,IpCfgMode_DHCP,IpCfgMode_LLA
    
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
        if (DeviceType_Ethernet == stDeviceList.DeviceInfo[i].enDeviceType) or (DeviceType_Ethernet_Vir == stDeviceList.DeviceInfo[i].enDeviceType):  
            print("SerialNum[%s] IP[%s] name[%s] DeviceVersion[%s].\r\nManufacturerName[%s] UserDefinedName[%s] MacAddress[%x:%x:%x:%x:%x:%x] CurrentSubNetMask[%s] DefultGateWay[%s] NetExport[%s] Type[%d].\r\n"
                %(bytes(bytearray(stDeviceList.DeviceInfo[i].chSerialNumber)).decode('ascii'), bytes(bytearray(stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chCurrentIp)).decode('ascii'),bytes(bytearray(stDeviceList.DeviceInfo[i].chModelName)).decode('ascii') ,bytes(bytearray(stDeviceList.DeviceInfo[i].chDeviceVersion)).decode('ascii') ,
                bytes(bytearray(stDeviceList.DeviceInfo[i].chManufacturerName)).decode('ascii'), bytes(bytearray(stDeviceList.DeviceInfo[i].chUserDefinedName)).decode('ascii'),
                stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chMacAddress[0], stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chMacAddress[1], stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chMacAddress[2],
                stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chMacAddress[3], stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chMacAddress[4], stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chMacAddress[5],
                bytes(bytearray(stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chCurrentSubNetMask)).decode('ascii'),bytes(bytearray(stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chDefultGateWay)).decode('ascii') ,bytes(bytearray(stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chNetExport)).decode('ascii') ,
                (stDeviceList.DeviceInfo[i].nDevTypeInfo & 0xff000000) >> 24))

            if IpCfgMode_Static & stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.enIPCfgMode:
                print("IPCfgMode[STATIC]")
            if IpCfgMode_DHCP & stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.enIPCfgMode:
                print("IPCfgMode[DHCP]")
            if IpCfgMode_LLA & stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.enIPCfgMode:
                print("IPCfgMode[LLA]")
                
        elif (DeviceType_USB == stDeviceList.DeviceInfo[i].enDeviceType or DeviceType_USB_Vir == stDeviceList.DeviceInfo[i].enDeviceType):
    
            print("SerialNum[%s] name[%s] DeviceVersion[%s].\r\nManufacturerName[%s] UserDefinedName[%s] VendorId[%d] ProductId[%d] enUsbProtocol[%d] DeviceGUID[%s] Type[%d].\r\n" %
                (bytes(bytearray(stDeviceList.DeviceInfo[i].chSerialNumber)).decode('ascii'), bytes(bytearray(stDeviceList.DeviceInfo[i].chModelName)).decode('ascii'),bytes(bytearray(stDeviceList.DeviceInfo[i].chDeviceVersion)).decode('ascii') ,
                bytes(bytearray(stDeviceList.DeviceInfo[i].chManufacturerName)).decode('ascii'), bytes(bytearray(stDeviceList.DeviceInfo[i].chUserDefinedName)).decode('ascii'),
                stDeviceList.DeviceInfo[i].SpecialInfo.stUsbInfo.nVendorId, stDeviceList.DeviceInfo[i].SpecialInfo.stUsbInfo.nProductId,
                stDeviceList.DeviceInfo[i].SpecialInfo.stUsbInfo.enUsbProtocol, bytes(bytearray(stDeviceList.DeviceInfo[i].SpecialInfo.stUsbInfo.chDeviceGUID)).decode('ascii'),
                (stDeviceList.DeviceInfo[i].nDevTypeInfo & 0xff000000) >> 24))
        
        print("***********************************************\r\n",)

    i = 0
    with open("CurrentDeviceInfo.txt","w") as f:
        if (DeviceType_Ethernet == stDeviceList.DeviceInfo[i].enDeviceType) or (DeviceType_Ethernet_Vir == stDeviceList.DeviceInfo[i].enDeviceType):  
            chDevInfo = "SerialNum[{:s}] IP[{:s}]  name[{:s}]  DeviceVersion[{:s}].ManufacturerName[{:s}] UserDefinedName[{:s}]  MacAddress[{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}] CurrentSubNetMask[{:s}] DefultGateWay[{:s}] NetExport[{:s}] Type[{:d}].".format(
                bytes(bytearray(stDeviceList.DeviceInfo[i].chSerialNumber)).decode('ascii').strip(b'\x00'.decode()), bytes(bytearray(stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chCurrentIp)).decode('ascii').strip(b'\x00'.decode()),bytes(bytearray(stDeviceList.DeviceInfo[i].chModelName)).decode('ascii').strip(b'\x00'.decode()) ,bytes(bytearray(stDeviceList.DeviceInfo[i].chDeviceVersion)).decode('ascii').strip(b'\x00'.decode()) ,
                bytes(bytearray(stDeviceList.DeviceInfo[i].chManufacturerName)).decode('ascii').strip(b'\x00'.decode()), bytes(bytearray(stDeviceList.DeviceInfo[i].chUserDefinedName)).decode('ascii').strip(b'\x00'.decode()),stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chMacAddress[0], stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chMacAddress[1], stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chMacAddress[2],
                stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chMacAddress[3], stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chMacAddress[4], stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chMacAddress[5],
                bytes(bytearray(stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chCurrentSubNetMask)).decode('ascii').strip(b'\x00'.decode()),bytes(bytearray(stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chDefultGateWay)).decode('ascii').strip(b'\x00'.decode()) ,bytes(bytearray(stDeviceList.DeviceInfo[i].SpecialInfo.stNetInfo.chNetExport)).decode('ascii').strip(b'\x00'.decode()) ,
                (stDeviceList.DeviceInfo[i].nDevTypeInfo & 0xff000000) >> 24)
                
        elif (DeviceType_USB == stDeviceList.DeviceInfo[i].enDeviceType or DeviceType_USB_Vir == stDeviceList.DeviceInfo[i].enDeviceType):
            chDevInfo = "SerialNum[{:s}] name[{:s}] DeviceVersion[{:s}].ManufacturerName[{:s}] UserDefinedName[{:s}] VendorId[{:d}] ProductId[{:d}] enUsbProtocol[{:d}] DeviceGUID[{:s}] Type[{:d}].".format(
                bytes(bytearray(stDeviceList.DeviceInfo[i].chSerialNumber)).decode('ascii').strip(b'\x00'.decode()), bytes(bytearray(stDeviceList.DeviceInfo[i].chModelName)).decode('ascii').strip(b'\x00'.decode()),bytes(bytearray(stDeviceList.DeviceInfo[i].chDeviceVersion)).decode('ascii').strip(b'\x00'.decode()) ,
                bytes(bytearray(stDeviceList.DeviceInfo[i].chManufacturerName)).decode('ascii').strip(b'\x00'.decode()), bytes(bytearray(stDeviceList.DeviceInfo[i].chUserDefinedName)).decode('ascii').strip(b'\x00'.decode()),stDeviceList.DeviceInfo[i].SpecialInfo.stUsbInfo.nVendorId, stDeviceList.DeviceInfo[i].SpecialInfo.stUsbInfo.nProductId,
                stDeviceList.DeviceInfo[i].SpecialInfo.stUsbInfo.enUsbProtocol, bytes(bytearray(stDeviceList.DeviceInfo[i].SpecialInfo.stUsbInfo.chDeviceGUID)).decode('ascii').strip(b'\x00'.decode()),
                (stDeviceList.DeviceInfo[i].nDevTypeInfo & 0xff000000) >> 24)
        f.write(chDevInfo) 

    # ch:创建相机示例 | en:Create a camera instance
    camera=Mv3dRgbd()

    # ch:打开设备 | en:Open device  
    ret = camera.MV3D_RGBD_OpenDevice(pointer(stDeviceList.DeviceInfo[int(i)]))

    if ret != 0:
        print ("MV3D_RGBD_OpenDevice fail! ret[0x%x]" % ret)
        os.system('pause')
        sys.exit()

    # ch:销毁句柄 | en:Destroy the device handle 
    ret=camera.MV3D_RGBD_CloseDevice()
    if ret != 0:
        print ("CloseDevice fail! ret[0x%x]" % ret)
        os.system('pause')
        sys.exit()

    print("Main done !")
    os.system('pause')
    sys.exit()

