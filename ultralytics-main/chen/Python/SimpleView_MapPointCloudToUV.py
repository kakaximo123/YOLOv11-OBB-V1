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
    ParamType_Float, ParamType_Int, ParamType_Enum, CoordinateType_Depth, CoordinateType_RGB, MV3D_RGBD_FLOAT_Z_UNIT,ImageType_Depth, ImageType_RGB8_Planar, ImageType_PointCloud, \
    PointCloudType_Common,PointCloudType_Normals ,PointCloudType_Texture, PointCloudType_Texture_Normals, MV3D_RGBD_ENUM_POINT_CLOUD_OUTPUT
    
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

    # ch:设置点云输出节点 | en:Set point cloud output node
    stParam = MV3D_RGBD_PARAM()
    stParam.enParamType = ParamType_Enum
    stParam.ParamInfo.stEnumParam.nCurValue = PointCloudType_Common
    ret = camera.MV3D_RGBD_SetParam(MV3D_RGBD_ENUM_POINT_CLOUD_OUTPUT, pointer(stParam))

    if 0!=ret:
        print ("SetParam fail! ret[0x%x]" % ret)
        camera.MV3D_RGBD_CloseDevice()
        os.system('pause')
        sys.exit()
   
    print ("Set point cloud output success.")

    # ch:开始取流 | en:Start grabbing
    ret=camera.MV3D_RGBD_Start()
    if ret != 0:
        print ("start fail! ret[0x%x]" % ret)
        camera.MV3D_RGBD_CloseDevice()
        os.system('pause')
        sys.exit()

    stFrameData = MV3D_RGBD_FRAME_DATA()
    stCalibInfo = MV3D_RGBD_CALIB_INFO()
    bCalibInfo = 0
    # ch:获取图像数据 | en:Get image data
    ret = camera.MV3D_RGBD_FetchFrame(pointer(stFrameData), 5000)
    if 0 == ret:
        print("MV3D_RGBD_FetchFrame success.")
        for i in range(0, stFrameData.nImageCount):
            if ImageType_PointCloud == stFrameData.stImageData[i].enImageType:
                pstPointCloudImage = stFrameData.stImageData[i]
                print("Get point cloud succeed: framenum (%d) height(%d) width(%d)  len (%d)!" %( pstPointCloudImage.nFrameNum,
                        pstPointCloudImage.nHeight, pstPointCloudImage.nWidth, pstPointCloudImage.nDataLen))
                # ch:获取uv坐标 | en: Get uv coordinate
                stUvMap = MV3D_RGBD_UV_DATA()
                if 0==bCalibInfo:
                    ret = camera.MV3D_RGBD_GetCalibInfo(stFrameData.stImageData[i].enCoordinateType, pointer(stCalibInfo))
                    if 0!=ret:
                        print("Get calib info failed...")
                        break
                    else:
                        bCalibInfo = 1
                ret = camera.MV3D_RGBD_MapPointCloudToUV(pointer(stFrameData.stImageData[i]), pointer(stCalibInfo), pointer(stUvMap))
                if 0==ret:
                    strMode = string_at(stUvMap.pData, stUvMap.nDataLen)
                    sValue = struct.unpack('f'*int(pstPointCloudImage.nHeight * pstPointCloudImage.nWidth * 2), strMode)
                    with open("UV_data.txt","w") as f:
                        for j in range(0, pstPointCloudImage.nHeight * pstPointCloudImage.nWidth):
                            strUVValue = "U:{:f},V:{:f} \n".format(sValue[j * 2],sValue[j * 2 + 1])
                            f.write(strUVValue)
                else:
                    print("Get uv map failed...")  
            # ch:解析彩色图纹理数据 | en: Parse color texture data
            if ImageType_RGB8_Planar == stFrameData.stImageData[i].enImageType:
                print("Get texture color image succeed: framenum (%d) height(%d) width(%d) len(%d)!" %(stFrameData.stImageData[i].nFrameNum,
                    stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nDataLen))
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
    
    print ("Main done")
    sys.exit()
