# -- coding: utf-8 --
import msvcrt
import os
from Mv3dRgbdImport.Mv3dRgbdDefine import *
from Mv3dRgbdImport.Mv3dRgbdApi import *
from Mv3dRgbdImport.Mv3dRgbdDefine import DeviceType_Ethernet, DeviceType_USB, DeviceType_Ethernet_Vir, DeviceType_USB_Vir, MV3D_RGBD_OK,DevException_Disconnect,ImageType_Depth,ImageType_RGB8_Planar

device_offline = False
ExceptionInfoCallBack=WINFUNCTYPE(None, c_void_p,c_void_p)

def exception_callback(pstExceptionData, userdata):
    if pstExceptionData!=None:
        stExceptionData = cast(pstExceptionData, POINTER(MV3D_RGBD_EXCEPTION_INFO)).contents
        if DevException_Disconnect == stExceptionData.enExceptionId:
            device_offline = True
            print("=== Event Callback: Device Offline!")


CALL_BACK_FUN=ExceptionInfoCallBack(exception_callback)

if __name__ == "__main__":

    nDeviceNum = ctypes.c_uint(0)
    nDeviceNum_p = byref(nDeviceNum)
    # ch:获取设备数量 | en:Get device number
    ret = Mv3dRgbd.MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, nDeviceNum_p)
    if ret != 0:
        print("MV3D_RGBD_GetDeviceNumber fail! ret[0x%x]" % ret)
        os.system('pause')
        sys.exit()
    if nDeviceNum == 0:
        print("find no device!")
        os.system('pause')
        sys.exit()
    print("Find devices numbers:", nDeviceNum.value)
    bLoop_exit = False
    while(bLoop_exit == False):

        # ch:枚举设备信息 | en:Enumerate device information
        stDeviceList = MV3D_RGBD_DEVICE_INFO_LIST()
        net = Mv3dRgbd.MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, 
                                        pointer(stDeviceList.DeviceInfo[0]), 20, nDeviceNum_p)
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
        camera = Mv3dRgbd()
        nConnectionNum = input("please input the number of the device to connect:")
        if int(nConnectionNum) >= nDeviceNum.value:
            print("intput error!")
            os.system('pause')
            sys.exit()
    
        # ch:打开设备 | en:Open device 
        ret = camera.MV3D_RGBD_OpenDevice(pointer(stDeviceList.DeviceInfo[int(nConnectionNum)]))
        if ret != 0:
            print("MV3D_RGBD_OpenDevice fail! ret[0x%x]" % ret)
            os.system('pause')
            sys.exit()

        # ch:注册回调函数 | en:Register the callback function
        camera.MV3D_RGBD_RegisterExceptionCallBack(CALL_BACK_FUN, None)

        # ch:开始取流 | en:Start grabbing
        ret = camera.MV3D_RGBD_Start()
        if ret != 0:
            print("start fail! ret[0x%x]" % ret)
            camera.MV3D_RGBD_CloseDevice()
            os.system('pause')
            sys.exit()
        
        bExit_main = False
        bSaveFrame = False
        stFrameData = MV3D_RGBD_FRAME_DATA()
        while bExit_main == False:
            # ch:获取图像数据 | en:Get image data
            ret = camera.MV3D_RGBD_FetchFrame(pointer(stFrameData),5000)
            if MV3D_RGBD_OK == ret:
                for i in range(0, stFrameData.nImageCount):
                    print("MV3D_RGBD_FetchFrame success: framenum (%d) height(%d) width(%d)  len (%d)!\r\n" %(stFrameData.stImageData[i].nFrameNum,
                    stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nDataLen))

                    if ((True == bSaveFrame) and (stFrameData.stImageData[i].pData is None) == False):
                        # ch:保存RAW图像 | en:Save raw image
                        if ImageType_Depth == stFrameData.stImageData[i].enImageType:
                            filename = "Depth_Frame[{:d}]_Width[{:d}]_Height[{:d}].raw".format(stFrameData.stImageData[i].nFrameNum, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight) 
                            strModeName = string_at(stFrameData.stImageData[i].pData,stFrameData.stImageData[i].nDataLen)
                        elif ImageType_RGB8_Planar == stFrameData.stImageData[i].enImageType:
                            filename = "RGB_Frame[{:d}]_Width[{:d}]_Height[{:d}].raw".format(stFrameData.stImageData[i].nFrameNum, stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight) 
                            strModeName = string_at(stFrameData.stImageData[i].pData,stFrameData.stImageData[i].nDataLen)
                        with open(filename,"wb") as f:
                            f.write(strModeName) 
                        
            if True == device_offline:
                print("Found device offline")
                break

            print("Please enter the action to be performed:\n")
            key = input("[q]Exit Current Frame [s]Save Photo [x]Exit Dev \n")
            if key == 'q':
                bExit_main = True
            
            if key == 's':
                bSaveFrame = True
            
            if key == 'x':
                bExit_main = True
                bLoop_exit = True

        if device_offline:
            print("device offline release resource")
        else:
            print("normal exit")
        


        # ch:停止取流 | en:Stop grabbing
        ret = camera.MV3D_RGBD_Stop()
        if ret != 0:
            print("stop fail! ret[0x%x]" % ret)
            os.system('pause')
            sys.exit()

        # ch:销毁句柄 | en:Destroy the device handle 
        ret = camera.MV3D_RGBD_CloseDevice()
        if ret != 0:
            print("CloseDevice fail! ret[0x%x]" % ret)
            os.system('pause')
            sys.exit()

    print("Main done !")   
    sys.exit()

