# -- coding: utf-8 --
import threading
import ctypes
import time
import os
import sys
from ctypes import *
import numpy as np
import cv2

from Mv3dRgbdImport.Mv3dRgbdDefine import *
from Mv3dRgbdImport.Mv3dRgbdApi import *
from Mv3dRgbdImport.Mv3dRgbdDefine import (
    DeviceType_Ethernet, DeviceType_USB, DeviceType_Ethernet_Vir, DeviceType_USB_Vir,
    MV3D_RGBD_FLOAT_Z_UNIT, ParamType_Float, ParamType_Int, ParamType_Enum, CoordinateType_Depth
)

g_bExit = False

def to_ndarray(st_img):
    """
    将 SDK 返回的图像数据转成 numpy 数组，并返回 (frame, kind)
    kind: 'bgr8' | 'mono8' | 'depth16' | 'unknown'
    """
    w = int(st_img.nWidth)
    h = int(st_img.nHeight)
    size = int(st_img.nDataLen)

    if not bool(st_img.pImageBuf):
        return None, 'unknown'

    # 取出底层缓冲
    cbuf_t = ctypes.c_ubyte * size
    cbuf = ctypes.cast(st_img.pImageBuf, POINTER(cbuf_t)).contents
    npbuf = np.frombuffer(cbuf, dtype=np.uint8)

    # 根据长度猜测通道/位深（常见情况：Mono8 / BGR8 / Depth16）
    if size == w * h:  # 8-bit 单通道
        frame = npbuf.reshape(h, w).copy()
        return frame, 'mono8'
    elif size == w * h * 3:  # 8-bit 3通道（通常 BGR8）
        frame = npbuf.reshape(h, w, 3).copy()
        return frame, 'bgr8'
    elif size == w * h * 2:  # 16-bit 单通道（常用于深度）
        frame = npbuf.view(np.uint16).reshape(h, w).copy()
        return frame, 'depth16'
    else:
        # 如有 stride 或其他格式，可在此基于 st_img.enPixelType 做更细分处理
        return None, 'unknown'

def visualize(frame, kind, win_prefix='Cam'):
    """
    将不同类型的图像显示出来；深度图做伪彩
    """
    if kind == 'bgr8':
        cv2.imshow(f'{win_prefix}-Color', frame)
    elif kind == 'mono8':
        cv2.imshow(f'{win_prefix}-Mono/IR', frame)
    elif kind == 'depth16':
        depth = frame
        # 将 16-bit 深度映射到 8-bit 以便显示
        # 自动拉伸对比；你也可以基于相机量程固定缩放（比如 0~4000mm）
        if depth.max() > 0:
            dep8 = cv2.convertScaleAbs(depth, alpha=255.0 / depth.max())
        else:
            dep8 = np.zeros_like(depth, dtype=np.uint8)
        dep_color = cv2.applyColorMap(dep8, cv2.COLORMAP_JET)
        cv2.imshow(f'{win_prefix}-Depth', dep_color)
    # 其他格式：忽略

def work_thread(camera):
    global g_bExit
    t0, frames = time.time(), 0

    while not g_bExit:
        stFrameData = MV3D_RGBD_FRAME_DATA()
        ret = camera.MV3D_RGBD_FetchFrame(pointer(stFrameData), 5000)
        if ret == 0:
            for i in range(0, stFrameData.nImageCount):
                st_img = stFrameData.stImageData[i]
                frame, kind = to_ndarray(st_img)
                if frame is None:
                    # 退回日志输出
                    print("Unknown image format: idx=%d, W=%d, H=%d, Len=%d"
                          % (i, st_img.nWidth, st_img.nHeight, st_img.nDataLen))
                    continue
                visualize(frame, kind, win_prefix='MV3D')
            frames += 1
        else:
            print("no data[0x%x]" % ret)

        # 处理键盘事件（在此而不是主线程，确保窗口可响应）
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q') or k == 27:  # q 或 ESC 退出
            g_bExit = True

        # 可选：打印 FPS
        if frames % 30 == 0:
            now = time.time()
            if now - t0 > 0:
                fps = frames / (now - t0)
                print("FPS: %.1f" % fps)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    # 1) 枚举设备
    nDeviceNum = ctypes.c_uint(0)
    nDeviceNum_p = byref(nDeviceNum)
    ret = Mv3dRgbd.MV3D_RGBD_GetDeviceNumber(
        DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir, nDeviceNum_p)
    if ret != 0:
        print("MV3D_RGBD_GetDeviceNumber fail! ret[0x%x]" % ret)
        sys.exit(1)
    if nDeviceNum.value == 0:
        print("find no device!")
        sys.exit(1)

    print("Find devices numbers:", nDeviceNum.value)
    stDeviceList = MV3D_RGBD_DEVICE_INFO_LIST()
    _ = Mv3dRgbd.MV3D_RGBD_GetDeviceList(
        DeviceType_Ethernet | DeviceType_USB | DeviceType_Ethernet_Vir | DeviceType_USB_Vir,
        pointer(stDeviceList.DeviceInfo[0]), 20, nDeviceNum_p)

    for i in range(0, nDeviceNum.value):
        print("\ndevice: [%d]" % i)
        model = "".join(chr(c) for c in stDeviceList.DeviceInfo[i].chModelName)
        sn = "".join(chr(c) for c in stDeviceList.DeviceInfo[i].chSerialNumber)
        print("device model name:", model.strip("\x00"))
        print("device SerialNumber:", sn.strip("\x00"))

    # 2) 选择并打开
    camera = Mv3dRgbd()
    nConnectionNum = input("please input the number of the device to connect:")
    if int(nConnectionNum) >= nDeviceNum.value:
        print("input error!")
        sys.exit(1)

    ret = camera.MV3D_RGBD_OpenDevice(pointer(stDeviceList.DeviceInfo[int(nConnectionNum)]))
    if ret != 0:
        print("MV3D_RGBD_OpenDevice fail! ret[0x%x]" % ret)
        sys.exit(1)

    # 可选：输出点云坐标制切到“深度图”模式（如果 SDK 支持）
    try:
        camera.MV3D_RGBD_SetCoordinateType(CoordinateType_Depth)
    except Exception:
        pass  # 某些固件/版本可能没有该接口

    # 3) 开始取流
    ret = camera.MV3D_RGBD_Start()
    if ret != 0:
        print("start fail! ret[0x%x]" % ret)
        camera.MV3D_RGBD_CloseDevice()
        sys.exit(1)

    # 4) 显示线程
    try:
        hthreadhandle = threading.Thread(target=work_thread, args=(camera,), daemon=True)
        hthreadhandle.start()
    except:
        print("error: unable to start thread")
        camera.MV3D_RGBD_Stop()
        camera.MV3D_RGBD_CloseDevice()
        sys.exit(1)

    # 主线程等待直到退出信号
    try:
        while not g_bExit:
            time.sleep(0.05)
    except KeyboardInterrupt:
        g_bExit = True

    hthreadhandle.join()

    # 5) 停止与关闭
    ret = camera.MV3D_RGBD_Stop()
    if ret != 0:
        print("stop fail! ret[0x%x]" % ret)

    ret = camera.MV3D_RGBD_CloseDevice()
    if ret != 0:
        print("CloseDevice fail! ret[0x%x]" % ret)
