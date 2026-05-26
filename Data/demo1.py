"""
-------------------------------------
# -*- coding: utf-8 -*-
# @Time    : 2020/10/1 15:44:12
# @Author  : Giyn
# @Email   : giyn.jy@gmail.com
# @File    : video_processing.py
# @Software: PyCharm
-------------------------------------
"""

import cv2
import logging
import os

# log information settings
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s - %(levelname)s: %(message)s')


def save_image(num, image):
    """Save the images.
    Args:
        num: serial number
        image: image resource
    Returns:
        None
    """
    # 创建目录（如果不存在）
    os.makedirs('Data/raw_pictures_6', exist_ok=True)
    # 使用相对路径
    image_path = 'Data/raw_pictures_6/{}.jpg'.format(str(num))
    cv2.imwrite(image_path, image)


file_path = '6.avi'

vc = cv2.VideoCapture(file_path)  # import video files

# determine whether to open normally
if vc.isOpened():
    ret, frame = vc.read()
else:
    ret = False

count = 0  # count the number of pictures
frame_interval = 1  # video frame count interval frequency
frame_interval_count = 0

# loop read video frame
while ret:
    ret, frame = vc.read()
    # store operation every time f frame
    if frame_interval_count % frame_interval == 0:
        save_image(count, frame)
        logging.info("num：" + str(count) + ", frame: " +
                     str(frame_interval_count))
        count += 1
    frame_interval_count += 1
    cv2.waitKey(1)

vc.release()