def detection_logic_cam1(img, display="Camera1"):  # 相机1的检测逻辑（基于质心跟踪）
    global tracker, targets  # 引用全局跟踪器和目标字典
    global t1, t2, delta_t, t_computed, prev_cy  # 引用时间相关全局变量
    global active_object, last_line1_time, last_line2_time, passed_line2  # 引用目标状态全局变量
    global current_trajectory, recording, trajectory_id  # （未定义，可能是遗留变量）
    global predict_buffer, model, scaler, model_device, inference_lock  # （部分未定义，推理相关）
    global cz_mm, Panduan, LAST_FLAG, delay_time, frame_counter  # 引用深度、状态、延迟等全局变量
    global depth_value, depth_ready, SPEED_NOW  # 引用深度和速度全局变量

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # 将BGR格式转换为RGB格式（OpenCV默认BGR，此处可能冗余）
    now = time.time()  # 获取当前时间戳
    frame_counter += 1  # 帧计数器加1
    # active_object = False  # 注释：默认设置无活跃目标

    h, w = img.shape[:2]  # 获取图像的高度和宽度


    # ==== ROI 定义 ====
    roi_x_left = 20  # ROI左边界x坐标
    roi_x_right = 1660  # ROI右边界x坐标
    roi_w_px = roi_x_right - roi_x_left  # ROI宽度（像素）
    roi = img[:, roi_x_left:roi_x_right].copy()  # 截取ROI区域，深拷贝避免原图像修改

    # ✅✅ 相机1尺度 ✅✅
    real_width_cm = 60.09  # ROI对应的实际宽度（厘米）
    real_height_cm = 74.00  # 图像对应的实际高度（厘米）
    pixels_per_cm_x = roi_w_px / real_width_cm  # X方向每厘米对应的像素数
    pixels_per_cm_y = h / real_height_cm  # Y方向每厘米对应的像素数

    # ✅✅深度相机尺度✅✅
    pixels_per_mmx = 59.8 / 515 * 10  # X方向每毫米对应的像素数（根据深度相机校准）
    pixels_per_mmy = 50 / 435 * 10  # Y方向每毫米对应的像素数（根据深度相机校准）

    # ==== 坐标轴 ====
    cv2.line(img, (roi_x_right, 0), (0, 0), (0, 0, 0), 3)  # 绘制X轴基准线（顶部横线）
    for cm in range(0, int(real_width_cm) + 1, 5):  # 每隔5厘米绘制刻度
        x_pos = int(roi_x_right - cm * pixels_per_cm_x)  # 计算刻度的x坐标
        cv2.line(img, (x_pos, 0), (x_pos, 25), (0, 0, 0), 2)  # 绘制刻度线
        cv2.putText(img, f"{cm}", (x_pos - 25, 45),  # 绘制刻度值
                    cv2.FONT_HERSHEY_TRIPLEX, 1, (50, 50, 50), 2)  # 字体、大小、颜色、粗细

    cv2.line(img, (roi_x_right, 0), (roi_x_right, h), (0, 0, 0), 3)  # 绘制Y轴基准线（右侧竖线）
    for cm in range(0, int(real_height_cm) + 1, 5):  # 每隔5厘米绘制刻度
        y_pos = int(cm * pixels_per_cm_y)  # 计算刻度的y坐标
        cv2.line(img, (roi_x_right - 25, y_pos), (roi_x_right, y_pos), (0, 0, 0), 2)  # 绘制刻度线
        cv2.putText(img, f"{-cm}", (roi_x_right - 90, y_pos + 10),  # 绘制刻度值（负号表示方向）
                    cv2.FONT_HERSHEY_TRIPLEX, 1, (50, 50, 50), 2)  # 字体设置

    # 绘制判断线
    # ==== 两条检测线 ====🩷🩷
    trigger_line = {"bottom_y": 2 * h // 3, "middle_y": 2 * h // 5, "top_y": h // 5}  # 触发线位置字典：底边线、中线、顶边线
    cv2.line(img, (0, trigger_line["bottom_y"]), (roi_x_right, trigger_line["bottom_y"]), (255, 0, 0), 2)  # 绘制底边线（蓝色）
    cv2.putText(img, "bottom_y", (10, trigger_line["bottom_y"] - 10),  # 标注底边线
                cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0), 2)  # 绿色字体

    cv2.line(img, (0, trigger_line["top_y"]), (roi_x_right, trigger_line["top_y"]), (255, 0, 0), 2)  # 绘制顶边线（蓝色）
    cv2.putText(img, "top_y", (10, trigger_line["top_y"] - 10),  # 标注顶边线
                cv2.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 0), 2)  # 绿色字体

    with inference_lock:  # 加推理锁，确保多线程下模型调用安全
        # 可选择先下采样 roi 再推理（见下一节），此处示范直接推理
        try:
            with torch.no_grad():  # 禁用梯度计算，节省内存并加速推理
                results = model(roi, device="cuda", conf=0.8)  # 调用YOLO模型推理ROI区域，使用CUDA加速，置信度阈值0.8
        except Exception as e:  # 捕获推理异常
            print("[ERROR] 推理失败：", e)  # 打印错误信息
            return img  # 返回原图像

    r = results[0]  # 获取第一帧的推理结果


    if r.obb is not None and len(r.obb.xywhr) > 0:  # 若检测到旋转边界框（OBB）且数量大于0
        boxes = r.obb.xywhr.cpu().numpy()  # 获取OBB框坐标（中心x,y，宽w，高h，旋转角r），转CPU并转为numpy数组
        classes = r.obb.cls.cpu().numpy().astype(int)  # 获取类别ID，转CPU、numpy并转为整数
        confs = r.obb.conf.cpu().numpy()  # 获取置信度数组，转CPU并转为numpy数组

        for box, cls_id, conf in zip(boxes, classes, confs):  # 遍历每个检测框、类别ID和置信度
            # if conf < CONF_THRESH:
            #     continue  # 注释：置信度低于阈值时跳过（此处已在推理时设置conf=0.8，可省略）
            name = model.names[cls_id]  # 根据类别ID获取类别名称
            cx, cy, w, h, angle = box  # 解析OBB框参数

            angle_deg = math.degrees(angle)  # 将弧度转为角度
            # 保证以长边为参考
            if h > w:  # 若框的高度大于宽度（即长边为垂直方向）
                angle_deg += 90.0  # 角度加90度，统一以长边为基准
            # 归一化到 [-180, 180)
            angle_deg = (angle_deg + 180) % 360 - 180  # 角度归一化处理
            # 压缩到 [-90, 90]
            if angle_deg > 90:  # 若角度大于90度
                angle_deg -= 180  # 减180度
            elif angle_deg < -90:  # 若角度小于-90度
                angle_deg += 180  # 加180度
            angle_deg = -angle_deg  # 角度取反（根据实际坐标系调整）

            print(f"{name}: cx={cx:.1f}, cy={cy:.1f}, w={w:.1f}, h={h:.1f}, 长边角度={angle_deg:.1f}°,置信度={conf:.1f} ")  # 打印检测信息


            # 绘制质心和类别名
            cv2.circle(img, (int(cx), int(cy)), 6, (0, 0, 255), 2)  # 绘制质心（红色圆点，半径6，线宽2）
            cv2.putText(img, f"{name}", (int(cx) + 5, int(cy) - 5),  # 标注类别名
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)  # 绿色字体，大小1.0，线宽2

            rect = ((cx, cy), (w, h), math.degrees(angle))  # 构造旋转矩形参数：(中心点, 尺寸, 角度[度数])
            box_pts = cv2.boxPoints(rect)  # 计算旋转矩形的4个顶点坐标
            box_pts = np.int0(box_pts)  # 转换为整数坐标
            cv2.drawContours(img, [box_pts], 0, (0, 0, 255), 2)  # 绘制旋转边界框（红色，线宽2）

            # === 像素->毫米 ===
            cx_mm = -(cy / pixels_per_cm_y) * 10  # 将Y方向像素坐标转为毫米（负号调整方向）
            cy_mm = (roi_x_right - cx) / pixels_per_cm_x * 10  # 将X方向像素坐标转为毫米

            shendu_location = (1010 - int(cy_mm / pixels_per_mmx), 200 + int(-cx_mm / pixels_per_mmy))  # 计算深度相机的查询坐标
            # === 初始化 targets ===
            if targets is None:  # 若目标字典未初始化
                targets = {  # 初始化目标字典
                    "last_line1_time": None,  # 上一次经过第一条线的时间
                    "last_line2_time": None,  # 上一次经过第二条线的时间
                    "frame_count": 0,  # ✅ 帧计数器（记录目标被检测的帧数）
                    "trajectory": [],  # 目标轨迹
                }

            # === 更新帧计数 ===
            targets["frame_count"] += 1  # 目标的检测帧数加1

            # === 仅在第10帧调用一次深度线程 ===
            # now1 = time.time()
            # if targets["frame_count"] == 10:
            #     shendu_location = (1010 - int(cy_mm / pixels_per_mmx), 200 + int(-cx_mm / pixels_per_mmy))
            #     threading.Thread(
            #         target=async_get_depth,
            #         args=(shendu_location[0], shendu_location[1]),
            #         daemon=True
            #     ).start()
            #
            # with depth_lock:
            #     if depth_ready:
            #         cz_mm = depth_value
            #         now2 = time.time()
            #         print("[INFO] 第10帧，已触发一次深度获取", shendu_location, depth_value)
            #         print("高度获取花费时间：", now2 - now1)
            #         depth_ready = False
            if targets["frame_count"] == 10:  # 若目标被检测到第10帧-------------------------------------------------------------------------------------------------------------------------------
                # now1 = time.time()
                shendu_location = (1010 - int(cy_mm / pixels_per_mmx), 200 + int(-cx_mm / pixels_per_mmy))  # 计算深度查询坐标
                cz_mm = cam.get_height(shendu_location[0], shendu_location[1])  # 同步调用深度相机获取高度
                # now2 = time.time()
                cz_mm = 1190 - cz_mm  # 深度值校准（根据实际场景调整）
                print("[INFO] 第10帧，已触发深度获取", shendu_location, cz_mm)  # 打印深度信息
            # print("高度获取花费时间：", now2 - now1)

            label = f"({cx_mm:.1f}mm, {cy_mm:.1f}mm, {cz_mm:.1f}mm)"  # 构造坐标标签（x,y,z毫米）
            cv2.putText(img, label, (int(cx + 5), int(cy + 40)),  # 标注坐标
                        cv2.FONT_HERSHEY_TRIPLEX, 1.0, (0, 255, 0), 2)  # 绿色字体
            # bottom和middle线之间
            if cy > trigger_line["bottom_y"]:  # 若目标质心在底边线下方
                active_object = True  # 标记有活跃目标
                send_all_flag(0, 0, 0, 0, float_order="BADC")  # 发送所有标志位为0
                send_position_6axis(0, 0, 0, 0, 0, 0, float_order="BADC")  # 发送所有轴位置为0
                print(f"检测到物体")  # 打印信息
            if trigger_line["top_y"] <= cy <= trigger_line["bottom_y"] and active_object:  # 若目标在顶边线和底边线之间且为活跃目标

                if Panduan == None:  # 若判断变量未初始化
                    targets["last_line1_time"] = now  # 记录经过第一条线（底边线）的时间
                    Panduan = 1  # 设置判断标志为1

                # if last_line1_time is None or now - last_line1_time > DEBOUNCE_INTERVAL:❗❗❗
                print("成功发送坐标")  # 打印信息

                send_all_flag(1, 0, 0, 0, float_order="BADC")  # 发送标志位9为1，其余为0
                send_position_6axis(cx_mm, cy_mm, cz_mm, 0, 0, -angle_deg, float_order="BADC")  # 发送坐标和角度（角度取反）
                # send_position_6axis(cx_mm, cy_mm, cz_mm, 0, 0, angle_deg, float_order="BADC")  # 注释：原始角度发送方式
                print(f"物体坐标角度为{cx_mm, cy_mm, cz_mm, angle_deg}")  # 打印发送的参数

            elif cy < trigger_line["top_y"] and active_object:  # 若目标在顶边线上方且为活跃目标
                send_all_flag(1, 1, 0, 1, float_order="BADC")  # 发送标志位9、10、12为1，11为0
                targets["last_line2_time"] = now  # 记录经过第二条线（顶边线）的时间
                time_interval = targets["last_line2_time"] - targets["last_line1_time"]  # 计算经过两条线的时间差
                # time_interval = round(time_interval, 2)  # 注释：时间差保留两位小数
                distance = (2/3 - 1/5) * real_height_cm  # 计算两条线之间的实际距离（厘米）
                print("时间间隔，距离", time_interval, distance, targets["last_line2_time"], targets["last_line1_time"])  # 打印时间差和距离
                SPEED_NOW = distance/time_interval  # 计算目标速度（厘米/秒）
                # send_position_6axis(cx_mm, cy_mm, cz_mm, SPEED_NOW, 0, -angle_deg,  float_order="BADC")  # 注释：速度发送到SPEED寄存器
                send_position_6axis(cx_mm, cy_mm, cz_mm, 0, SPEED_NOW * 10, -angle_deg, float_order="BADC")  # 速度×10后发送到RY寄存器
                # 24.64
                delay_time = 57 / SPEED_NOW  # 计算延迟时间（57为经验参数，单位厘米）
                print("速度为", SPEED_NOW)  # 打印当前速度
                print("停止发送坐标,delay_time", delay_time)  # 打印延迟时间
                active_object = False  # 标记无活跃目标
                LAST_FLAG = True  # 标记最后一次触发
                # send_all_flag(1, 1, 0)  # 注释：发送标志位的旧方式
    else:  # 未检测到OBB
        print("No OBB detected")  # 打印提示