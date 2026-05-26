"""
Flask应用程序主文件
功能：
1. 提供RESTful API接口用于数据库操作（增删改查）
2. 定时任务：每秒自动生成10个随机数据并插入数据库
3. 在子线程中运行Flask服务器，保持主线程可执行其他任务
"""

# ==================== 导入依赖模块 ====================
from flask import Flask, session  # Flask框架核心类，session用于会话管理
from flask import request, redirect, url_for  # request获取请求数据，redirect重定向，url_for生成URL
from flask import jsonify, Response  # 将Python字典转换为JSON响应，Response用于视频流
from config import Config  # 导入配置文件
from flask_cors import CORS  # 跨域资源共享，允许前端跨域访问
from mysql import db, Object  # 导入数据库实例和User模型类
from datetime import datetime  # 日期时间处理
from apscheduler.schedulers.background import BackgroundScheduler  # 后台任务调度器
from apscheduler.triggers.interval import IntervalTrigger  # 间隔触发器，用于定时任务
from methods import insert_real_data  # 导入插入随机数据的函数
import go  # 导入自定义的go模块（全局变量）
import threading  # 导入线程模块，用于多线程处理
import cv2  # OpenCV用于摄像头访问
import numpy as np  # NumPy用于图像处理
import os  # 操作系统接口，用于文件路径操作
import time
from threading import Lock
from flask import send_file
from flask import Flask
from flask_socketio import SocketIO



app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")  # 允许跨域

# ==================== 全局变量初始化 ====================
go.redirect = 0  # 初始化全局重定向标志位（0=不重定向，1=重定向到delete_tables）

#截图
snapshot_lock = Lock()  # 保证多线程安全
latest_snapshot_path = None  # 全局变量保存截图路径
SNAPSHOT_DIR = os.path.join("static", "snapshots")
MAX_SNAPSHOTS = 20  # 最多保留的截图数，可按需调整

# ==================== Flask应用初始化 ====================
# 创建Flask应用实例，__name__用于确定应用根路径
myserver = Flask(__name__)
myserver.config.from_object(Config)  # 从Config类加载配置
CORS(myserver)  # 启用跨域支持，允许所有来源访问API
db.init_app(myserver)  # 将数据库实例与Flask应用绑定
current_time = datetime.now()  # 获取当前时间，用于创建数据时的时间戳

# ==================== 定时任务调度器配置 ====================
# 创建后台调度器，用于执行定时任务
scheduler = BackgroundScheduler()

# 添加定时任务：每秒执行一次insert_real_data函数
# func: 要执行的函数
# trigger: 触发器类型，IntervalTrigger表示间隔触发
# seconds: 间隔秒数（1秒）
# start_date: 任务开始时间（当前时间）
# id: 任务唯一标识符
# name: 任务名称（便于管理）
# replace_existing: 如果任务已存在则替换
scheduler.add_job(
    func=insert_real_data,
    trigger=IntervalTrigger(seconds=1, start_date=current_time),
    id='insert_real_data_job',
    name='每秒插入一次真实数据',
    replace_existing=True,
    max_instances=3,  # 允许最多3个实例同时运行
)

# ==================== 路由定义（API接口） ====================
@myserver.route('/api/output/create_tables', methods=['GET', 'POST'])
def create_tables():
    """
    API接口：创建数据库表
    方法：GET 或 POST
    功能：根据模型定义创建数据库表（如果表不存在则创建，已存在则不操作）
    返回：JSON格式的成功消息或错误信息
    """
    try:
        # 需要在应用上下文中执行数据库操作
        with myserver.app_context():
            print("开始创建表...")
            db.create_all()  # 创建所有定义的表
            print("创建完成")
        return jsonify({"message": '数据库表已成功创建（如果不存在）'})
    except Exception as e:
        # 如果出错，返回500状态码和错误信息
        return jsonify({"error": f"建表失败: {str(e)}"}), 500

@myserver.route('/api/output/find_data', methods=['GET', 'POST'])
def find_data():
    """
    API接口：查询所有数据
    方法：GET 或 POST
    功能：从数据库查询所有Object记录，转换为JSON格式返回给前端
    返回：JSON格式的查询结果，包含success标志、消息、记录数量和data数组
    """
    try:
        # 从数据库查询所有Object记录
        objects = Object.query.all()

        # 将数据库对象转换为字典格式，方便前端使用
        result = []
        for object in objects:
            result.append({
                "id": object.id,  # 用户ID
                "location_x": object.location_x,
                "location_y": object.location_y,
                "location_z": object.location_z,
                "location_rz": object.location_rz,
                "speed": object.speed,
                "confidence": object.confidence,
                "count":object.count,
                "time": object.time.strftime('%Y-%m-%d %H:%M:%S') if object.time else None  # 时间（格式化或None）
            })

        # 返回JSON数据给前端
        return jsonify({
            "success": True,
            "message": "查询成功",
            "count": len(result),  # 记录总数
            "data": result  # 数据数组
        })
    except Exception as e:
        # 如果查询失败，返回错误信息
        return jsonify({
            "success": False,
            "error": f"查询失败: {str(e)}"
        }), 500

@myserver.route('/api/output/find_data_by_id', methods=['GET', 'POST'])
def find_data_by_id():
    """
    API接口：根据ID查询单条数据
    方法：GET 或 POST
    参数：id (查询参数或JSON body中的id字段)
    功能：从数据库查询指定ID的Object记录
    返回：JSON格式的查询结果
    """
    try:
        # 获取ID参数（支持GET查询参数和POST JSON body）
        if request.method == 'GET':
            id_param = request.args.get('id')
        else:
            id_param = request.json.get('id') if request.json else None

        if id_param is None:
            return jsonify({
                "success": False,
                "error": "缺少参数id"
            }), 400

        try:
            id_value = int(id_param)
        except ValueError:
            return jsonify({
                "success": False,
                "error": "id必须是有效的数字"
            }), 400

        # 从数据库查询指定ID的记录
        object = Object.query.get(id_value)

        if object is None:
            return jsonify({
                "success": False,
                "error": f"未找到ID为{id_value}的记录"
            }), 404

        # 将数据库对象转换为字典格式
        result = {
            "id": object.id,
            "location_x": object.location_x,
            "location_y": object.location_y,
            "location_z": object.location_z,
            "location_rz": object.location_rz,
            "count": object.count,
            "speed": object.speed,
            "confidence": object.confidence,
            "time": object.time.strftime('%Y-%m-%d %H:%M:%S') if object.time else None
        }

        # 返回JSON数据给前端
        return jsonify({
            "success": True,
            "message": "查询成功",
            "data": result
        })
    except Exception as e:
        # 如果查询失败，返回错误信息
        return jsonify({
            "success": False,
            "error": f"查询失败: {str(e)}"
        }), 500

# ==================== 查询图片相关 ====================
# 全局变量：jpg对象
def get_image_directory():
    """
    获取图片目录路径
    返回：图片目录的绝对路径
    """
    # 获取项目根目录（app.py所在目录）
    project_root = os.path.dirname(os.path.abspath(__file__))
    # 构建image目录路径
    image_dir = os.path.join(project_root, 'image')
    return image_dir
def get_image_content_type(filename):
    """
    根据文件扩展名获取Content-Type
    参数：filename - 文件名
    返回：MIME类型字符串
    """
    ext = os.path.splitext(filename.lower())[1]
    content_types = {
        '.jpg': 'image/jpeg',
        '.jpeg': 'image/jpeg',
        '.png': 'image/png',
        '.gif': 'image/gif',
        '.bmp': 'image/bmp',
        '.webp': 'image/webp'
    }
    return content_types.get(ext, 'application/octet-stream')
@myserver.route('/api/output/images/<int:id>/<filename>', methods=['GET'])
def get_image(id, filename):
    """
    API接口：获取识别结果图片
    方法：GET
    参数：id (路径参数，数据ID), filename (路径参数，图片文件名，如image.jpg或1.jpg)
    功能：从项目根目录下的image文件夹返回图片文件
    返回：图片文件或404错误
    示例：
        GET /api/output/images/1/image.jpg  -> 查找 image/1.jpg
        GET /api/output/images/1/1.jpg      -> 直接返回 image/1.jpg
    """
    from flask import send_from_directory

    # 获取图片目录路径
    image_base_dir = get_image_directory()

    # 支持的图片格式列表
    image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.gif', '.webp']

    # 调试信息
    print(f"📸 图片请求: ID={id}, filename={filename}")
    print(f"📁 图片目录: {image_base_dir}")

    # 检查目录是否存在
    if not os.path.exists(image_base_dir):
        print(f"❌ 错误：图片目录不存在: {image_base_dir}")
        return jsonify({
            "success": False,
            "error": f"图片目录不存在: {image_base_dir}"
        }), 404

    # 如果filename是通用名称（如image.jpg），则根据id查找对应的图片
    if filename.lower() in ['image.jpg', 'image.png', 'image.jpeg', 'image.bmp', 'image.gif', 'image.webp']:
        # 根据id查找对应的图片文件（尝试多种格式）
        image_file = None
        for ext in image_extensions:
            potential_file = os.path.join(image_base_dir, f"{id}{ext}")
            if os.path.exists(potential_file) and os.path.isfile(potential_file):
                image_file = f"{id}{ext}"
                print(f"✅ 找到图片: {image_file}")
                break

        if not image_file:
            print(f"❌ 未找到ID为{id}的图片文件（尝试了所有格式）")
            return jsonify({
                "success": False,
                "error": f"未找到ID为{id}的图片文件"
            }), 404
    else:
        # 如果filename是具体的文件名，直接使用
        image_file = filename
        image_path = os.path.join(image_base_dir, image_file)

        # 检查文件是否存在
        if not os.path.exists(image_path) or not os.path.isfile(image_path):
            print(f"❌ 文件不存在: {image_file}")
            return jsonify({
                "success": False,
                "error": f"图片不存在: {image_file}"
            }), 404

    # 发送图片文件
    try:
        full_path = os.path.join(image_base_dir, image_file)
        file_size = os.path.getsize(full_path)
        print(f"📤 发送图片: {image_file} ({file_size} bytes)")

        response = send_from_directory(image_base_dir, image_file)

        # 设置正确的Content-Type
        response.headers['Content-Type'] = get_image_content_type(image_file)

        # 设置缓存控制（可选：缓存1小时）
        response.headers['Cache-Control'] = 'public, max-age=3600'

        # 允许跨域
        response.headers['Access-Control-Allow-Origin'] = '*'

        print(f"✅ 图片发送成功")
        return response

    except Exception as e:
        print(f"❌ 发送图片失败: {str(e)}")
        import traceback
        traceback.print_exc()
        return jsonify({
            "success": False,
            "error": f"发送图片失败: {str(e)}"
        }), 500
@myserver.route('/api/output/list_images', methods=['GET'])
def list_images():
    """
    API接口：获取文件夹内的所有图片列表
    方法：GET
    参数：
        limit (可选): 限制返回数量，如 ?limit=10
        sort (可选): 排序方式，'id' 或 'name'，默认 'id'
    功能：从项目根目录下的image文件夹返回所有图片文件列表
    返回：JSON格式的图片列表，包含文件名、ID、大小、URL等信息
    """
    # 获取图片目录路径
    image_base_dir = get_image_directory()

    # 支持的图片格式列表
    image_extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.gif', '.webp']

    # 获取查询参数
    limit = request.args.get('limit', type=int)
    sort_by = request.args.get('sort', 'id', type=str).lower()

    print(f"📋 请求图片列表，目录: {image_base_dir}")

    # 检查目录是否存在
    if not os.path.exists(image_base_dir):
        print(f"❌ 错误：图片目录不存在: {image_base_dir}")
        return jsonify({
            "success": False,
            "error": f"图片目录不存在: {image_base_dir}",
            "images": []
        }), 404

    try:
        # 获取目录下所有文件
        all_files = os.listdir(image_base_dir)

        # 筛选出图片文件
        image_files = []
        for filename in all_files:
            file_path = os.path.join(image_base_dir, filename)

            # 只处理文件，忽略目录
            if not os.path.isfile(file_path):
                continue

            # 检查文件扩展名
            _, ext = os.path.splitext(filename.lower())
            if ext not in image_extensions:
                continue

            # 尝试从文件名提取ID（假设文件名是数字+扩展名，如 1.jpg）
            file_id = None
            try:
                name_without_ext = os.path.splitext(filename)[0]
                file_id = int(name_without_ext)
            except ValueError:
                # 如果无法解析为数字，跳过或使用0
                pass

            # 获取文件大小和修改时间
            file_size = os.path.getsize(file_path)
            file_mtime = os.path.getmtime(file_path)

            image_files.append({
                "filename": filename,
                "id": file_id,
                "size": file_size,
                "size_mb": round(file_size / (1024 * 1024), 2),  # 转换为MB
                "modified_time": datetime.fromtimestamp(file_mtime).strftime('%Y-%m-%d %H:%M:%S'),
                "url": f"/api/output/images/{file_id if file_id is not None else 0}/{filename}"
            })

        # 排序
        if sort_by == 'name':
            image_files.sort(key=lambda x: x['filename'])
        else:  # 默认按ID排序
            image_files.sort(key=lambda x: (x['id'] is not None, x['id'] if x['id'] is not None else float('inf')))

        # 如果指定了limit，则限制返回数量
        if limit and limit > 0:
            image_files = image_files[:limit]

        print(f"✅ 找到 {len(image_files)} 张图片")

        return jsonify({
            "success": True,
            "count": len(image_files),
            "images": image_files,
            "directory": image_base_dir
        })

    except Exception as e:
        print(f"❌ 获取图片列表失败: {str(e)}")
        import traceback
        traceback.print_exc()
        return jsonify({
            "success": False,
            "error": f"获取图片列表失败: {str(e)}",
            "images": []
        }), 500

# ==================== MP4视频流相关 ====================
# 全局变量：MP4对象
@myserver.route('/api/output/videos/<filename>', methods=['GET', 'HEAD', 'OPTIONS'])
def get_video(filename):
    """
    API接口：获取MP4视频文件
    方法：GET
    参数：filename (路径参数，视频文件名，如test.mp4)
    功能：从指定文件夹返回MP4视频文件，支持HTTP Range请求（用于视频流式播放）
    返回：视频文件或404错误
    """
    import os
    from flask import send_from_directory, request, make_response
    from urllib.parse import unquote

    # 视频文件夹路径
    video_base_dir = r'C:\Users\zhang\xwechat_files\wxid_3asrw9vexlh822_be6f\msg\file\2025-11\后端\flaskProject1\video'

    # URL解码文件名（处理特殊字符）
    filename = unquote(filename)

    # 处理OPTIONS预检请求（CORS）
    if request.method == 'OPTIONS':
        response = make_response()
        response.headers['Access-Control-Allow-Origin'] = '*'
        response.headers['Access-Control-Allow-Methods'] = 'GET, HEAD, OPTIONS'
        response.headers['Access-Control-Allow-Headers'] = 'Range, Content-Type'
        response.headers['Access-Control-Max-Age'] = '3600'
        return response

    print(f"🎬 视频请求: method={request.method}, filename={filename}")

    # 确保是mp4格式
    if not filename.lower().endswith('.mp4'):
        print(f"❌ 不支持的文件格式: {filename}")
        return jsonify({
            "success": False,
            "error": "只支持MP4格式视频"
        }), 400

    # 检查目录是否存在
    if not os.path.exists(video_base_dir):
        print(f"❌ 视频目录不存在: {video_base_dir}")
        return jsonify({
            "success": False,
            "error": f"视频目录不存在: {video_base_dir}"
        }), 404

    # 构建视频文件路径
    video_path = os.path.join(video_base_dir, filename)
    print(f"📁 视频路径: {video_path}")

    # 检查文件是否存在
    if not os.path.exists(video_path) or not os.path.isfile(video_path):
        print(f"❌ 视频文件不存在: {video_path}")
        return jsonify({
            "success": False,
            "error": f"视频不存在: {filename}"
        }), 404

    try:
        file_size = os.path.getsize(video_path)
        print(f"✅ 视频文件大小: {file_size} bytes")

        # 处理Range请求（用于视频流式播放）
        range_header = request.headers.get('Range', None)
        print(f"📡 Range请求头: {range_header}")

        if range_header:
            # 解析Range请求头
            byte_start = 0
            byte_end = file_size - 1

            try:
                range_match = range_header.replace('bytes=', '').split('-')
                if range_match[0]:
                    byte_start = int(range_match[0])
                if len(range_match) > 1 and range_match[1]:
                    byte_end = int(range_match[1])
            except (ValueError, IndexError) as e:
                print(f"⚠️ Range解析错误: {e}")

            byte_start = max(0, byte_start)
            byte_end = min(file_size - 1, byte_end)
            content_length = byte_end - byte_start + 1

            print(f"📊 Range: {byte_start}-{byte_end}, Content-Length: {content_length}")

            # 读取文件片段
            with open(video_path, 'rb') as f:
                f.seek(byte_start)
                data = f.read(content_length)

            # 创建206 Partial Content响应
            response = make_response(data, 206)
            response.headers['Content-Range'] = f'bytes {byte_start}-{byte_end}/{file_size}'
            response.headers['Accept-Ranges'] = 'bytes'
            response.headers['Content-Length'] = str(content_length)
            response.headers['Content-Type'] = 'video/mp4'
            response.headers['Access-Control-Allow-Origin'] = '*'
            response.headers['Access-Control-Allow-Methods'] = 'GET, HEAD, OPTIONS'
            response.headers['Access-Control-Allow-Headers'] = 'Range'
            print(f"✅ 返回206 Partial Content响应")

            # HEAD请求只返回头部，不返回内容
            if request.method == 'HEAD':
                response = make_response('', 206)
                response.headers['Content-Range'] = f'bytes {byte_start}-{byte_end}/{file_size}'
                response.headers['Accept-Ranges'] = 'bytes'
                response.headers['Content-Length'] = str(content_length)
                response.headers['Content-Type'] = 'video/mp4'
                response.headers['Access-Control-Allow-Origin'] = '*'
                response.headers['Access-Control-Allow-Methods'] = 'GET, HEAD, OPTIONS'
                response.headers['Access-Control-Allow-Headers'] = 'Range'
                return response

            return response
        else:
            # 返回整个文件
            print(f"📤 返回完整文件")
            response = send_from_directory(video_base_dir, os.path.basename(video_path))
            response.headers['Content-Type'] = 'video/mp4'
            response.headers['Accept-Ranges'] = 'bytes'
            response.headers['Content-Length'] = str(file_size)
            response.headers['Access-Control-Allow-Origin'] = '*'
            response.headers['Access-Control-Allow-Methods'] = 'GET, HEAD, OPTIONS'
            response.headers['Access-Control-Allow-Headers'] = 'Range'

            # HEAD请求只返回头部，不返回内容
            if request.method == 'HEAD':
                response = make_response('', 200)
                response.headers['Content-Type'] = 'video/mp4'
                response.headers['Accept-Ranges'] = 'bytes'
                response.headers['Content-Length'] = str(file_size)
                response.headers['Access-Control-Allow-Origin'] = '*'
                response.headers['Access-Control-Allow-Methods'] = 'GET, HEAD, OPTIONS'
                response.headers['Access-Control-Allow-Headers'] = 'Range'
                return response

            return response

    except Exception as e:
        import traceback
        error_msg = str(e)
        traceback_str = traceback.format_exc()
        print(f"❌ 发送视频失败: {error_msg}")
        print(f"📋 错误详情:\n{traceback_str}")
        return jsonify({
            "success": False,
            "error": f"发送视频失败: {error_msg}"
        }), 500

@myserver.route('/api/output/list_videos', methods=['GET'])
def list_videos():
    """
    API接口：获取文件夹内的所有MP4视频列表
    方法：GET
    参数：可选的limit参数限制返回数量
    功能：从指定文件夹返回所有MP4视频文件列表
    返回：JSON格式的视频列表
    """
    import os

    # 视频文件夹路径
    video_base_dir = r'C:\Users\zhang\xwechat_files\wxid_3asrw9vexlh822_be6f\msg\file\2025-11\后端\flaskProject1\video\test.mp4'

    # 获取可选的limit参数
    limit = request.args.get('limit', type=int)

    # 检查目录是否存在
    if not os.path.exists(video_base_dir):
        return jsonify({
            "success": False,
            "error": f"视频目录不存在: {video_base_dir}",
            "videos": []
        }), 404

    try:
        # 获取目录下所有MP4文件
        video_files = []
        for filename in os.listdir(video_base_dir):
            if filename.lower().endswith('.mp4'):
                file_path = os.path.join(video_base_dir, filename)
                if os.path.isfile(file_path):
                    file_size = os.path.getsize(file_path)
                    video_files.append({
                        "filename": filename,
                        "size": file_size,
                        "url": f"/api/output/videos/{filename}"
                    })

        # 按文件名排序
        video_files.sort(key=lambda x: x['filename'])

        # 如果指定了limit，则限制返回数量
        if limit and limit > 0:
            video_files = video_files[:limit]

        return jsonify({
            "success": True,
            "count": len(video_files),
            "videos": video_files,
            "directory": video_base_dir
        })

    except Exception as e:
        print(f"❌ 获取视频列表失败: {str(e)}")
        return jsonify({
            "success": False,
            "error": f"获取视频列表失败: {str(e)}",
            "videos": []
        }), 500

#截图
def cleanup_old_snapshots(folder, max_files=MAX_SNAPSHOTS):
    """
    自动清理旧截图文件，只保留最新 max_files 个
    """
    try:
        files = [os.path.join(folder, f) for f in os.listdir(folder) if f.endswith(".jpg")]
        files.sort(key=os.path.getmtime, reverse=True)
        for f in files[max_files:]:
            os.remove(f)
            print(f"🧹 已删除旧截图: {f}")
    except Exception as e:
        print(f"⚠️ 清理旧截图时出错: {str(e)}")

# ========== 截图触发接口 ==========
@myserver.route('/api/snapshot/trigger', methods=['POST'])
def trigger_snapshot():
    """
    前端发送信号 {signal: 1} 时触发截图
    """
    global latest_snapshot_path
    try:
        with snapshot_lock:
            # 读取当前视频帧（此处使用摄像头0，可替换为你的视频流类）
            cap = cv2.VideoCapture(0)
            ret, frame = cap.read()
            cap.release()

            if not ret:
                return jsonify({"status": "error", "message": "无法读取视频流帧"}), 500

            # 创建保存目录
            os.makedirs(SNAPSHOT_DIR, exist_ok=True)

            # 保存图片文件
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"snapshot_{timestamp}.jpg"
            full_path = os.path.join(SNAPSHOT_DIR, filename)
            cv2.imwrite(full_path, frame)

            latest_snapshot_path = full_path
            print(f"📸 已生成截图: {full_path}")

            # 清理旧文件
            cleanup_old_snapshots(SNAPSHOT_DIR)

        return jsonify({
            "status": "success",
            "message": "截图成功",
            "path": "/api/snapshot/latest"
        }), 200

    except Exception as e:
        return jsonify({"status": "error", "message": str(e)}), 500

# ========== 输出最新截图 ==========
@myserver.route('/api/snapshot/latest', methods=['GET'])
def get_latest_snapshot():
    """
    返回最近一次截图
    """
    global latest_snapshot_path
    if latest_snapshot_path and os.path.exists(latest_snapshot_path):
        return send_file(latest_snapshot_path, mimetype='image/jpeg')
    else:
        return jsonify({"status": "error", "message": "尚未截图"}), 404

# ==================== 摄像头视频流相关 ====================
# 全局变量：摄像头对象
camera = None

def init_camera():
    """
    初始化摄像头
    功能：打开默认摄像头（索引0）
    返回：摄像头对象或None
    """
    global camera
    try:
        if camera is None:
            camera = cv2.VideoCapture(0)  # 0表示默认摄像头
            if not camera.isOpened():
                print("⚠️ 警告：无法打开摄像头，请检查摄像头是否连接")
                return None
            # 设置摄像头分辨率（可选）
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            print("✅ 摄像头初始化成功")
        return camera
    except Exception as e:
        print(f"❌ 摄像头初始化失败: {str(e)}")
        return None

def generate_frames():
    """
    生成视频帧的生成器函数
    功能：持续从摄像头读取帧，编码为JPEG格式，用于视频流传输
    返回：生成器，每次yield一个JPEG编码的帧
    """
    global camera

    # 创建错误帧（摄像头不可用时显示）
    def create_error_frame(message="Camera Not Available"):
        error_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(error_frame, message, (50, 220),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(error_frame, "Please check camera connection", (50, 260),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        _, buffer = cv2.imencode('.jpg', error_frame)
        return buffer.tobytes()

    # 初始化摄像头
    camera = init_camera()
    retry_count = 0
    max_retries = 3

    while True:
        try:
            # 如果摄像头未初始化或读取失败，尝试重新初始化
            if camera is None or not camera.isOpened():
                if retry_count < max_retries:
                    print(f"⚠️ 尝试重新初始化摄像头 ({retry_count + 1}/{max_retries})...")
                    camera = None  # 重置全局变量
                    camera = init_camera()
                    retry_count += 1
                    if camera is None:
                        # 返回错误帧
                        error_bytes = create_error_frame("Camera Initialization Failed")
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + error_bytes + b'\r\n')
                        import time
                        time.sleep(1)  # 等待1秒后重试
                        continue
                else:
                    # 超过最大重试次数，持续返回错误帧
                    error_bytes = create_error_frame("Camera Not Available")
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + error_bytes + b'\r\n')
                    import time
                    time.sleep(1)
                    continue

            # 重置重试计数（成功初始化后）
            retry_count = 0

            # 读取摄像头帧
            success, frame = camera.read()
            if not success:
                print("⚠️ 无法读取摄像头帧，尝试重新初始化...")
                camera.release()
                camera = None
                continue

            # 将帧编码为JPEG格式
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ret:
                continue

            # 转换为字节流
            frame_bytes = buffer.tobytes()

            # 生成MJPEG格式的视频流
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

        except Exception as e:
            print(f"❌ 生成视频帧时出错: {str(e)}")
            # 发生异常时返回错误帧，而不是中断流
            try:
                error_bytes = create_error_frame(f"Error: {str(e)[:30]}")
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + error_bytes + b'\r\n')
            except:
                pass
            import time
            time.sleep(1)  # 等待后继续

@myserver.route('/api/output/video_feed')
def video_feed():
    """
    API接口：视频流端点
    方法：GET
    功能：提供实时摄像头视频流（MJPEG格式）
    返回：视频流响应（multipart/x-mixed-replace）
    使用：前端可以通过 <img src="/api/output/video_feed"> 或 <video> 标签显示
    """
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# ==================== 辅助函数 ====================

def run_flask_server():
    """
    在子线程中运行Flask服务器的函数
    功能：启动Flask开发服务器，监听所有网络接口（0.0.0.0）的5000端口
    注意：在子线程中运行，避免阻塞主线程
    """
    # 打印启动信息和API接口列表
    print("=" * 50)
    print("🚀 Flask服务器正在启动（子线程）...")
    print("📍 访问地址: http://localhost:5000")
    print("📋 可用API接口:")
    print("   - GET/POST http://localhost:5000/api/output/create_tables")
    print("   - GET/POST http://localhost:5000/api/output/find_data")
    print("   - GET/POST http://localhost:5000/api/output/insert_data")
    print("   - GET/POST http://localhost:5000/api/output/update_data")
    print("   - GET/POST http://localhost:5000/api/output/delete_data")
    print("   - GET      http://localhost:5000/api/output/video_feed (摄像头视频流)")
    print("   - GET      http://localhost:5000/api/output/videos/<id>/<filename> (获取视频文件)")
    print("   - GET      http://localhost:5000/api/output/list_videos (获取视频列表)")
    print("=" * 50)

    # 启动Flask服务器
    # debug=False: 关闭调试模式（子线程中调试模式可能不会自动重载）
    # host='0.0.0.0': 监听所有网络接口，允许外部访问
    # port=5000: 监听5000端口
    # use_reloader=False: 禁用自动重载（子线程中不支持）
    myserver.run(debug=False, host='0.0.0.0', port=5000, use_reloader=False)

# ==================== 主程序入口 ====================

if __name__ == '__main__':
    """
    程序主入口
    功能：
    1. 启动定时任务调度器（每秒生成10个随机数）
    2. 在子线程中启动Flask服务器
    3. 保持主线程运行，等待用户中断（Ctrl+C）
    """
    # 启动调度器（在后台线程中运行，每秒生成10个随机数）
    print("📌 启动随机数生成任务（子线程）...")
    scheduler.start()  # 启动调度器，开始执行定时任务
    print("✅ 随机数生成任务已启动：每秒生成10个随机数")

    # 创建并启动Flask服务器线程
    print("📌 主线程：准备启动Flask服务器（子线程）...")
    flask_thread = threading.Thread(target=run_flask_server, daemon=True)
    # target: 线程要执行的函数
    # daemon=True: 设置为守护线程，主程序退出时自动终止
    flask_thread.start()  # 启动线程

    print("✅ Flask服务器已在子线程中启动")
    print("💡 主线程可以继续执行其他任务...")
    print("⏹️  按 Ctrl+C 停止服务器\n")

    # 保持主线程运行，否则子线程会被终止
    try:
        # 这里可以添加主线程的其他任务
        # 例如：其他业务逻辑、监控任务等
        while True:
            # 每1秒检查一次，保持主线程活跃
            # threading.Event().wait(1) 会暂停1秒，避免CPU占用过高
            threading.Event().wait(1)
    except KeyboardInterrupt:
        # 捕获Ctrl+C中断信号，优雅地关闭服务
        print("\n🛑 正在停止服务器...")
        scheduler.shutdown()  # 停止调度器，终止定时任务
        print("✅ 所有服务已停止")
