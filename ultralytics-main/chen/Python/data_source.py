# # data_source.py
# from datetime import datetime
#

#
# # ✅ Flask 定时任务将通过这个函数获取当前最新数据
# def get_current_data():
#     """返回当前真实数据（供 Flask 插入数据库使用）"""
#     return {
#         'location_x': location_x,
#         'location_y': location_y,
#         'location_z': location_z,
#         'location_rz': location_rz,
#         'speed': speed,
#         'count':count,
#         'confidence': confidence,
#         'time': datetime.now()
#     }

# data_source.py
from datetime import datetime
from beifen import handler



def get_current_data():
    """返回当前检测结果"""
    data = getattr(handler, 'result_data', None)
    if not data:
        return None

    return {
        'location_x': data.get('location_x', 0.0),
        'location_y': data.get('location_y', 0.0),
        'location_z': data.get('location_z', 0.0),
        'location_rz': data.get('angle', 0.0),
        'speed': data.get('speed', 0.0),
        'count': data.get('object_id', 0),
        'confidence': data.get('confidence', 0.0),
        'time': datetime.now()
    }







