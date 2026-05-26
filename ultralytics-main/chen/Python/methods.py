from mysql import db, Object
from datetime import datetime

def insert_real_data():
    """从 data_source.py 获取变量并插入数据库"""
    try:
        # ✅ 延迟导入 Flask 实例
        from app import myserver
        from data_source import get_current_data
        # from get_ssssss import get_current_data

        with myserver.app_context():
            data = get_current_data()
            if data:
                new_obj = Object(
                    location_x=data['location_x'],
                    location_y=data['location_y'],
                    location_z=data['location_z'],
                    location_rz=data['location_rz'],
                    speed=data['speed'],
                    count=data['count'],
                    confidence=data['confidence'],
                    time=data['time']
                )
                db.session.add(new_obj)
                db.session.commit()
                print(f"✅ 成功插入真实数据: x={data['location_x']:.2f}, y={data['location_y']:.2f},z={data['location_z']:.2f,rz={data['location_rz']:.2f}},v={data['speed']:.2f},ci={data['confidence']:.2f}")
    except Exception as e:
        import traceback
        print(f"❌ 插入真实数据失败: {e}")
        print(traceback.format_exc())
