from flask_sqlalchemy import SQLAlchemy
from datetime import datetime
import random

db = SQLAlchemy()

class Object(db.Model):
    __tablename__ = 't1'
    id = db.Column(db.Integer, primary_key=True)
    location_x = db.Column(db.Float, unique=False, nullable=False)
    location_y = db.Column(db.Float, unique=False, nullable=False)
    location_z = db.Column(db.Float, unique=False, nullable=False)
    location_rz = db.Column(db.Float, unique=False, nullable=False)
    speed = db.Column(db.Float, unique=False, nullable=False)
    count = db.Column(db.Float, unique=False, nullable=False)
    confidence = db.Column(db.Float, unique=False, nullable=False)
    time = db.Column(db.DateTime, default=datetime.now)
    x_counter = 0
    # mood = db.relationship("Mood", backref='databa', lazy='dynamic')
    def __repr__(self):
        return f'<Object {self.id!r} {self.location_x!s} {self.location_y!s} {self.location_z!s} {self.location_rz!s} {self.speed!s} {self.time!s}>'




