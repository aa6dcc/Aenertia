# server/database/utils.py

from datetime import datetime
from decimal import Decimal

def iso_timestamp():
    """Returns UTC ISO8601 timestamp string"""
    return datetime.utcnow().isoformat()

def to_decimal(value):
    """Safely convert numeric value to Decimal"""
    return Decimal(str(value))

def velocity_between(x1, y1, x2, y2, dt):
    """Computes velocity given position change and time delta"""
    if dt <= 0:
        return 0.0
    dx = x2 - x1
    dy = y2 - y1
    return (dx**2 + dy**2)**0.5 / dt
