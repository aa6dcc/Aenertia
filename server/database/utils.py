# used to store generic helper functions that don’t belong to a specific module but are reused in many places

from datetime import datetime
from decimal import Decimal

def get_timestamp():
    return datetime.utcnow().isoformat()

def to_decimal(value):
    return Decimal(str(value))
