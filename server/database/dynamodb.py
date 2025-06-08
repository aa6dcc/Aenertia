# implement DynamoDB CRUD operations

from aws_config import dynamodb
import boto3
from boto3.dynamodb.conditions import Key
from decimal import Decimal
from .aws_config import AWS_REGION
import botocore.exceptions


dynamodb = boto3.resource('dynamodb', region_name=AWS_REGION)

_table_cache = {}

def get_table(name):
    if name not in _table_cache:
        _table_cache[name] = dynamodb.Table(name)
    return _table_cache[name]


def update_robot_state(device_id, x, y, battery, mode, status, current_key_loc):
    table = get_table('RobotState')
    from .utils import iso_timestamp
    table.put_item(Item={
        'device_id': device_id,
        'timestamp': iso_timestamp(),
        'x': Decimal(str(x)),
        'y': Decimal(str(y)),
        'battery': Decimal(str(battery)),
        'mode': mode,
        'status': status,
        'current_key_loc': current_key_loc
    })

def log_location(device_id, x, y):
    table = get_table('LocationHistory')
    from .utils import iso_timestamp
    table.put_item(Item={
        'device_id': device_id,
        'timestamp': iso_timestamp(),
        'x': Decimal(str(x)),
        'y': Decimal(str(y))
    })

def save_key_location(name, x, y):
    table = get_table('KeyLocations')
    table.put_item(Item={
        'name': name,
        'x': Decimal(str(x)),
        'y': Decimal(str(y))
    })

def update_pid_values(device_id, loop, pg, dg, ig, sp, rot=0):
    table = get_table('PIDConfigs')
    table.put_item(Item={
        'device_id': device_id,
        'loop': loop,
        'pg': Decimal(str(pg)),
        'dg': Decimal(str(dg)),
        'ig': Decimal(str(ig)),
        'sp': Decimal(str(sp)),
        'rot': Decimal(str(rot))
    })

def get_all_key_locations():
    table = get_table('KeyLocations')
    response = table.scan()
    return response['Items']

def find_nearest_key_location(x, y):
    from math import sqrt
    key_locations = get_all_key_locations()
    closest = None
    min_dist = float('inf')
    for loc in key_locations:
        dx = float(loc['x']) - x
        dy = float(loc['y']) - y
        dist = sqrt(dx**2 + dy**2)
        if dist < min_dist:
            min_dist = dist
            closest = loc['name']
    return closest or "Unknown"

def get_battery_level():
    try:
        with open('/sys/class/power_supply/BAT0/capacity', 'r') as f:
            return float(f.read().strip())
    except:
        return 100.0  # default fallback

def get_mode():
    return "AUTONOMOUS"  # TODO: hook to actual mode topic or flag

def get_status(velocity):
    return "MOVING" if velocity > 0.1 else "IDLE"


def safe_put_item(table, item):
    try:
        table.put_item(Item=item)
    except botocore.exceptions.BotoCoreError as e:
        print(f"Failed to write to {table.name}: {e}")