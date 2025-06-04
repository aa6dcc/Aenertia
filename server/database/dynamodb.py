# implement DynamoDB CRUD operations

from aws_config import dynamodb
import boto3
from boto3.dynamodb.conditions import Key
from decimal import Decimal
from .aws_config import AWS_REGION

dynamodb = boto3.resource('dynamodb', region_name=AWS_REGION)

def get_table(name):
    return dynamodb.Table(name)

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
    table = get_table('PIDValues')
    table.put_item(Item={
        'device_id': device_id,
        'loop': loop,
        'pg': Decimal(str(pg)),
        'dg': Decimal(str(dg)),
        'ig': Decimal(str(ig)),
        'sp': Decimal(str(sp)),
        'rot': Decimal(str(rot))
    })
