import boto3
from datetime import datetime

dynamodb = boto3.resource('dynamodb', region_name='eu-north-1')
table = dynamodb.Table('RobotState')

table.put_item(Item={
    'device_id': 'robot001',
    'timestamp': datetime.utcnow().isoformat(),
    'x': 15.2,
    'y': 30.5,
    'battery': 87,
    'mode': 'autonomous',
    'status': 'connected',
    'current_key_loc': 'ChargingStation'
})
