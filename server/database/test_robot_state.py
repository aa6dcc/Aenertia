import boto3
from datetime import datetime
from decimal import Decimal

dynamodb = boto3.resource('dynamodb', region_name='eu-north-1')
table = dynamodb.Table('RobotState')

table.put_item(Item={
    'device_id': 'robot001',
    'timestamp': datetime.utcnow().isoformat(),
    'x': Decimal('15.2'),
    'y': Decimal('30.5'),
    'battery': Decimal('87'),
    'mode': 'autonomous',
    'status': 'connected',
    'current_key_loc': 'ChargingStation'
})
