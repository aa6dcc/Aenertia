import boto3
from datetime import datetime

dynamodb = boto3.resource('dynamodb', region_name='eu-north-1')
table = dynamodb.Table('LocationHistory')

table.put_item(Item={
    'device_id': 'robot001',
    'timestamp': datetime.utcnow().isoformat(),
    'x': 15.2,
    'y': 30.5
})
