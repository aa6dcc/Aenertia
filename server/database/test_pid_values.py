import boto3

dynamodb = boto3.resource('dynamodb', region_name='eu-north-1')
table = dynamodb.Table('PIDValues')

# Inner loop
table.put_item(Item={
    'device_id': 'robot001',
    'loop': 'inner',
    'pg': 1.2,
    'dg': 0.01,
    'ig': 0.05,
    'sp': 20.0,
    'rot': 0.0
})

# Outer loop
table.put_item(Item={
    'device_id': 'robot001',
    'loop': 'outer',
    'pg': 2.5,
    'dg': 0.02,
    'ig': 0.1,
    'sp': 50.0,
    'rot': 90.0
})
