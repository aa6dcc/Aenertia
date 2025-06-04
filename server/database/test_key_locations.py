import boto3

dynamodb = boto3.resource('dynamodb', region_name='eu-north-1')
table = dynamodb.Table('KeyLocations')

table.put_item(Item={
    'name': 'ChargingStation',
    'x': 15.2,
    'y': 30.5
})
