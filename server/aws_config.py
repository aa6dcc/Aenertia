# AWS session, clients, env

import boto3

REGION = 'eu-north-1'

session = boto3.Session(region_name=REGION)

dynamodb = session.resource('dynamodb')
