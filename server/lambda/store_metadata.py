# Lambda function to store file metadata in DynamoDB after S3 upload.

import json

def lambda_handler(event, context):
    print("📦 Event received:", json.dumps(event, indent=2))

    # Check for S3 trigger
    if "Records" in event and event["Records"][0]["eventSource"] == "aws:s3":
        record = event["Records"][0]
        bucket = record["s3"]["bucket"]["name"]
        key = record["s3"]["object"]["key"]
        s3_path = f"s3://{bucket}/{key}"
        print(f"🚀 Triggered by upload: {s3_path}")
        return {
            "statusCode": 200,
            "body": json.dumps(f"Processed upload: {s3_path}")
        }

    # Handle test input
    try:
        battery_level = event["metadata"]["battery_level"]
        if battery_level < 10:
            return {
                "statusCode": 200,
                "body": json.dumps(f"⚠️ WARNING: Battery low ({battery_level}%)")
            }
        else:
            return {
                "statusCode": 200,
                "body": json.dumps("✅ Battery level OK")
            }
    except (KeyError, TypeError):
        return {
            "statusCode": 400,
            "body": json.dumps("Invalid input format.")
        }