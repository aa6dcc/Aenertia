# Handles SLAM and generic file uploads to AWS S3

import os
from datetime import datetime
import boto3
from botocore.exceptions import ClientError
from server.aws_config import session

# Set up S3 client
s3 = session.client("s3")
BUCKET_NAME = "aener-shark-uploads"  

# ----------- SLAM MAP SUPPORT -----------

def timestamped_key(filename, prefix="slam/maps/"):
    """
    Create an S3 key like slam/maps/2025-06-04T15-20-10-map.png
    """
    now = datetime.utcnow().strftime("%Y-%m-%dT%H-%M-%S")
    base = os.path.basename(filename)
    return f"{prefix}{now}-{base}"

def upload_slam_map(map_path):
    """
    Upload a SLAM map file to S3 with a timestamped key.
    """
    key = timestamped_key(map_path)
    try:
        s3.upload_file(map_path, BUCKET_NAME, key)
        print(f"✅ Uploaded map to s3://{BUCKET_NAME}/{key}")
        return f"s3://{BUCKET_NAME}/{key}"
    except ClientError as e:
        print(f"❌ SLAM upload failed: {e}")
        return None

# ----------- GENERIC FILE UPLOAD -----------

def upload_file_to_s3(local_path, bucket_name=BUCKET_NAME, s3_key=None):
    """
    Upload any file to S3 at a specific key.
    """
    if not os.path.isfile(local_path):
        print(f"❌ File does not exist: {local_path}")
        return None

    if not s3_key:
        s3_key = os.path.basename(local_path)

    try:
        s3.upload_file(local_path, bucket_name, s3_key)
        print(f"✅ Uploaded {local_path} to s3://{bucket_name}/{s3_key}")
        return f"s3://{bucket_name}/{s3_key}"
    except ClientError as e:
        print(f"❌ Upload failed: {e}")
        return None

# ----------- EXAMPLE USAGE -----------

if __name__ == "__main__":
    # Test SLAM map upload
    slam_test_path = "/home/pi/maps/latest_map.png"
    upload_slam_map(slam_test_path)

    # Test generic file upload
    log_test_path = "/tmp/test_log.txt"
    with open(log_test_path, "w") as f:
        f.write("Log upload test")
    upload_file_to_s3(log_test_path, s3_key="logs/test_log.txt")
