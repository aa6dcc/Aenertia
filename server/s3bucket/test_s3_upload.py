import os
from server.s3bucket.s3_upload import upload_file_to_s3

# Use cross-platform temp directory
tmp_dir = os.path.join(os.getcwd(), "tmp")
os.makedirs(tmp_dir, exist_ok=True)

test_file_path = os.path.join(tmp_dir, "s3_test_file.txt")

# Create a dummy file
with open(test_file_path, "w") as f:
    f.write("This is a test upload to S3.")

# Upload
upload_file_to_s3(test_file_path, bucket_name="aener-shark-uploads", s3_key="test/s3_test_file.txt")
