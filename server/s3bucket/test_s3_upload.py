import sys
import os

# Add project root to sys.path so imports work
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from server.s3bucket.s3_upload import upload_file_to_s3

# Upload the file
file_to_upload = os.path.join(os.path.dirname(__file__), "my_test.txt")
s3_key = "uploads/my_test.txt"  # You can change the key if needed

print("📤 Uploading to S3...")
result = upload_file_to_s3(file_to_upload, s3_key=s3_key)

if result:
    print(f"✅ Upload complete! File is at: {result}")
else:
    print("❌ Upload failed.")
