# Makes the s3bucket module importable as a package.

from .s3_upload import upload_file_to_s3

__all__ = ["upload_file_to_s3"]
