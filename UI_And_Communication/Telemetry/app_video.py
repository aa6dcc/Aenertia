# app_video.py

from flask import Flask, Response
from camera_stream import gen_frames

app = Flask(__name__)

@app.route('/video_feed')
def video_feed():
    return Response(
        gen_frames(),
        mimetype='multipart/x-mixed-replace; boundary=frame'
    )

if __name__ == '__main__':
    # Listen on all interfaces so other devices can reach it
    app.run(host='0.0.0.0', port=8001, threaded=True)
