# voice_server.py

from flask import Flask, request, jsonify
from flask_cors import CORS
from dotenv import load_dotenv
from openai import OpenAI
import os

# 1. 从 .env 文件加载环境变量
load_dotenv()  # 这会把 .env 文件中的 OPENAI_API_KEY 等变量加载到 os.environ

# 2. 从环境变量读取 API Key
api_key = os.getenv("OPENAI_API_KEY")
if not api_key:
    raise RuntimeError("Error: OPENAI_API_KEY is not set in the environment. "
                       "Please create a .env file with OPENAI_API_KEY=sk-...")

# 3. 创建 OpenAI v1 客户端，并传入 api_key
client = OpenAI(api_key=api_key)

# 4. Flask 应用及 CORS 配置
app = Flask(__name__)
CORS(app)  # 允许前端（不同端口/主机）调用

@app.route("/interpret", methods=["POST"])
def interpret():
    user_input = request.json.get("command", "")
    print(f"Received voice command: {user_input}")

    prompt = (
        "You are a robot assistant. Convert the command into one of: "
        "'follow', 'return', 'stop', 'manual', 'autonomous'.\n"
        f"Command: {user_input}"
    )

    try:
        # 使用 OpenAI v1 接口
        response = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.0,
            max_tokens=4
        )
        # 提取 AI 回答
        result = response.choices[0].message.content.strip().lower()
        print(f"GPT interpreted as: {result}")
        return jsonify(result=result)
    except Exception as e:
        # 发生任何异常时返回 500 并在日志打印错误
        print(f"Error from OpenAI API: {e}")
        return jsonify(error=str(e)), 500

if __name__ == "__main__":
    # 监听 0.0.0.0:5001，以便同一局域网内其他设备也可以访问
    app.run(host="0.0.0.0", port=5001)
