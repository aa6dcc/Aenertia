# voice_server.py
from flask import Flask, request, jsonify
import openai
import os

app = Flask(__name__)

# Load your OpenAI key from environment variable
openai.api_key = os.getenv("OPENAI_API_KEY")

@app.route("/interpret", methods=["POST"])
def interpret():
    user_input = request.json.get("command", "")
    prompt = (
        "You are a robot assistant. Convert the command into one of: "
        "'follow', 'return', 'stop', 'manual', 'autonomous'.\n"
        f"Command: {user_input}"
    )
    try:
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}]
        )
        result = response.choices[0].message.content.strip().lower()
        return jsonify(result=result)
    except Exception as e:
        return jsonify(error=str(e)), 500

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)
