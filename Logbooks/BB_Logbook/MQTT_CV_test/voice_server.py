# voice_server.py

from flask import Flask, request, jsonify
from flask_cors import CORS       # ← import Flask-CORS
import openai
import os

app = Flask(__name__)
CORS(app)  # ← allow cross‐origin requests from your front end

# Load your OpenAI key from environment variable
openai.api_key = os.getenv("OPENAI_API_KEY")

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
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.0,
            max_tokens=4
        )
        result = response.choices[0].message.content.strip().lower()
        print(f"GPT interpreted as: {result}")
        return jsonify(result=result)
    except Exception as e:
        print(f"Error from OpenAI API: {e}")
        return jsonify(error=str(e)), 500

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)
