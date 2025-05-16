import json
from fastapi.testclient import TestClient
from app import app  # assuming your FastAPI code is in app.py

client = TestClient(app)

with open("example_prompt.json", "r") as f:
    data = json.load(f)

response = client.post("/generate_plan", json=data)
print(response.json())
