# Dockerfile for Drone_Onboard_AV
FROM python:3.10-slim

WORKDIR /app

# Install dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy code
COPY . .

# Runtime environment
ENV PYTHONUNBUFFERED=1

CMD ["python", "main.py"]
