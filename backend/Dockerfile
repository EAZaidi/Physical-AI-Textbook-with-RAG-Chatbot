FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy only application code
COPY agent_api ./agent_api

# Expose port 7860 (Hugging Face requirement)
EXPOSE 7860

# Run FastAPI with uvicorn on port 7860
CMD ["uvicorn", "agent_api.main:app", "--host", "0.0.0.0", "--port", "7860"]
