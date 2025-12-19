#!/bin/bash
# Startup script for Hugging Face Spaces
# Runs both FastAPI backend and serves static frontend

set -e

echo "=================================================="
echo "Starting AI-Native Textbook on Hugging Face"
echo "=================================================="

# Start FastAPI backend on port 8000
echo "Starting FastAPI backend..."
uvicorn agent_api.main:app --host 0.0.0.0 --port 8000 &
BACKEND_PID=$!

# Wait for backend to be ready
echo "Waiting for backend to be ready..."
sleep 5

# Start simple HTTP server for frontend on port 7860 (HF Spaces standard port)
echo "Starting frontend server on port 7860..."
cd /app/frontend
python -m http.server 7860 &
FRONTEND_PID=$!

echo "=================================================="
echo "Services started successfully!"
echo "Backend API: http://localhost:8000"
echo "Frontend: http://localhost:7860"
echo "=================================================="

# Wait for both processes
wait $BACKEND_PID $FRONTEND_PID
