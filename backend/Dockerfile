# Backend Deployment Configuration

## Dockerfile for Backend
FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY backend/src/ ./src/
COPY start_backend.py ./
COPY backend/.env ./

# Expose port
EXPOSE 8000

# Run the application
CMD ["python", "start_backend.py"]