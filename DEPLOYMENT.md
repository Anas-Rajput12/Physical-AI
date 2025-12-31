# Deployment Instructions

## GitHub Pages (Frontend)

The Docusaurus frontend is automatically deployed to GitHub Pages using GitHub Actions when changes are pushed to the main branch.

## Backend Deployment

### Docker Deployment
To deploy the backend application using Docker:

1. Make sure you have Docker and Docker Compose installed
2. Create a `.env` file with the required environment variables:
   ```bash
   OPENROUTER_API_KEY=your_openrouter_api_key
   QDRANT_API_KEY=your_qdrant_api_key
   QDRANT_URL=your_qdrant_url
   NEON_DATABASE_URL=your_neon_database_url
   ```
3. Run the application:
   ```bash
   docker-compose up -d
   ```

### Manual Deployment
1. Navigate to the backend directory:
   ```bash
   cd backend
   ```
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Set environment variables
4. Start the server:
   ```bash
   python -m uvicorn src.api.main:app --host 0.0.0.0 --port 8000
   ```