# RAG-Powered Search Guide

## Overview

Your documentation site now features a FictionLab-inspired dark theme with an integrated RAG (Retrieval Augmented Generation) search functionality.

## Features

### 🎨 Dark Theme
- Professional dark mode with orange/gold accents
- Smooth transitions and hover effects
- Improved readability with carefully chosen colors

### 🔍 Smart Search Bar
- **Location**: Top-right corner of the navbar
- **Keyboard Shortcut**: Press `Ctrl+K` (or `Cmd+K` on Mac) to focus the search
- **RAG Integration**: Queries are sent to your RAG backend for intelligent responses

### 🔐 Medium-Style Paywall
- **Free Reading**: Access first ~40% of content freely
- **Scroll Trigger**: Login prompt appears after scrolling 40% through page
- **Benefits**: Unlock full access, save progress, join community
- **Easy Auth**: Sign in with email or OAuth (Google/GitHub)

### ⌨️ Keyboard Shortcuts
- `Ctrl+K` / `Cmd+K`: Focus search bar
- `Escape`: Close search dropdown
- `Enter`: Submit search query

## How It Works

1. **Type your question** in the search bar
2. The query is sent to your RAG backend at `http://localhost:8001/query`
3. The RAG system:
   - Searches through your documentation
   - Uses AI to generate a relevant answer
   - Returns sources from the documentation
4. Results appear in a dropdown (navbar) or expanded view (search page)

## Running the RAG Backend

To enable the search functionality, you need to start the RAG backend:

### Option 1: Using NPM Scripts
```bash
# Install dependencies
npm run rag:setup

# Ingest documentation
npm run rag:ingest

# Start the server
npm run rag:start
```

### Option 2: Manual Setup
```bash
cd ../rag_backend
python -m venv .venv
.venv\Scripts\activate  # On Windows
# source .venv/bin/activate  # On Unix/Mac
pip install -r requirements.txt
python ingest.py
python server.py
```

The RAG backend will run on `http://localhost:8001`

## Configuration

The RAG API URL can be configured in `docusaurus.config.js`:

```javascript
customFields: {
  ragApiUrl: process.env.RAG_API_URL || 'http://localhost:8001',
}
```

You can set the `RAG_API_URL` environment variable to use a different backend URL.

## Styling Customization

### Colors
Edit [src/css/custom.css](src/css/custom.css) to customize:
- Primary color: `--ifm-color-primary` (currently orange)
- Background colors
- Text colors
- Border colors

### Search Bar
Edit [src/components/SearchBar.module.css](src/components/SearchBar.module.css) to customize:
- Search input styling
- Dropdown appearance
- Keyboard shortcut badge
- Results display

### Paywall
Edit [src/components/PaywallGate/PaywallGate.module.css](src/components/PaywallGate/PaywallGate.module.css) to customize:
- Modal appearance
- Button styles
- Overlay color and animation
- Scroll trigger percentage (modify `PaywallGate.js` line ~17)

Change scroll percentage to show paywall earlier or later:
```javascript
// Show paywall after scrolling 40% down (edit this value)
if (scrolled > 40 && !user) {
  setShowPaywall(true);
}
```

## Authentication

### Login System

Users can create accounts and authenticate in multiple ways:

1. **Email/Password**: Standard registration and login
2. **Google OAuth**: Click "Sign In" → Google account
3. **GitHub OAuth**: Click "Sign In" → GitHub account

### Accessing Protected Resources

Once authenticated:
- Full access to all chapters
- RAG-powered search without restrictions
- Saved reading progress
- Community features

### Environment Setup for Auth

The backend requires authentication credentials in `.env`:

```env
# Generate a secure random SECRET
SECRET=your_32_character_random_string

# OAuth (optional)
GOOGLE_OAUTH_CLIENT_ID=xxx
GOOGLE_OAUTH_CLIENT_SECRET=xxx
GITHUB_OAUTH_CLIENT_ID=xxx
GITHUB_OAUTH_CLIENT_SECRET=xxx
```

See [SETUP_GUIDE.md](../rag_backend/SETUP_GUIDE.md#authentication-system) for detailed OAuth setup.

## Troubleshooting

### Search returns errors
- Ensure RAG backend is running on port 8001
- Check browser console for error messages
- Verify CORS is enabled in the backend

### Search bar not visible
- Clear browser cache
- Rebuild the site: `npm run build`

### Paywall not showing
- Verify AuthProvider is loaded in `src/theme/Root.js`
- Check browser console for errors
- Ensure `PaywallGate` component is active

### Login issues
- Clear cookies: `Application` → `Cookies` → delete all for localhost
- Verify backend server is running
- Check that `SECRET` env var is set
- Check that the Navbar wrapper is properly configured

### Keyboard shortcut not working
- Make sure no other extension is using `Ctrl+K`
- Try clicking the search input directly

## Development

To modify the search behavior:
1. Edit [src/components/SearchBar.js](src/components/SearchBar.js)
2. Update the RAG backend at [../rag_backend/server.py](../rag_backend/server.py)
3. Restart both the Docusaurus dev server and RAG backend

## Production Deployment

When deploying to production:
1. Update `ragApiUrl` in `docusaurus.config.js` to your production RAG backend URL
2. Ensure the RAG backend is accessible from your deployment domain
3. Configure proper CORS settings
4. Use environment variables for configuration

## Support

For issues or questions:
- Check the [rag_backend/README.md](../rag_backend/README.md)
- Review [rag_backend/SETUP_GUIDE.md](../rag_backend/SETUP_GUIDE.md)
- Ensure all dependencies are installed
