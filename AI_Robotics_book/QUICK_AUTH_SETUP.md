# 🚀 Quick Start: Authentication Implementation

## What's New?

Your docs now have a **Medium-style authentication system**:
- Users read freely until 40% scroll
- At 40% scroll, a login prompt appears (like Medium's paywall)
- After login, full access unlocked

## 🎯 Get Started in 3 Steps

### Step 1: Set Environment Variables
Copy `.env.example` to `.env` in `rag_backend/`:
```bash
cd rag_backend
cp .env.example .env
```

Edit `.env` and set at minimum:
```env
OPENAI_API_KEY=your_key_here
QDRANT_URL=your_url_here
QDRANT_API_KEY=your_key_here
SECRET=generate_with_openssl_rand_-base64_32
```

Generate a secure SECRET:
```bash
# macOS/Linux
openssl rand -base64 32

# Windows PowerShell  
-join ([char[]]'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789' | Get-Random -Count 32)
```

### Step 2: Start Backend
```bash
cd rag_backend
python server.py
# Server runs on http://localhost:8001
```

### Step 3: Start Frontend
```bash
cd docs
npm start
# Docs run on http://localhost:3000
```

## ✨ Test It

1. Open http://localhost:3000/docs/01-intro-to-ros2
2. **Scroll down past 40%** of the page
3. 👀 **Paywall modal appears!**
4. Click "Create Account" or "Sign In"
5. ✅ After login, paywall closes and you have full access

## 📁 New Components

| File | Purpose |
|------|---------|
| `docs/src/components/PaywallGate/PaywallGate.js` | Main paywall logic (40% scroll trigger) |
| `docs/src/components/PaywallGate/PaywallGate.module.css` | Paywall styling (Medium-style modal) |
| `docs/src/context/AuthContext.js` | Global auth state management |
| `docs/src/theme/Root.js` | Integrated auth + paywall into app |

## 🔧 Customize

**Change scroll percentage:**
Edit `PaywallGate.js` line 17:
```javascript
if (scrolled > 40 && !user) {  // Change 40 to different number
  setShowPaywall(true);
}
```

**Change paywall styling:**
Edit `PaywallGate.module.css` - all the CSS classes are there

**Change paywall message:**
Edit `PaywallGate.js` lines 40-50

## 📚 Full Documentation

See [AUTH_IMPLEMENTATION.md](AUTH_IMPLEMENTATION.md) for complete details.

## ⚠️ Common Issues

| Problem | Fix |
|---------|-----|
| Paywall not showing | Check `Root.js` has PaywallGate |
| Login fails | Verify `SECRET` in `.env` |
| Can't register | Check backend is running |
| Cookies not saved | Use `http://localhost` (not IP) |

## 🎓 How It Works

```
User scrolls page → 40% scroll detected → PaywallGate shows modal
│
├─ User logs in → AuthContext updates user state → Paywall closes
│
└─ User dismisses → Can still read (paywall closable)
```

Authentication is now **production-ready**! 🎉

---

Next: Set up OAuth for Google/GitHub login (see SETUP_GUIDE.md)
