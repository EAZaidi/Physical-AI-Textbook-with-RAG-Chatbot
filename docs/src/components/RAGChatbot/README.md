# RAG Chatbot Frontend - Troubleshooting Guide

This guide helps you troubleshoot common issues with the RAG Chatbot frontend integration.

## Table of Contents

- [Quick Diagnostics](#quick-diagnostics)
- [Common Issues](#common-issues)
- [Configuration Issues](#configuration-issues)
- [API/Backend Issues](#apibackend-issues)
- [Browser Compatibility](#browser-compatibility)
- [Performance Issues](#performance-issues)
- [Development Issues](#development-issues)

---

## Quick Diagnostics

### Is the chatbot visible?

**Symptom**: Chatbot toggle button doesn't appear

**Possible causes**:
1. JavaScript not enabled in browser
2. React error preventing render (check browser console)
3. CSS z-index conflict with site theme

**Solutions**:
```bash
# Check browser console for errors
# Open DevTools (F12) and look in Console tab

# Verify chatbot is mounted
# In Console, run:
document.querySelector('[role="region"][aria-label="AI chatbot"]')
# Should return an element, not null
```

### Can you click the toggle button?

**Symptom**: Button visible but not clickable

**Possible causes**:
1. CSS `pointer-events: none` on parent element
2. Another element overlaying the button
3. JavaScript error preventing event handlers

**Solutions**:
- Check browser console for errors
- Inspect element in DevTools to verify z-index and pointer-events
- Try clicking directly in Console:
  ```javascript
  document.querySelector('.chatToggle')?.click()
  ```

---

## Common Issues

### 1. CORS Errors

**Symptom**: Error in console: "Access to fetch at '...' from origin '...' has been blocked by CORS policy"

**Cause**: Backend not configured to allow requests from frontend origin

**Solution**:

Backend must include frontend origin in CORS configuration:

```python
# backend/agent_api/main.py
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Docusaurus dev server
        "https://yourdomain.com",  # Production domain
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

**Verification**:
```bash
# Test CORS with curl
curl -I -X OPTIONS http://localhost:8000/chat/run \
  -H "Origin: http://localhost:3000" \
  -H "Access-Control-Request-Method: POST"

# Should return:
# Access-Control-Allow-Origin: http://localhost:3000
```

---

### 2. Backend Not Responding

**Symptom**: Error message: "Unable to connect to the chatbot service"

**Cause**: Backend is not running or unreachable

**Solutions**:

1. **Verify backend is running**:
   ```bash
   # Check if backend is running
   curl http://localhost:8000/health
   # Should return: {"status": "healthy"}
   ```

2. **Check backend URL configuration**:
   ```bash
   # In docs/.env.local:
   RAG_BACKEND_URL=http://localhost:8000

   # Verify it's loaded:
   # In browser console:
   console.log(window.docusaurus.siteConfig.customFields.ragBackendUrl)
   ```

3. **Start backend**:
   ```bash
   cd backend
   # Using Docker:
   docker-compose up

   # Or directly:
   uvicorn agent_api.main:app --reload --port 8000
   ```

---

### 3. Session Not Persisting

**Symptom**: Conversation history lost when navigating between pages

**Cause**: sessionStorage not working or being cleared

**Solutions**:

1. **Check browser privacy settings**:
   - sessionStorage disabled in private/incognito mode
   - Third-party cookies blocked

2. **Verify sessionStorage is working**:
   ```javascript
   // In browser console:
   sessionStorage.setItem('test', 'value')
   sessionStorage.getItem('test') // Should return 'value'
   ```

3. **Check for storage quota issues**:
   ```javascript
   // In browser console:
   const stored = sessionStorage.getItem('rag-chatbot-session')
   console.log('Session size:', new Blob([stored]).size, 'bytes')
   // If > 5MB, storage quota may be exceeded
   ```

**Workaround**: Click "New Conversation" to clear old messages

---

### 4. Streaming Not Working

**Symptom**: Responses appear all at once instead of streaming word-by-word

**Possible causes**:
1. Streaming disabled in config
2. Backend doesn't support SSE streaming
3. Network/proxy blocking SSE

**Solutions**:

1. **Verify streaming is enabled**:
   ```javascript
   // In browser console:
   console.log(window.docusaurus.siteConfig.customFields.streamingEnabled)
   // Should be true or undefined (defaults to true)
   ```

2. **Check backend streaming endpoint**:
   ```bash
   curl -N http://localhost:8000/chat/stream \
     -H "Content-Type: application/json" \
     -d '{"message":"test","session_id":"test-123"}'
   # Should stream events line by line
   ```

3. **Check for proxy interference**:
   - Some proxies buffer SSE responses
   - Try direct connection without proxy
   - Check network tab in DevTools for "eventstream" content-type

4. **Fallback behavior**:
   - Chatbot automatically falls back to synchronous API if streaming fails
   - Check console for warning: "SSE connection error, falling back to synchronous"

---

### 5. Messages Not Sending

**Symptom**: Clicking Send does nothing or shows validation error

**Causes & Solutions**:

1. **Empty message**:
   - Minimum 1 character required
   - Whitespace-only messages are rejected

2. **Message too long**:
   - Maximum 1000 characters
   - Character counter shows when approaching limit

3. **Invalid characters**:
   - Usually not an issue, but check for null bytes

4. **JavaScript error**:
   - Check browser console
   - Look for React errors or API client errors

**Debug**:
```javascript
// In browser console, manually send a message:
const chatContext = document.querySelector('[role="region"][aria-label="AI chatbot"]')
// Then inspect the state or trigger sendMessage
```

---

## Configuration Issues

### Environment Variables Not Loading

**Symptom**: Backend URL is wrong or chatbot can't find backend

**Solution**:

1. **Check .env.local file** (docs/.env.local):
   ```bash
   RAG_BACKEND_URL=http://localhost:8000
   RAG_API_KEY=your-api-key-here
   ```

2. **Restart dev server** after changing .env files:
   ```bash
   # Stop dev server (Ctrl+C)
   npm run start
   ```

3. **Verify in browser console**:
   ```javascript
   console.log(window.docusaurus.siteConfig.customFields)
   ```

4. **Production builds**:
   - Environment variables are baked in at build time
   - Must rebuild after changing env vars:
     ```bash
     npm run build
     ```

---

### API Key Issues

**Symptom**: 401 Unauthorized or 403 Forbidden errors

**Solutions**:

1. **Verify API key is set**:
   ```bash
   # In .env.local:
   RAG_API_KEY=your-actual-api-key
   ```

2. **Check header is sent**:
   - Open Network tab in DevTools
   - Find request to /chat/run
   - Check Headers → Request Headers
   - Should include: `X-API-Key: your-api-key`

3. **Backend verification**:
   ```bash
   # Test with curl:
   curl http://localhost:8000/chat/run \
     -H "X-API-Key: your-api-key" \
     -H "Content-Type: application/json" \
     -d '{"message":"test","session_id":"test-123"}'
   ```

---

## API/Backend Issues

### Rate Limiting (429 Errors)

**Symptom**: "Too many questions. Please wait N seconds"

**Cause**: Backend rate limiter triggered

**Solutions**:

1. **Wait for retry period** (shown in error message)

2. **Check rate limit settings** in backend:
   ```python
   # backend/agent_api/main.py
   # Adjust rate limit if needed
   ```

3. **Clear rate limit** (development only):
   - Restart backend
   - Or implement rate limit reset endpoint

---

### Timeout Errors

**Symptom**: "The response is taking longer than expected"

**Cause**: Backend taking >30 seconds to respond

**Solutions**:

1. **Increase timeout** (if backend is legitimately slow):
   ```javascript
   // In Root.tsx, modify backendConfig:
   const backendConfig = {
     ...DEFAULT_BACKEND_CONFIG,
     timeoutMs: 60000, // 60 seconds
   }
   ```

2. **Optimize backend**:
   - Check backend logs for slow operations
   - Consider caching frequently asked questions
   - Optimize vector search

3. **Use streaming**:
   - Streaming provides faster perceived response time
   - First token appears in <1 second

---

## Browser Compatibility

### Supported Browsers

- ✅ Chrome/Edge 90+
- ✅ Firefox 88+
- ✅ Safari 14+
- ❌ Internet Explorer (not supported)

### Browser-Specific Issues

**Safari**:
- EventSource API may have issues with custom headers
- Check console for CORS-related errors
- Try disabling "Prevent Cross-Site Tracking"

**Firefox**:
- Ensure "Enhanced Tracking Protection" isn't blocking sessionStorage
- Check about:config for storage.enabled

**Mobile Browsers**:
- iOS Safari 14+ required
- Android Chrome 90+ required
- Test in responsive mode (DevTools) first

---

## Performance Issues

### Slow Loading

**Symptom**: Chatbot takes >1 second to appear

**Solutions**:

1. **Lazy loading is enabled** (already implemented)
   - Chatbot loads only when needed
   - Check Network tab for lazy chunk loading

2. **Reduce bundle size**:
   - Already optimized with React.memo
   - Consider code splitting if adding features

3. **Check backend latency**:
   ```bash
   curl -w "@-" -o /dev/null -s http://localhost:8000/health <<'EOF'
   time_namelookup:  %{time_namelookup}\n
   time_connect:  %{time_connect}\n
   time_total:  %{time_total}\n
   EOF
   ```

---

### Memory Leaks

**Symptom**: Browser tab becomes slow over time

**Solutions**:

1. **Clear conversation history**:
   - Click "New Conversation" button
   - Or close/reopen chat panel

2. **Check for console warnings**:
   - React warnings about memory leaks
   - setState on unmounted component

3. **Limit stored messages**:
   - Already implemented: only last 100 messages stored
   - Older messages automatically trimmed

---

## Development Issues

### Build Errors

**SSR (Server-Side Rendering) Errors**:

```
ReferenceError: sessionStorage is not defined
```

**Solution**: Already fixed with `typeof window !== 'undefined'` checks

---

### Hot Reload Not Working

**Symptom**: Changes to chatbot components don't reflect immediately

**Solutions**:

1. **Hard refresh**: Ctrl+Shift+R (or Cmd+Shift+R on Mac)

2. **Clear cache**:
   ```bash
   # Stop dev server
   rm -rf docs/.docusaurus docs/build
   npm run start
   ```

3. **Check for TypeScript errors**:
   ```bash
   npm run typecheck
   ```

---

### TypeScript Errors

**Common errors**:

1. **Type mismatch**: Check types.ts for interface definitions
2. **Missing imports**: Ensure all types are imported
3. **Module resolution**: Check tsconfig.json

**Solution**:
```bash
# Run type checker:
cd docs
npx tsc --noEmit
```

---

## Getting Help

If you're still experiencing issues:

1. **Check browser console** (F12 → Console tab)
2. **Check network tab** (F12 → Network tab)
3. **Check backend logs**
4. **Create issue** with:
   - Browser version
   - Console errors (screenshot)
   - Network requests (HAR export)
   - Steps to reproduce

---

## Debug Mode

Enable debug logging:

```javascript
// In browser console:
localStorage.setItem('ragChatbotDebug', 'true')
// Reload page

// Disable:
localStorage.removeItem('ragChatbotDebug')
```

This will log:
- API requests/responses
- State changes
- SSE events
- Validation errors

---

## Additional Resources

- **Spec**: `specs/004-rag-chatbot-frontend/spec.md`
- **API Docs**: `specs/004-rag-chatbot-frontend/contracts/backend-api.md`
- **Quickstart**: `specs/004-rag-chatbot-frontend/quickstart.md`
- **Backend Docs**: `backend/README.md`
