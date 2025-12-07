/**
 * API Configuration Utility
 * 
 * Determines the base URL for API requests based on environment.
 * 
 * Configuration Priority:
 * 1. Environment variable API_URL or REACT_APP_API_URL (set at build time)
 * 2. Development: Automatically detects localhost and uses http://localhost:8000
 * 3. Production: Uses relative path (if backend is on same domain)
 *                OR set API_URL environment variable for cross-domain backend
 * 
 * To set API_URL for production:
 * - Build time: Set environment variable before building
 *   Example: API_URL=https://your-backend.vercel.app npm run build
 * - Or add to docusaurus.config.ts customFields (requires config update)
 * 
 * Note: Docusaurus webpack replaces process.env variables at build time.
 * Variables must be prefixed with REACT_APP_ or set via webpack DefinePlugin.
 */
export function getApiBaseUrl(): string {
  // Check for environment variable (set at build time)
  // Docusaurus webpack replaces process.env variables at build time
  // For Docusaurus, you can also use customFields in docusaurus.config.ts
  const envApiUrl = 
    (typeof process !== 'undefined' && process.env?.API_URL) ||
    (typeof process !== 'undefined' && process.env?.REACT_APP_API_URL);

  if (envApiUrl && typeof envApiUrl === 'string' && envApiUrl.trim() !== '') {
    return envApiUrl.trim();
  }

  // Check if we're in development (localhost)
  if (typeof window !== 'undefined') {
    const isLocalhost = 
      window.location.hostname === 'localhost' ||
      window.location.hostname === '127.0.0.1' ||
      window.location.hostname === '' ||
      window.location.hostname.startsWith('127.');

    if (isLocalhost) {
      return 'http://localhost:8000';
    }
  }

  // Production: Use relative path (assumes backend is on same domain)
  // ⚠️ If your backend is on a DIFFERENT domain (e.g., Vercel, Railway),
  //    you MUST set the API_URL environment variable before building:
  //    API_URL=https://your-backend.vercel.app npm run build
  return '';
}

/**
 * Get the full API URL for an endpoint
 * @param endpoint - API endpoint path (e.g., '/api/chat/message')
 * @returns Full URL to the API endpoint
 */
export function getApiUrl(endpoint: string): string {
  const baseUrl = getApiBaseUrl();
  
  // Remove leading slash from endpoint if baseUrl is empty (relative path)
  const cleanEndpoint = endpoint.startsWith('/') ? endpoint : `/${endpoint}`;
  
  if (baseUrl) {
    // Ensure baseUrl doesn't end with slash and endpoint starts with /
    const cleanBaseUrl = baseUrl.endsWith('/') ? baseUrl.slice(0, -1) : baseUrl;
    return `${cleanBaseUrl}${cleanEndpoint}`;
  }
  
  // Relative path - use current origin
  return cleanEndpoint;
}

