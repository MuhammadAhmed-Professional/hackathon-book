/**
 * Frontend configuration for API base URL.
 * Defaults to localhost for development.
 * 
 * Note: In browser environment, process.env is not available.
 * For production, you can set window.API_BASE_URL before this module loads,
 * or configure it via Docusaurus's customFields in docusaurus.config.js
 */
const getApiBaseUrl = () => {
  // Check if running in browser and window variable is set (highest priority)
  if (typeof window !== 'undefined' && window.API_BASE_URL) {
    return window.API_BASE_URL;
  }
  
  // Check if we're in development (localhost) or production
  const isDevelopment = typeof window !== 'undefined' && 
    (window.location.hostname === 'localhost' || 
     window.location.hostname === '127.0.0.1' ||
     window.location.hostname.includes('localhost'));
  
  // Use Railway URL for production, localhost for development
  if (isDevelopment) {
    return 'http://localhost:8000';
  }
  
  // Replace with your actual Railway backend URL
  return 'https://agentic-book-production.up.railway.app';
};

const config = {
  API_BASE_URL: getApiBaseUrl(),
};

export default config;

