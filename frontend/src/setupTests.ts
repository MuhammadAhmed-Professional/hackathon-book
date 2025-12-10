/**
 * Test setup configuration for Jest and React Testing Library.
 */
import '@testing-library/jest-dom';

// Mock localStorage with proper Jest mock functions that support mockReturnValue
const localStorageMock = {
  getItem: jest.fn(),
  setItem: jest.fn(),
  removeItem: jest.fn(),
  clear: jest.fn(),
};

// Make sure the mocks are accessible on global.localStorage
Object.defineProperty(global, 'localStorage', {
  value: localStorageMock,
  writable: true,
  configurable: true,
});

// Mock crypto.randomUUID
Object.defineProperty(global, 'crypto', {
  value: {
    randomUUID: () => 'test-uuid-123',
  },
  configurable: true,
});

// Mock fetch globally
(global as any).fetch = jest.fn();

// Mock scrollIntoView for DOM elements (used in Chatbot component)
Element.prototype.scrollIntoView = jest.fn();

// Reset mocks before each test
beforeEach(() => {
  jest.clearAllMocks();
  localStorageMock.clear();
  (global.fetch as jest.Mock).mockClear();
  (Element.prototype.scrollIntoView as jest.Mock).mockClear();
});
