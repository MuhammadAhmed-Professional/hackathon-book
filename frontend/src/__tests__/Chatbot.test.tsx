/**
 * Tests for Chatbot component.
 *
 * Tests core chatbot functionality including message sending, response handling,
 * skill execution, and error scenarios.
 */
import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import Chatbot from '../components/Chatbot';

// Mock config - must be before any imports that use it
jest.mock('../config', () => {
  return {
    __esModule: true,
    default: {
      API_BASE_URL: 'http://localhost:8000',
    },
  };
});

describe('Chatbot Component', () => {
  beforeEach(() => {
    // Reset localStorage mocks
    (localStorage.getItem as jest.Mock).mockReturnValue(null);
    (localStorage.setItem as jest.Mock).mockClear();
  });

  describe('Rendering', () => {
    test('renders chatbot interface', () => {
      // Arrange & Act
      render(<Chatbot />);

      // Assert
      expect(screen.getByPlaceholderText(/ask a question/i)).toBeInTheDocument();
      expect(screen.getByRole('button', { name: /send/i })).toBeInTheDocument();
    });

    test('renders with welcome message for empty chat', () => {
      // Arrange & Act
      render(<Chatbot />);

      // Assert
      expect(screen.getByText(/ask me anything about/i)).toBeInTheDocument();
    });

    test('renders skill buttons in interface', () => {
      // Arrange & Act
      render(<Chatbot />);

      // Assert
      expect(screen.getByText(/explain term/i)).toBeInTheDocument();
      expect(screen.getByText(/summarize/i)).toBeInTheDocument();
      expect(screen.getByText(/generate quiz/i)).toBeInTheDocument();
    });
  });

  describe('Message Sending', () => {
    test('sends message when user clicks Send button', async () => {
      // Arrange
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          answer_text: 'ROS 2 is a robotics middleware',
          sources: [],
          confidence: 0.95,
        }),
      });

      render(<Chatbot />);
      const input = screen.getByPlaceholderText(/ask a question/i) as HTMLInputElement;
      const sendButton = screen.getByRole('button', { name: /send/i });

      // Act
      await userEvent.type(input, 'What is ROS 2?');
      fireEvent.click(sendButton);

      // Assert
      await waitFor(() => {
        expect(fetch).toHaveBeenCalledWith(
          'http://localhost:8000/ask',
          expect.objectContaining({
            method: 'POST',
            headers: expect.objectContaining({
              'Content-Type': 'application/json',
            }),
            body: expect.stringContaining('What is ROS 2?'),
          })
        );
      });
    });

    test('sends message when user presses Enter key', async () => {
      // Arrange
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          answer_text: 'Test answer',
          sources: [],
        }),
      });

      render(<Chatbot />);
      const input = screen.getByPlaceholderText(/ask a question/i);

      // Act
      await userEvent.type(input, 'Test question{Enter}');

      // Assert
      await waitFor(() => {
        expect(fetch).toHaveBeenCalled();
      });
    });

    test('clears input field after sending message', async () => {
      // Arrange
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          answer_text: 'Answer',
          sources: [],
        }),
      });

      render(<Chatbot />);
      const input = screen.getByPlaceholderText(/ask a question/i) as HTMLInputElement;

      // Act
      await userEvent.type(input, 'Question');
      fireEvent.click(screen.getByRole('button', { name: /send/i }));

      // Assert
      await waitFor(() => {
        expect(input.value).toBe('');
      });
    });

    test('does not send empty message', async () => {
      // Arrange
      render(<Chatbot />);
      const sendButton = screen.getByRole('button', { name: /send/i });

      // Act
      fireEvent.click(sendButton);

      // Assert
      expect(fetch).not.toHaveBeenCalled();
    });

    test('does not send whitespace-only message', async () => {
      // Arrange
      render(<Chatbot />);
      const input = screen.getByPlaceholderText(/ask a question/i);
      const sendButton = screen.getByRole('button', { name: /send/i });

      // Act
      await userEvent.type(input, '   ');
      fireEvent.click(sendButton);

      // Assert
      expect(fetch).not.toHaveBeenCalled();
    });
  });

  describe('Response Handling', () => {
    test('displays bot response after receiving answer', async () => {
      // Arrange
      const mockAnswer = 'ROS 2 is the Robot Operating System version 2';
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          answer_text: mockAnswer,
          sources: [],
        }),
      });

      render(<Chatbot />);
      const input = screen.getByPlaceholderText(/ask a question/i);

      // Act
      await userEvent.type(input, 'What is ROS 2?{Enter}');

      // Assert
      await waitFor(() => {
        expect(screen.getByText(mockAnswer)).toBeInTheDocument();
      });
    });

    test('displays sources when included in response', async () => {
      // Arrange
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          answer_text: 'Test answer',
          sources: [
            {
              chapter_id: 'module1/ros2-introduction',
              chunk_id: 'chunk_001',
              relevance_score: 0.95,
            },
          ],
        }),
      });

      render(<Chatbot />);

      // Act
      await userEvent.type(screen.getByPlaceholderText(/ask a question/i), 'Test{Enter}');

      // Assert
      await waitFor(() => {
        expect(screen.getByText(/source/i)).toBeInTheDocument();
      });
    });

    test('displays user message in chat', async () => {
      // Arrange
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({ answer_text: 'Answer', sources: [] }),
      });

      render(<Chatbot />);
      const userQuestion = 'What is a ROS 2 topic?';

      // Act
      await userEvent.type(screen.getByPlaceholderText(/ask a question/i), `${userQuestion}{Enter}`);

      // Assert
      expect(screen.getByText(userQuestion)).toBeInTheDocument();
    });

    test('saves conversation ID when returned', async () => {
      // Arrange
      const mockConversationId = 123;
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          answer_text: 'Answer',
          sources: [],
          conversation_id: mockConversationId,
        }),
      });

      // Mock console.log to verify logging
      const consoleSpy = jest.spyOn(console, 'log').mockImplementation();

      render(<Chatbot />);

      // Act
      await userEvent.type(screen.getByPlaceholderText(/ask a question/i), 'Question{Enter}');

      // Assert
      await waitFor(() => {
        expect(consoleSpy).toHaveBeenCalledWith(
          expect.stringContaining(`Conversation saved: ID=${mockConversationId}`)
        );
      });

      consoleSpy.mockRestore();
    });
  });

  describe('Loading State', () => {
    test('shows loading state while waiting for response', async () => {
      // Arrange
      (fetch as jest.Mock).mockImplementation(
        () => new Promise((resolve) => setTimeout(resolve, 100))
      );

      render(<Chatbot />);
      const input = screen.getByPlaceholderText(/ask a question/i);

      // Act
      await userEvent.type(input, 'Test{Enter}');

      // Assert
      // Loading indicator should appear (loading dots)
      await waitFor(() => {
        const loadingDots = document.querySelector('.loading-dots');
        expect(loadingDots).toBeInTheDocument();
      });
    });

    test('disables send button while loading', async () => {
      // Arrange
      (fetch as jest.Mock).mockImplementation(
        () => new Promise((resolve) => setTimeout(() => resolve({
          ok: true,
          json: async () => ({ answer_text: 'Answer', sources: [] }),
        }), 100))
      );

      render(<Chatbot />);
      const input = screen.getByPlaceholderText(/ask a question/i);
      const sendButton = screen.getByRole('button', { name: /send/i });

      // Act
      await userEvent.type(input, 'Test');
      fireEvent.click(sendButton);

      // Assert - send button should be disabled during loading
      expect(sendButton).toBeDisabled();
    });
  });

  describe('Error Handling', () => {
    test('displays error message on API failure', async () => {
      // Arrange
      (fetch as jest.Mock).mockRejectedValueOnce(new Error('Network error'));

      render(<Chatbot />);

      // Act
      await userEvent.type(screen.getByPlaceholderText(/ask a question/i), 'Test{Enter}');

      // Assert
      await waitFor(() => {
        expect(screen.getByText(/error/i)).toBeInTheDocument();
        expect(screen.getByText(/try again/i)).toBeInTheDocument();
      });
    });

    test('displays error message when response is not ok', async () => {
      // Arrange
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: false,
        status: 500,
      });

      render(<Chatbot />);

      // Act
      await userEvent.type(screen.getByPlaceholderText(/ask a question/i), 'Test{Enter}');

      // Assert
      await waitFor(() => {
        expect(screen.getByText(/error/i)).toBeInTheDocument();
      });
    });
  });

  describe('Authentication', () => {
    test('includes auth token in request when user is authenticated', async () => {
      // Arrange
      const authToken = 'test-jwt-token';
      (localStorage.getItem as jest.Mock).mockReturnValue(authToken);
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({ answer_text: 'Answer', sources: [] }),
      });

      render(<Chatbot />);

      // Act
      await userEvent.type(screen.getByPlaceholderText(/ask a question/i), 'Test{Enter}');

      // Assert
      await waitFor(() => {
        expect(fetch).toHaveBeenCalledWith(
          expect.any(String),
          expect.objectContaining({
            headers: expect.objectContaining({
              'Authorization': `Bearer ${authToken}`,
            }),
          })
        );
      });
    });

    test('includes session ID when user is not authenticated', async () => {
      // Arrange
      (localStorage.getItem as jest.Mock).mockReturnValue(null);
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({ answer_text: 'Answer', sources: [] }),
      });

      render(<Chatbot />);

      // Act
      await userEvent.type(screen.getByPlaceholderText(/ask a question/i), 'Test{Enter}');

      // Assert
      await waitFor(() => {
        expect(fetch).toHaveBeenCalledWith(
          expect.any(String),
          expect.objectContaining({
            body: expect.stringContaining('session_id'),
          })
        );
      });
    });
  });

  describe('Agent Skills', () => {
    test('executes explain skill when button clicked', async () => {
      // Arrange
      global.prompt = jest.fn().mockReturnValue('ROS 2 topic');
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          output: 'A ROS 2 topic is a named bus for messages',
          model_used: 'gpt-4o-mini',
          skill_name: 'explain-term',
        }),
      });

      render(<Chatbot />);
      const explainButton = screen.getByText(/explain term/i);

      // Act
      fireEvent.click(explainButton);

      // Assert
      await waitFor(() => {
        expect(fetch).toHaveBeenCalledWith(
          'http://localhost:8000/skills/explain',
          expect.objectContaining({
            method: 'POST',
          })
        );
      });
    });

    test('displays skill output with badge', async () => {
      // Arrange
      global.prompt = jest.fn().mockReturnValue('quaternion');
      (fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          output: 'A quaternion is a mathematical representation...',
          model_used: 'gpt-4o-mini',
          skill_name: 'explain-term',
        }),
      });

      render(<Chatbot />);

      // Act
      fireEvent.click(screen.getByText(/explain term/i));

      // Assert
      await waitFor(() => {
        expect(screen.getByText(/explain-term/i)).toBeInTheDocument();
      });
    });
  });
});
