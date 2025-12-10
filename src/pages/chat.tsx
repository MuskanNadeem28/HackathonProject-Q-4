import React, { useState, useEffect, useRef } from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import './chat.css';

const ChatPage = () => {
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [contextWindow, setContextWindow] = useState('full_book'); // Default to full book
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message to the chat
    const userMessage = { role: 'user', content: inputValue, timestamp: new Date() };
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API
      const response = await fetch('http://localhost:8000/api/v1/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          context_window: contextWindow,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Add assistant message to the chat
      const assistantMessage = {
        role: 'assistant',
        content: data.response,
        sources: data.sources,
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error:', error);
      const errorMessage = {
        role: 'assistant',
        content: `Sorry, I encountered an error: ${error.message}. Please try again.`,
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const clearChat = () => {
    setMessages([]);
  };

  return (
    <Layout title="RAG Chat" description="Chat with the Physical AI & Humanoid Robotics book using RAG">
      <div className="chat-container">
        <div className="chat-header">
          <h1>Physical AI & Humanoid Robotics RAG Chat</h1>
          <p>Ask questions about the book content using Retrieval-Augmented Generation</p>

          <div className="context-selector">
            <label htmlFor="context-window">Context Window:</label>
            <select
              id="context-window"
              value={contextWindow}
              onChange={(e) => setContextWindow(e.target.value)}
              disabled={isLoading}
            >
              <option value="full_book">Full Book</option>
              <option value="module_ros2">Module 1: ROS 2</option>
              <option value="module_simulation">Module 2: Simulation Environments</option>
              <option value="module_isaac">Module 3: NVIDIA Isaac</option>
              <option value="module_vla">Module 4: VLA Systems</option>
              <option value="module_capstone">Capstone: Autonomous Humanoid</option>
            </select>
          </div>
        </div>

        <div className="chat-messages">
          {messages.length === 0 ? (
            <div className="welcome-message">
              <h2>Welcome to the Physical AI & Humanoid Robotics RAG Chat!</h2>
              <p>Ask me anything about the book content. I'll search the relevant sections and provide accurate answers based on the book material.</p>
              <div className="example-questions">
                <h3>Try asking:</h3>
                <ul>
                  <li>What is ROS 2 and how does it differ from ROS 1?</li>
                  <li>Explain the architecture of NVIDIA Isaac platform</li>
                  <li>How do Vision-Language-Action systems work in robotics?</li>
                  <li>What are the key components of a humanoid robot?</li>
                </ul>
              </div>
            </div>
          ) : (
            messages.map((message, index) => (
              <div key={index} className={`message ${message.role}`}>
                <div className="message-content">
                  <div className="message-text">{message.content}</div>
                  {message.sources && message.sources.length > 0 && (
                    <div className="sources">
                      <details>
                        <summary>Sources</summary>
                        <ul>
                          {message.sources.map((source, idx) => (
                            <li key={idx}>
                              <strong>{source.title}</strong>
                              <p>{source.content.substring(0, 100)}...</p>
                            </li>
                          ))}
                        </ul>
                      </details>
                    </div>
                  )}
                </div>
              </div>
            ))
          )}
          {isLoading && (
            <div className="message assistant">
              <div className="message-content">
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <form className="chat-input-form" onSubmit={handleSubmit}>
          <input
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder="Ask a question about the Physical AI & Humanoid Robotics book..."
            disabled={isLoading}
          />
          <button type="submit" disabled={isLoading || !inputValue.trim()}>
            {isLoading ? 'Sending...' : 'Send'}
          </button>
          <button type="button" onClick={clearChat} disabled={isLoading}>
            Clear
          </button>
        </form>
      </div>
    </Layout>
  );
};

export default ChatPage;