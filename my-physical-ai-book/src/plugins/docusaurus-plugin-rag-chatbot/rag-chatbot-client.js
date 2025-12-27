/**
 * RAG Chatbot Client-side Integration for Docusaurus
 * This script initializes the chatbot widget when the page loads
 */

// Create a global configuration object if it doesn't exist
if (typeof window.RAG_CHATBOT_CONFIG === 'undefined') {
  window.RAG_CHATBOT_CONFIG = {
    backendUrl: process.env.REACT_APP_BACKEND_URL || 'http://localhost:8000',
    position: 'bottom-right',
    title: 'Ask about this book',
    initiallyOpen: false
  };
}

// Define the API client
class ApiClient {
  constructor(baseURL) {
    this.baseURL = baseURL || window.RAG_CHATBOT_CONFIG?.backendUrl || 'http://localhost:8000';
  }

  /**
   * Ask a question to the chatbot
   * @param {string} question - The question to ask
   * @param {string} sessionId - Optional session ID for conversation context
   * @param {Object} pageContext - Optional context about the current page
   * @returns {Promise<Object>} The response from the chatbot
   */
  async askQuestion(question, sessionId = null, pageContext = null) {
    try {
      const response = await fetch(`${this.baseURL}/api/ask`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question,
          session_id: sessionId,
          page_context: pageContext || {}
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status_code}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error asking question:', error);
      throw error;
    }
  }

  /**
   * Ask a question based on selected text
   * @param {string} question - The question to ask
   * @param {string} selectedText - The text selected by the user
   * @param {string} sessionId - Optional session ID for conversation context
   * @param {Object} pageContext - Optional context about the current page
   * @returns {Promise<Object>} The response from the chatbot
   */
  async askWithSelectedText(question, selectedText, sessionId = null, pageContext = null) {
    try {
      const response = await fetch(`${this.baseURL}/api/ask-selected`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question,
          selected_text: selectedText,
          session_id: sessionId,
          page_context: pageContext || {}
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status_code}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error asking with selected text:', error);
      throw error;
    }
  }

  /**
   * Check the health of the backend service
   * @returns {Promise<Object>} Health check response
   */
  async healthCheck() {
    try {
      const response = await fetch(`${this.baseURL}/api/health`);

      if (!response.ok) {
        throw new Error(`Health check failed with status: ${response.status_code}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Health check failed:', error);
      throw error;
    }
  }
}

/**
 * ChatWidget Component
 * Main component for the RAG chatbot widget
 */
class ChatWidget {
  constructor(containerId, options = {}) {
    this.containerId = containerId;
    this.options = {
      backendUrl: options.backendUrl || window.RAG_CHATBOT_CONFIG?.backendUrl || 'http://localhost:8000',
      position: options.position || window.RAG_CHATBOT_CONFIG?.position || 'bottom-right',
      initiallyOpen: options.initiallyOpen || window.RAG_CHATBOT_CONFIG?.initiallyOpen || false,
      title: options.title || window.RAG_CHATBOT_CONFIG?.title || 'Ask about this book',
      ...options
    };

    this.apiClient = new ApiClient(this.options.backendUrl);
    this.isOpen = this.options.initiallyOpen;
    this.sessionId = this.generateSessionId();
    this.pageContext = this.getPageContext();
    this.selectedText = '';

    this.initializeWidget();
  }

  generateSessionId() {
    return 'session_' + Date.now() + '_' + Math.random().toString(36).substr(2, 9);
  }

  getPageContext() {
    return {
      url: window.location.href,
      title: document.title,
      section: this.getCurrentSection()
    };
  }

  getCurrentSection() {
    // Try to extract current section from URL or page structure
    const h1 = document.querySelector('h1');
    const h2 = document.querySelector('h2');

    if (h2) return h2.textContent.trim();
    if (h1) return h1.textContent.trim();

    return 'Unknown Section';
  }

  initializeWidget() {
    this.createWidgetHTML();
    this.attachEventListeners();

    // Add event listener for text selection
    document.addEventListener('mouseup', this.handleTextSelection.bind(this));

    // Initialize with widget closed or open based on options
    if (!this.options.initiallyOpen) {
      this.closeWidget();
    }
  }

  createWidgetHTML() {
    const container = document.getElementById(this.containerId);
    if (!container) {
      console.error(`Container with id '${this.containerId}' not found`);
      return;
    }

    container.innerHTML = `
      <div id="rag-chatbot-widget" class="rag-chatbot-widget ${this.options.position}">
        <div id="rag-chatbot-header" class="rag-chatbot-header">
          <div class="rag-chatbot-title">${this.options.title}</div>
          <button id="rag-chatbot-toggle" class="rag-chatbot-toggle">
            ${this.isOpen ? '−' : '+'}
          </button>
        </div>
        <div id="rag-chatbot-content" class="rag-chatbot-content">
          <div id="rag-chatbot-messages" class="rag-chatbot-messages">
            <div class="rag-chatbot-message rag-chatbot-message-bot">
              Hello! I'm your book assistant. Ask me anything about this content.
            </div>
          </div>
          <div class="rag-chatbot-input-area">
            <textarea
              id="rag-chatbot-input"
              class="rag-chatbot-input"
              placeholder="Ask a question about this book..."
              rows="2"
            ></textarea>
            <button id="rag-chatbot-send" class="rag-chatbot-send">Send</button>
          </div>
          <div id="rag-chatbot-status" class="rag-chatbot-status"></div>
        </div>
      </div>
    `;
  }

  attachEventListeners() {
    // Toggle button
    document.getElementById('rag-chatbot-toggle').addEventListener('click', () => {
      this.toggleWidget();
    });

    // Send button
    document.getElementById('rag-chatbot-send').addEventListener('click', () => {
      this.handleSendMessage();
    });

    // Input key events
    const input = document.getElementById('rag-chatbot-input');
    input.addEventListener('keydown', (e) => {
      if (e.key === 'Enter' && !e.shiftKey) {
        e.preventDefault();
        this.handleSendMessage();
      }
    });

    // Add visual indicator when text is selected
    input.addEventListener('focus', () => {
      this.clearSelectedTextIndicator();
    });
  }

  handleTextSelection() {
    const selectedText = window.getSelection().toString().trim();

    if (selectedText && selectedText.length > 0) {
      this.selectedText = selectedText;

      // Add visual feedback
      this.showSelectedTextIndicator(selectedText);
    }
  }

  showSelectedTextIndicator(text) {
    const statusEl = document.getElementById('rag-chatbot-status');
    if (statusEl) {
      statusEl.innerHTML = `
        <div class="rag-chatbot-selected-text">
          Using selected text: "${text.substring(0, 50)}${text.length > 50 ? '...' : ''}"
          <button class="rag-chatbot-clear-selection" onclick="window.ragChatWidget.clearSelectedText()">Clear</button>
        </div>
      `;
    }
  }

  clearSelectedTextIndicator() {
    const statusEl = document.getElementById('rag-chatbot-status');
    if (statusEl) {
      statusEl.innerHTML = '';
    }
  }

  clearSelectedText() {
    this.selectedText = '';
    this.clearSelectedTextIndicator();
  }

  toggleWidget() {
    if (this.isOpen) {
      this.closeWidget();
    } else {
      this.openWidget();
    }
  }

  openWidget() {
    const widget = document.getElementById('rag-chatbot-widget');
    const toggleBtn = document.getElementById('rag-chatbot-toggle');

    if (widget) widget.classList.remove('rag-chatbot-widget-closed');
    if (toggleBtn) toggleBtn.textContent = '−';

    this.isOpen = true;
  }

  closeWidget() {
    const widget = document.getElementById('rag-chatbot-widget');
    const toggleBtn = document.getElementById('rag-chatbot-toggle');

    if (widget) widget.classList.add('rag-chatbot-widget-closed');
    if (toggleBtn) toggleBtn.textContent = '+';

    this.isOpen = false;
  }

  async handleSendMessage() {
    const input = document.getElementById('rag-chatbot-input');
    const message = input.value.trim();

    if (!message) return;

    // Add user message to chat
    this.addMessage(message, 'user');

    // Clear input
    input.value = '';

    // Show typing indicator
    this.showTypingIndicator();

    try {
      // Determine which API to call based on whether text is selected
      let response;
      if (this.selectedText) {
        response = await this.apiClient.askWithSelectedText(
          message,
          this.selectedText,
          this.sessionId,
          this.pageContext
        );
      } else {
        response = await this.apiClient.askQuestion(
          message,
          this.sessionId,
          this.pageContext
        );
      }

      // Add bot response to chat
      this.addMessage(response.answer, 'bot', response.sources);

      // Clear selected text after use
      this.clearSelectedText();
    } catch (error) {
      console.error('Error sending message:', error);
      this.addMessage('Sorry, I encountered an error. Please try again.', 'bot');
    } finally {
      // Remove typing indicator
      this.removeTypingIndicator();
    }
  }

  addMessage(text, sender, sources = null) {
    const messagesContainer = document.getElementById('rag-chatbot-messages');
    if (!messagesContainer) return;

    const messageClass = sender === 'user' ? 'rag-chatbot-message-user' : 'rag-chatbot-message-bot';
    const messageElement = document.createElement('div');
    messageElement.className = `rag-chatbot-message ${messageClass}`;

    // Add the message text
    messageElement.innerHTML = `<div class="rag-chatbot-message-text">${this.escapeHtml(text)}</div>`;

    // Add sources if provided
    if (sources && sources.length > 0) {
      const sourcesHtml = sources.map(source =>
        `<div class="rag-chatbot-source">
          <div class="rag-chatbot-source-title">Source:</div>
          <div class="rag-chatbot-source-content">${this.escapeHtml(source.content_snippet)}</div>
        </div>`
      ).join('');

      messageElement.innerHTML += `<div class="rag-chatbot-sources">${sourcesHtml}</div>`;
    }

    messagesContainer.appendChild(messageElement);

    // Scroll to bottom
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }

  showTypingIndicator() {
    const messagesContainer = document.getElementById('rag-chatbot-messages');
    if (!messagesContainer) return;

    const typingElement = document.createElement('div');
    typingElement.id = 'rag-chatbot-typing';
    typingElement.className = 'rag-chatbot-message rag-chatbot-message-bot rag-chatbot-typing';
    typingElement.innerHTML = `
      <div class="rag-chatbot-message-text">
        <span class="rag-chatbot-typing-dots">
          <span>.</span><span>.</span><span>.</span>
        </span>
      </div>
    `;

    messagesContainer.appendChild(typingElement);
    messagesContainer.scrollTop = messagesContainer.scrollHeight;
  }

  removeTypingIndicator() {
    const typingElement = document.getElementById('rag-chatbot-typing');
    if (typingElement) {
      typingElement.remove();
    }
  }

  escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
  }
}

// Initialize the chatbot when the DOM is loaded
function initializeChatbot() {
  // Create a container for the chatbot widget
  let container = document.getElementById('rag-chatbot-container');
  if (!container) {
    container = document.createElement('div');
    container.id = 'rag-chatbot-container';
    document.body.appendChild(container);
  }

  // Initialize the chatbot widget
  window.ragChatWidget = new ChatWidget('rag-chatbot-container', window.RAG_CHATBOT_CONFIG || {});
}

// Wait for the page to be fully loaded before initializing
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initializeChatbot);
} else {
  // DOM is already loaded, initialize immediately
  initializeChatbot();
}

// Add CSS styles dynamically
if (!document.getElementById('rag-chatbot-styles')) {
  const style = document.createElement('style');
  style.id = 'rag-chatbot-styles';
  style.textContent = `
    .rag-chatbot-widget {
      position: fixed;
      bottom: 20px;
      right: 20px;
      width: 350px;
      max-height: 500px;
      display: flex;
      flex-direction: column;
      border-radius: 10px;
      box-shadow: 0 4px 20px rgba(0, 0, 0, 0.15);
      background: white;
      overflow: hidden;
      z-index: 10000;
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, Cantarell, 'Open Sans', 'Helvetica Neue', sans-serif;
    }

    .rag-chatbot-header {
      background: #2563eb;
      color: white;
      padding: 12px 16px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      cursor: pointer;
    }

    .rag-chatbot-title {
      font-weight: 600;
      font-size: 14px;
    }

    .rag-chatbot-toggle {
      background: none;
      border: none;
      color: white;
      font-size: 18px;
      cursor: pointer;
      padding: 0;
      width: 24px;
      height: 24px;
      display: flex;
      align-items: center;
      justify-content: center;
    }

    .rag-chatbot-content {
      display: flex;
      flex-direction: column;
      height: 400px;
    }

    .rag-chatbot-messages {
      flex: 1;
      overflow-y: auto;
      padding: 16px;
      display: flex;
      flex-direction: column;
      gap: 12px;
    }

    .rag-chatbot-message {
      max-width: 85%;
      padding: 10px 14px;
      border-radius: 18px;
      font-size: 14px;
      line-height: 1.4;
    }

    .rag-chatbot-message-user {
      align-self: flex-end;
      background: #2563eb;
      color: white;
    }

    .rag-chatbot-message-bot {
      align-self: flex-start;
      background: #f3f4f6;
      color: #374151;
    }

    .rag-chatbot-input-area {
      display: flex;
      padding: 12px;
      border-top: 1px solid #e5e7eb;
      background: white;
    }

    .rag-chatbot-input {
      flex: 1;
      padding: 10px 12px;
      border: 1px solid #d1d5db;
      border-radius: 20px;
      resize: none;
      font-size: 14px;
      font-family: inherit;
      max-height: 80px;
    }

    .rag-chatbot-input:focus {
      outline: none;
      border-color: #2563eb;
      box-shadow: 0 0 0 2px rgba(37, 99, 235, 0.1);
    }

    .rag-chatbot-send {
      background: #2563eb;
      color: white;
      border: none;
      border-radius: 20px;
      padding: 0 16px;
      margin-left: 8px;
      cursor: pointer;
      font-weight: 500;
    }

    .rag-chatbot-send:hover {
      background: #1d4ed8;
    }

    .rag-chatbot-typing-dots {
      display: flex;
      align-items: center;
      gap: 4px;
    }

    .rag-chatbot-typing-dots span {
      display: inline-block;
      width: 6px;
      height: 6px;
      background: #6b7280;
      border-radius: 50%;
      animation: typing 1.4s infinite ease-in-out;
    }

    .rag-chatbot-typing-dots span:nth-child(2) {
      animation-delay: 0.2s;
    }

    .rag-chatbot-typing-dots span:nth-child(3) {
      animation-delay: 0.4s;
    }

    @keyframes typing {
      0%, 60%, 100% { transform: translateY(0); }
      30% { transform: translateY(-5px); }
    }

    .rag-chatbot-status {
      padding: 8px 12px;
      font-size: 12px;
      color: #6b7280;
      border-top: 1px solid #e5e7eb;
      background: #f9fafb;
    }

    .rag-chatbot-selected-text {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding: 4px 0;
    }

    .rag-chatbot-clear-selection {
      background: #ef4444;
      color: white;
      border: none;
      border-radius: 12px;
      padding: 4px 8px;
      font-size: 11px;
      cursor: pointer;
    }

    .rag-chatbot-sources {
      margin-top: 8px;
      padding-top: 8px;
      border-top: 1px solid #e5e7eb;
    }

    .rag-chatbot-source {
      margin-top: 8px;
      padding: 8px;
      background: #f9fafb;
      border-radius: 6px;
      font-size: 12px;
    }

    .rag-chatbot-source-title {
      font-weight: 600;
      color: #374151;
      margin-bottom: 4px;
    }

    .rag-chatbot-source-content {
      color: #6b7280;
    }

    .rag-chatbot-widget-closed {
      width: 60px;
      height: 60px;
    }

    .rag-chatbot-widget-closed .rag-chatbot-content,
    .rag-chatbot-widget-closed .rag-chatbot-header > .rag-chatbot-title {
      display: none;
    }

    .rag-chatbot-widget-closed .rag-chatbot-toggle {
      position: relative;
      width: 100%;
      height: 100%;
      font-size: 24px;
      display: flex;
      align-items: center;
      justify-content: center;
    }

    @media (max-width: 768px) {
      .rag-chatbot-widget {
        width: calc(100% - 40px);
        left: 20px;
        right: 20px;
        max-height: 50vh;
      }
    }
  `;
  document.head.appendChild(style);
}