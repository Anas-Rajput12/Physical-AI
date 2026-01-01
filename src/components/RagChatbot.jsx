import { useState, useRef, useEffect } from "react";
import { FaComments, FaTrash } from "react-icons/fa";
import "./Chatbot.css";

const BACKEND_URL =
  import.meta.env.REACT_APP_BACKEND_URL || "http://127.0.0.1:8000";

export default function RagChatbot() {
  const [open, setOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);

  const chatEndRef = useRef(null);

  // 🔽 Auto-scroll to bottom
  useEffect(() => {
    chatEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages, loading]);

  const sendMessage = async () => {
    if (!input.trim() || loading) return;

    const userMessage = { sender: "user", text: input };
    setMessages((prev) => [...prev, userMessage]);

    const currentInput = input;
    setInput("");
    setLoading(true);

    try {
      const selectedText = window.getSelection()?.toString() || "";
      const hasSelectedText = selectedText.trim().length > 0;

      const endpoint = hasSelectedText ? "/api/ask-selected" : "/api/ask";

      const pageContext = {
        url: window.location.href,
        title: document.title,
        section:
          document.querySelector("h1")?.textContent ||
          document.querySelector("h2")?.textContent ||
          "Unknown",
      };

      const body = hasSelectedText
        ? {
            question: currentInput,
            selected_text: selectedText,
            session_id: null,
            page_context: pageContext,
          }
        : {
            question: currentInput,
            session_id: null,
            page_context: pageContext,
          };

      const response = await fetch(BACKEND_URL + endpoint, {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
          Accept: "application/json",
        },
        body: JSON.stringify(body),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || "Request failed");
      }

      setMessages((prev) => [
        ...prev,
        { sender: "bot", text: data.answer || "No response" },
      ]);
    } catch (error) {
      setMessages((prev) => [
        ...prev,
        { sender: "bot", text: `Error: ${error.message}` },
      ]);
    } finally {
      setLoading(false);
    }
  };

  const clearChat = () => setMessages([]);

  const handleKeyPress = (e) => {
    if (e.key === "Enter") sendMessage();
  };

  return (
    <div className="chat-container">
      {/* Floating Button */}
      <button className="chat-button" onClick={() => setOpen(!open)}>
        <FaComments size={22} />
      </button>

      {open && (
        <div className="chat-box">
          <div className="chat-header">
            <strong>Physical AI</strong>
            <div>
              <button className="icon-btn" onClick={clearChat}>
                <FaTrash />
              </button>
              <button className="close-btn" onClick={() => setOpen(false)}>
                ×
              </button>
            </div>
          </div>

          <div className="chat-body">
            {messages.map((msg, i) => (
              <div key={i} className={`bubble ${msg.sender}`}>
                {msg.text}
              </div>
            ))}

            {loading && <div className="bubble bot">Typing...</div>}
            <div ref={chatEndRef} />
          </div>

          <div className="chat-input">
            <input
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyDown={handleKeyPress}
              placeholder="Type message..."
              disabled={loading}
            />
            <button onClick={sendMessage} disabled={loading}>
              Send
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
