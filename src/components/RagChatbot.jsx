import { useState, useRef, useEffect } from 'react';
import { FaComments } from 'react-icons/fa';
import './Chatbot.css';

export default function Chatbot() {
  const [open, setOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);

  // 🔽 Auto-scroll reference
  const messagesEndRef = useRef(null);

  // 🔽 Scroll to bottom when messages update
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages, loading]);

  const sendMessage = async () => {
    if (!input.trim() || loading) return;

    const userMessage = { sender: "user", text: input };
    setMessages(prev => [...prev, userMessage]);

    const currentInput = input;
    setInput("");
    setLoading(true);

    try {
      const selected_text = window.getSelection()?.toString() || "";

      const res = await fetch("https://muhammadanasqadri-backend.hf.space/api/chat", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          question: currentInput,
          selected_text,
        }),
      });

      const data = await res.json();

      if (!res.ok) {
        throw new Error(data.detail || res.statusText);
      }

      const botReply = data.answer || "No response from server.";
      setMessages(prev => [...prev, { sender: "bot", text: botReply }]);
    } catch (err) {
      let errorMsg = err.message || "Unable to reach server";

      if (errorMsg.includes("401") || errorMsg.includes("Unauthorized")) {
        errorMsg = "Backend configuration error. API keys may be missing.";
      } else if (errorMsg.includes("500")) {
        errorMsg = "Backend server error. Please try again later.";
      } else if (errorMsg.includes("Not Found")) {
        errorMsg = "API endpoint not found.";
      }

      setMessages(prev => [
        ...prev,
        { sender: "bot", text: `Backend Error: ${errorMsg}` },
      ]);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === "Enter" && !loading) {
      sendMessage();
    }
  };

  return (
    <div className="chat-container">
      <button className="chat-button" onClick={() => setOpen(!open)}>
        <FaComments size={24} />
      </button>

      {open && (
        <div className="chat-box">
          <div className="chat-header">
            <strong>Physical AI</strong>
            <button className="close-btn" onClick={() => setOpen(false)}>
              ×
            </button>
          </div>

          <div className="chat-body">
            {messages.map((m, i) => (
              <div key={i} className={`bubble ${m.sender}`}>
                {m.text}
              </div>
            ))}

            {loading && (
              <div className="bubble bot">
                <em>Typing...</em>
              </div>
            )}

            {/* 🔽 Auto-scroll anchor */}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input">
            <input
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Type your message..."
              disabled={loading}
            />
            <button onClick={sendMessage} disabled={loading}>
              {loading ? "..." : "Send"}
            </button>
          </div>
        </div>
      )}
    </div>
  );
}