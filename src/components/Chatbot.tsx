import { useState } from 'react';
import { FaComments } from 'react-icons/fa';
import './Chatbot.css';

export default function Chatbot() {
  const [open, setOpen] = useState(false);
  const [messages, setMessages] = useState<{ sender: string; text: string }[]>([]);
  const [input, setInput] = useState("");
  const [loading, setLoading] = useState(false);

  const sendMessage = async () => {
    if (!input.trim()) return;

    const userMessage = { sender: "user", text: input };
    setMessages(prev => [...prev, userMessage]);
    const currentInput = input;
    setInput("");
    setLoading(true);

    try {
      // Get selected text, if any
      const selected_text = window.getSelection()?.toString();

      const res = await fetch("https://muhammadanasqadri-backend.hf.space/", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          query: currentInput,
          selected_text: selected_text,
        }),
      });

      const data = await res.json();

      if (!res.ok) {
        throw new Error(data.detail || `Request failed with status: ${res.statusText}`);
      }
      
      if (data.detail) {
          throw new Error(data.detail);
      }

      const botReply = data.answer || "No response from server";

      setMessages(prev => [...prev, { sender: "bot", text: botReply }]);
    } catch (err: any) {
      const errorMsg = err.message || "Unable to reach server";
      setMessages(prev => [...prev, { sender: "bot", text: `Backend Error: ${errorMsg}` }]);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter' && !loading) sendMessage();
  };

  return (
    <div className="chat-container">
      {/* Floating Chat Icon */}
      <button className="chat-button" onClick={() => setOpen(!open)}>
        <FaComments size={24} />
      </button>

      {/* Chat Box */}
      {open && (
        <div className="chat-box">
          <div className="chat-header">
            <strong>Physical AI</strong>
            <button className="close-btn" onClick={() => setOpen(false)}>×</button>
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
          </div>

          <div className="chat-input">
            <input
              value={input}
              onChange={e => setInput(e.target.value)}
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
