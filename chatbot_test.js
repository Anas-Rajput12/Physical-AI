// Test script to simulate chatbot functionality
async function testChatbot() {
    console.log("Testing RAG Chatbot API connection...");

    const testQuestions = [
        "What is a RAG chatbot?",
        "Hello",
        "How does this work?",
        "What can you do?"
    ];

    for (const question of testQuestions) {
        console.log(`\nTesting question: "${question}"`);

        try {
            const response = await fetch("https://muhammadanasqadri-backend.hf.space/api/chat", {
                method: "POST",
                headers: {
                    "Content-Type": "application/json"
                },
                body: JSON.stringify({
                    question: question,
                    selected_text: ""
                })
            });

            console.log(`Response status: ${response.status}`);

            const data = await response.json();
            console.log(`Response data:`, data);

            if (response.ok) {
                console.log(`✅ Answer: ${data.answer || 'No answer field'}`);
                console.log(`✅ Sources: ${data.sources ? data.sources.length : 0}`);
            } else {
                console.log(`❌ Error: ${data.detail || 'Unknown error'}`);
            }
        } catch (error) {
            console.log(`❌ Network Error: ${error.message}`);
        }

        // Add delay between requests to avoid overwhelming the server
        await new Promise(resolve => setTimeout(resolve, 1000));
    }

    console.log("\n--- Test Complete ---");
}

// Run the test
testChatbot().catch(console.error);