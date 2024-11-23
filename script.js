// JavaScript 用于处理聊天功能
function sendMessage() {
    var userInput = document.getElementById('user-input').value;
    if (userInput.trim() === "") {
        alert("请输入消息");
        return;
    }

    // 显示用户输入
    var chatLog = document.getElementById('chat-log');
    var userMessage = document.createElement('p');
    userMessage.classList.add('user-message');
    userMessage.innerText = '你: ' + userInput;
    chatLog.appendChild(userMessage);

    // 清空输入框
    document.getElementById('user-input').value = '';

    // 调用 OpenAI API
    fetch('https://api.openai.com/v1/chat/completions', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
            'Authorization': 'Bearer Token-NjVhMGQ3ZTUxYjk5YmMwMDEzMDY4MjBmOlk3blhtRmJtTFBRQVU1NHZMWA=='  // 你的 API 密钥
        },
        body: JSON.stringify({
            model: 'gpt-3.5-turbo',  // 使用 GPT-3.5 模型
            messages: [
                { role: 'system', content: '你是一个帮助用户的聊天机器人。' },
                { role: 'user', content: userInput }
            ]
        })
    })
    .then(response => response.json())
    .then(data => {
        // 显示 ChatGPT 回复
        var aiMessage = document.createElement('p');
        aiMessage.classList.add('ai-message');
        aiMessage.innerText = 'AI: ' + data.choices[0].message.content;
        chatLog.appendChild(aiMessage);

        // 滚动到底部
        chatLog.scrollTop = chatLog.scrollHeight;
    })
    .catch(error => {
        console.error('Error:', error);
    });
}
