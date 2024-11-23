
// JavaScript 用于切换选项卡
function showTab(tabName) {
    // 隐藏所有内容区域
    var tabs = document.querySelectorAll('.tab-content');
    tabs.forEach(function(tab) {
        tab.style.display = 'none';
    });

    // 显示选中的内容区域
    var activeTab = document.getElementById(tabName);
    activeTab.style.display = 'block';

    // 高亮当前选中的按钮
    var buttons = document.querySelectorAll('.tab-button');
    buttons.forEach(function(button) {
        button.classList.remove('active'); // 移除所有按钮的 active 类
    });

    var activeButton = document.querySelector(`[data-tab="${tabName}"]`);
    activeButton.classList.add('active'); // 给当前按钮添加 active 类
}

// 猜数字游戏相关变量
let randomNumber = Math.floor(Math.random() * 100) + 1; // 随机生成1到100的数字
let attempts = 0; // 用户尝试的次数

// 游戏逻辑：检查用户输入的猜测
function checkGuess() {
    const userGuess = document.getElementById('user-guess').value;
    const messageElement = document.getElementById('message');
    const attemptsElement = document.getElementById('attempts');
    
    attempts++; // 增加尝试次数
    attemptsElement.innerText = `尝试次数: ${attempts}`; // 更新尝试次数

    if (userGuess < randomNumber) {
        messageElement.innerText = '猜的数字太小了！试试大一点的数字。';
        messageElement.style.color = 'red';
    } else if (userGuess > randomNumber) {
        messageElement.innerText = '猜的数字太大了！试试小一点的数字。';
        messageElement.style.color = 'red';
    } else {
        messageElement.innerText = `恭喜你！猜对了！正确答案是 ${randomNumber}。`;
        messageElement.style.color = 'green';
        // 游戏结束后，可以重置游戏
        randomNumber = Math.floor(Math.random() * 100) + 1; // 重新生成一个随机数字
        attempts = 0; // 重置尝试次数
        attemptsElement.innerText = `尝试次数: ${attempts}`;
    }
}

