// 使用相对路径来访问本地的 Markdown 文件
const jsonFilePath = 'files.json'; // JSON 文件路径

// 加载 JSON 文件，获取 Markdown 文件列表
fetch(jsonFilePath)
    .then(response => response.json())
    .then(data => {
        const fileListElement = document.getElementById('file-list');
        data.files.forEach(file => {
            const listItem = document.createElement('li');
            listItem.textContent = file.name;
            listItem.onclick = () => loadMarkdown(file.path);
            fileListElement.appendChild(listItem);
        });
    })
    .catch(error => {
        console.error('加载 JSON 文件时出错:', error);
        document.getElementById('file-list').innerHTML = '<p>无法加载文件列表。</p>';
    });

// 根据路径加载和渲染 Markdown 文件
function loadMarkdown(filePath) {
    fetch(filePath) // 直接通过相对路径访问本地文件
        .then(response => response.text())
        .then(markdownContent => {
            const htmlContent = marked(markdownContent);  // 使用 marked 库将 Markdown 转换为 HTML
            document.getElementById('file-content').innerHTML = htmlContent;
        })
        .catch(error => {
            console.error('加载 Markdown 文件时出错:', error);
            document.getElementById('file-content').innerHTML = '<p>无法加载文件内容。</p>';
        });
}
