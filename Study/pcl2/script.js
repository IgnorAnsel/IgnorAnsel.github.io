// 加载 JSON 文件，获取 Markdown 文件列表
fetch('files.json')  // 直接通过相对路径加载 JSON 文件
    .then(response => response.json())
    .then(data => {
        const fileListElement = document.getElementById('file-list');
        data.files.forEach(file => {
            const listItem = document.createElement('li');
            listItem.textContent = file.name;
            listItem.onclick = () => loadMarkdown(file.path);  // 加载 Markdown 文件
            fileListElement.appendChild(listItem);
        });
    })
    .catch(error => {
        console.error('加载 JSON 文件时出错:', error);
        document.getElementById('file-list').innerHTML = '<p>无法加载文件列表。</p>';
    });

// 根据路径加载和渲染 Markdown 文件
function loadMarkdown(filePath) {
    const baseUrl = "https://ignoransel.github.io/Study/pcl2/";
    const fullUrl = new URL(filePath, baseUrl).href;  // 自动处理路径拼接
    fetch(fullUrl)
        .then(response => response.text())
        .then(markdownContent => {
            const htmlContent = marked(markdownContent);  // 使用 marked 将 Markdown 转换为 HTML
            document.getElementById('file-content').innerHTML = htmlContent;
        })
        .catch(error => {
            console.error('加载 Markdown 文件时出错:', error);
            document.getElementById('file-content').innerHTML = '<p>无法加载文件内容。</p>';
        });
}

