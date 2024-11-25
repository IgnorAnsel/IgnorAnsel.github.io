async function fetchFileList() {
    try {
        // 加载 filelist.json
        const response = await fetch('urdf_files.json');
        if (!response.ok) {
            throw new Error(`无法加载文件列表：${response.status}`);
        }
        const fileList = await response.json();
        createFileAccordion(fileList);
    } catch (error) {
        console.error('加载文件列表失败:', error);
        document.getElementById('file-list').textContent = '加载文件列表失败，请检查 filelist.json 是否存在。';
    }
}

function createFileAccordion(fileList) {
    const container = document.getElementById('file-list');
    container.innerHTML = ''; // 清空加载中的提示

    fileList.forEach((fileName) => {
        // 创建文件折叠栏
        const fileContainer = document.createElement('div');
        fileContainer.classList.add('file-container');

        const fileHeader = document.createElement('button');
        fileHeader.classList.add('file-header');
        fileHeader.textContent = fileName;

        const fileContent = document.createElement('div');
        fileContent.classList.add('file-content');
        fileContent.style.display = 'none';

        // 点击折叠栏加载并显示文件内容
        fileHeader.addEventListener('click', async () => {
            if (fileContent.textContent === '') {
                fileContent.textContent = '加载中...';
                try {
                    const response = await fetch(`Study/urdf/${fileName}`);
                    if (!response.ok) {
                        throw new Error(`无法加载文件内容：${response.status}`);
                    }
                    const fileData = await response.text();
                    fileContent.textContent = fileData;
                } catch (error) {
                    console.error('加载文件内容失败:', error);
                    fileContent.textContent = '加载文件内容失败，请检查路径或文件。';
                }
            }
            // 切换显示/隐藏
            fileContent.style.display =
                fileContent.style.display === 'none' ? 'block' : 'none';
        });

        // 将文件头部和内容添加到容器中
        fileContainer.appendChild(fileHeader);
        fileContainer.appendChild(fileContent);
        container.appendChild(fileContainer);
    });
}

// 页面加载完成后调用
window.onload = fetchFileList;
