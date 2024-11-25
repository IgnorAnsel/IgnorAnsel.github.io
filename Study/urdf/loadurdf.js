// 定义加载 URDF 文件的函数
async function loadURDF() {
    try {
        // 读取文件路径
        const response = await fetch('test.urdf');
        if (!response.ok) {
            throw new Error(`HTTP 错误！状态码：${response.status}`);
        }
        // 获取文件内容
        const urdfContent = await response.text();
        document.getElementById('urdf-content').textContent = urdfContent;
    } catch (error) {
        console.error('无法加载 URDF 文件:', error);
        document.getElementById('urdf-content').textContent = '无法加载文件，请检查路径或文件是否存在。';
    }
}

// 页面加载完成后调用
window.onload = loadURDF;
