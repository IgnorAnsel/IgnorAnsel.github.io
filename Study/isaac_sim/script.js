// 获取所有的按钮
const toggleButtons = document.querySelectorAll('.toggle-btn');

// 为每个按钮添加点击事件
toggleButtons.forEach(button => {
  button.addEventListener('click', function () {
    // 切换当前按钮对应的子菜单显示/隐藏
    const submenu = this.nextElementSibling;
    if (submenu && submenu.classList.contains('submenu')) {
      submenu.style.display = submenu.style.display === 'block' ? 'none' : 'block';

      // 切换三角形按钮的方向
      this.textContent = submenu.style.display === 'block' ? '▼' : '▶';
    }
  });
});
