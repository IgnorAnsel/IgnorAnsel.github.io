// script.js
document.querySelectorAll('.toggle-btn').forEach(button => {
    button.addEventListener('click', function () {
      const submenu = this.nextElementSibling;
      if (submenu && submenu.classList.contains('submenu')) {
        submenu.style.display = submenu.style.display === 'block' ? 'none' : 'block';
        this.textContent = submenu.style.display === 'block' ? '▼' : '▶';
      }
    });
  });
  