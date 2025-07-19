#!/usr/bin/env python3

"""
简单的Tkinter测试脚本，确认GUI迁移成功。
显示当前Python版本。
仅支持Python 3.11+。
"""

import tkinter as tk
from tkinter import ttk
import sys
import platform

# 检查Python版本
if sys.version_info < (3, 11):
    print("错误: 此程序需要Python 3.11或更高版本")
    print(f"当前版本: {sys.version}")
    sys.exit(1)

def main():
    # 初始化Tkinter
    root = tk.Tk()
    root.title("GUI迁移测试")
    root.geometry("400x300")
    
    # 创建标签
    label = ttk.Label(root, text="GUI迁移成功！\nTkinter替代Qt5工作正常。", font=('Arial', 12))
    label.pack(pady=20)
    
    # 显示Python版本
    version_text = f"Python版本: {platform.python_version()}\n系统: {platform.system()} {platform.release()}"
    version_label = ttk.Label(root, text=version_text)
    version_label.pack(pady=10)
    
    # 仅支持Python 3.11+的标签
    compat_label = ttk.Label(root, text="仅支持Python 3.11及以上版本", 
                            foreground="blue", font=('Arial', 10, 'italic'))
    compat_label.pack(pady=5)
    
    # 创建按钮
    button = ttk.Button(root, text="退出", command=root.quit)
    button.pack(pady=10)
    
    # 运行主循环
    root.mainloop()

if __name__ == "__main__":
    main() 