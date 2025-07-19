#!/usr/bin/env python3

"""
简单的Tkinter测试脚本，确认GUI迁移成功。
显示当前Python版本。
"""

import tkinter as tk
from tkinter import ttk
import sys
import platform

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
    
    # 创建按钮
    button = ttk.Button(root, text="退出", command=root.quit)
    button.pack(pady=10)
    
    # 运行主循环
    root.mainloop()

if __name__ == "__main__":
    main() 