#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
悬臂式掘进机沙盒代码验证脚本
静态检查所有C#源文件的语法和结构完整性
"""

import os
import re
import sys
from pathlib import Path
from collections import defaultdict

class CSharpCodeValidator:
    def __init__(self, root_dir):
        self.root_dir = Path(root_dir)
        self.errors = []
        self.warnings = []
        self.info = []
        self.files = []
        
    def find_all_cs_files(self):
        """查找所有C#源文件"""
        cs_files = list(self.root_dir.rglob("*.cs"))
        self.files = [f for f in cs_files if "obj" not in str(f) and "bin" not in str(f)]
        self.info.append(f"找到 {len(self.files)} 个C#源文件")
        return self.files
    
    def check_brackets(self, content, filepath):
        """检查括号匹配"""
        stack = []
        bracket_map = {')': '(', '}': '{', ']': '['}
        line_counts = []
        current_line = 1
        
        for i, char in enumerate(content):
            if char == '\n':
                current_line += 1
            if char in '({[':
                stack.append((char, current_line))
            elif char in ')}]':
                if not stack:
                    self.errors.append(f"{filepath}:{current_line} - 多余的闭合括号 '{char}'")
                    continue
                last_char, last_line = stack.pop()
                if last_char != bracket_map[char]:
                    self.errors.append(f"{filepath}:{current_line} - 括号不匹配: 期望 '{bracket_map[char]}' 但得到 '{char}' (对应第{last_line}行)")
        
        for char, line in stack:
            self.errors.append(f"{filepath}:{line} - 未闭合的括号 '{char}'")
    
    def check_namespace(self, content, filepath):
        """检查命名空间声明"""
        namespace_match = re.search(r'namespace\s+([\w\.]+)', content)
        if not namespace_match:
            if "AssemblyInfo" not in filepath.name:
                self.warnings.append(f"{filepath} - 缺少命名空间声明")
        else:
            ns = namespace_match.group(1)
            expected_prefix = "RoadheaderSandbox"
            if not ns.startswith(expected_prefix):
                self.warnings.append(f"{filepath} - 命名空间 '{ns}' 应以 '{expected_prefix}' 开头")
    
    def check_using_directives(self, content, filepath):
        """检查using指令和引用"""
        lines = content.split('\n')
        for i, line in enumerate(lines, 1):
            if 'using UnityEngine' in line and ';' not in line:
                self.errors.append(f"{filepath}:{i} - using语句缺少分号")
            
            if re.match(r'^\s*using\s+[\w\.]+;', line):
                # 检查可能的命名空间冲突
                if 'System.Math' in line and 'Mathd' in content:
                    self.warnings.append(f"{filepath}:{i} - 注意: 项目使用自定义Mathd而非System.Math")
    
    def check_class_definitions(self, content, filepath):
        """检查类定义完整性"""
        # 检查类声明
        class_patterns = re.finditer(r'(public|private|internal|protected)\s+(partial\s+)?(class|struct|interface|enum)\s+(\w+)', content)
        classes = []
        for match in class_patterns:
            classes.append(match.group(4))
        
        if not classes and "AssemblyInfo" not in filepath.name:
            self.warnings.append(f"{filepath} - 未找到类/结构定义")
        
        self.info.append(f"{filepath.name}: 定义了 {len(classes)} 个类型: {', '.join(classes)}")
    
    def check_method_braces(self, content, filepath):
        """检查方法体大括号"""
        lines = content.split('\n')
        for i, line in enumerate(lines, 1):
            # 检查方法声明后是否有大括号
            if re.match(r'^\s*(public|private|protected|internal)\s+[\w<>\[\]]+\s+\w+\s*\([^)]*\)\s*$', line):
                if i < len(lines) and '{' not in lines[i] and '{' not in line:
                    # 可能是接口或抽象方法，检查修饰符
                    if 'abstract' not in line and 'interface' not in content.split('\n')[max(0,i-10):i]:
                        self.warnings.append(f"{filepath}:{i} - 方法声明可能缺少大括号")
    
    def check_semicolons(self, content, filepath):
        """检查语句结束分号"""
        lines = content.split('\n')
        skip_patterns = [
            r'^\s*using\s',
            r'^\s*#',
            r'^\s*//',
            r'^\s*/\*',
            r'\*/\s*$',
            r'^\s*$',
            r'^\s*[{}]\s*$',
            r'^\s*(namespace|class|struct|interface|enum|if|else|for|foreach|while|do|switch|case|try|catch|finally)\b',
            r'^\s*\[.*\]\s*$',
            r'\([^)]*\)\s*$',
        ]
        
        for i, line in enumerate(lines, 1):
            stripped = line.strip()
            if not stripped:
                continue
            
            should_have_semicolon = True
            for pattern in skip_patterns:
                if re.match(pattern, stripped):
                    should_have_semicolon = False
                    break
            
            if should_have_semicolon and not stripped.endswith(';') and not stripped.endswith('{') and not stripped.endswith('}'):
                # 检查是否是跨多行的语句
                if i > 1 and '=' in lines[i-2] and not lines[i-2].strip().endswith(';'):
                    continue
                # 可能是方法调用或属性访问链
                if re.match(r'^\s*\w+\s*\.\s*\w+', stripped) and '(' in stripped:
                    continue
                self.warnings.append(f"{filepath}:{i} - 可能缺少分号: {stripped[:50]}")
    
    def check_string_constants(self, content, filepath):
        """检查字符串常量"""
        # 简单检查未闭合的引号
        in_string = False
        in_char = False
        escape = False
        current_line = 1
        
        for i, char in enumerate(content):
            if char == '\n':
                current_line += 1
                if in_string:
                    self.errors.append(f"{filepath}:{current_line-1} - 未闭合的字符串")
                    in_string = False
                continue
            
            if escape:
                escape = False
                continue
            
            if char == '\\':
                escape = True
                continue
            
            if char == '"' and not in_char:
                in_string = not in_string
            elif char == "'" and not in_string:
                in_char = not in_char
        
        if in_string:
            self.errors.append(f"{filepath}:{current_line} - 文件结束时字符串未闭合")
    
    def validate_file(self, filepath):
        """验证单个文件"""
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()
            
            self.check_namespace(content, filepath)
            self.check_using_directives(content, filepath)
            self.check_brackets(content, filepath)
            self.check_class_definitions(content, filepath)
            self.check_method_braces(content, filepath)
            self.check_string_constants(content, filepath)
            self.check_semicolons(content, filepath)
            
            return True
        except Exception as e:
            self.errors.append(f"{filepath} - 读取文件失败: {str(e)}")
            return False
    
    def validate_all(self):
        """验证所有文件"""
        files = self.find_all_cs_files()
        
        print("=" * 80)
        print("悬臂式掘进机 ATO 沙盒 - 代码验证报告")
        print("=" * 80)
        print()
        
        for filepath in sorted(files):
            print(f"验证: {filepath.relative_to(self.root_dir)}")
            self.validate_file(filepath)
            print()
        
        print("=" * 80)
        print("验证结果摘要")
        print("=" * 80)
        
        print(f"\n文件总数: {len(self.files)}")
        print(f"错误数: {len(self.errors)}")
        print(f"警告数: {len(self.warnings)}")
        print()
        
        if self.errors:
            print("错误列表:")
            for error in self.errors:
                print(f"  ❌ {error}")
            print()
        
        if self.warnings:
            print("警告列表:")
            for warning in self.warnings:
                print(f"  ⚠️  {warning}")
            print()
        
        if self.info:
            print("信息:")
            for info in self.info:
                print(f"  ℹ️  {info}")
            print()
        
        if not self.errors:
            print("✅ 所有文件语法结构验证通过!")
            return 0
        else:
            print(f"❌ 发现 {len(self.errors)} 个错误, {len(self.warnings)} 个警告")
            return 1

def main():
    root_dir = Path(__file__).parent
    validator = CSharpCodeValidator(root_dir)
    return validator.validate_all()

if __name__ == "__main__":
    sys.exit(main())
