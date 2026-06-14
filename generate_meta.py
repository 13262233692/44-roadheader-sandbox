#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
自动生成Unity项目meta文件
"""

import os
import sys
import hashlib
from pathlib import Path

META_TEMPLATE = '''fileFormatVersion: 2
guid: {guid}
MonoImporter:
  externalObjects: {{}}
  serializedVersion: 2
  defaultReferences: []
  executionOrder: {order}
  icon: {{instanceID: 0}}
  userData: 
  assetBundleName: 
  assetBundleVariant: 
'''

FOLDER_META_TEMPLATE = '''fileFormatVersion: 2
guid: {guid}
folderAsset: yes
DefaultImporter:
  externalObjects: {{}}
  userData: 
  assetBundleName: 
  assetBundleVariant: 
'''

ASSET_META_TEMPLATE = '''fileFormatVersion: 2
guid: {guid}
{importer}:
  externalObjects: {{}}
  userData: 
  assetBundleName: 
  assetBundleVariant: 
'''

def generate_guid(path):
    """基于路径生成稳定的GUID"""
    hash_obj = hashlib.md5(str(path).encode('utf-8'))
    return hash_obj.hexdigest()

def get_execution_order(path):
    """根据文件路径设置执行顺序"""
    path_str = str(path)
    if 'Core/Math' in path_str:
        return 100
    elif 'Physics/' in path_str:
        return 200
    elif 'Kinematics/' in path_str:
        return 300
    elif 'Robotics/' in path_str:
        return 400
    elif 'Debug/' in path_str:
        return 500
    return 0

def create_meta_for_file(filepath):
    """为单个文件创建meta文件"""
    meta_path = Path(str(filepath) + '.meta')
    if meta_path.exists():
        return False
    
    guid = generate_guid(filepath)
    rel_path = filepath
    
    if filepath.suffix == '.cs':
        order = get_execution_order(rel_path)
        content = META_TEMPLATE.format(guid=guid, order=order)
    elif filepath.suffix == '.unity':
        content = ASSET_META_TEMPLATE.format(guid=guid, importer='DefaultImporter')
    else:
        content = ASSET_META_TEMPLATE.format(guid=guid, importer='DefaultImporter')
    
    with open(meta_path, 'w', encoding='utf-8') as f:
        f.write(content)
    return True

def create_meta_for_folder(folderpath):
    """为文件夹创建meta文件"""
    meta_path = Path(str(folderpath) + '.meta')
    if meta_path.exists():
        return False
    
    guid = generate_guid(folderpath)
    content = FOLDER_META_TEMPLATE.format(guid=guid)
    
    with open(meta_path, 'w', encoding='utf-8') as f:
        f.write(content)
    return True

def process_directory(root_dir):
    """递归处理目录"""
    root = Path(root_dir)
    count_files = 0
    count_folders = 0
    
    # 首先处理所有文件夹
    for dirpath, dirnames, filenames in os.walk(root):
        current_dir = Path(dirpath)
        if current_dir != root and 'obj' not in str(current_dir) and 'bin' not in str(current_dir):
            if create_meta_for_folder(current_dir):
                count_folders += 1
                print(f"创建文件夹meta: {current_dir.relative_to(root)}")
    
    # 然后处理所有文件
    for dirpath, dirnames, filenames in os.walk(root):
        current_dir = Path(dirpath)
        if 'obj' in str(current_dir) or 'bin' in str(current_dir):
            continue
            
        for filename in filenames:
            if filename.endswith('.meta'):
                continue
            filepath = current_dir / filename
            if create_meta_for_file(filepath):
                count_files += 1
                print(f"创建文件meta: {filepath.relative_to(root)}")
    
    print(f"\n完成! 共创建 {count_folders} 个文件夹meta, {count_files} 个文件meta")
    return count_files + count_folders

def main():
    root_dir = Path(__file__).parent / 'Assets'
    if not root_dir.exists():
        root_dir = Path(__file__).parent
    
    print("=" * 60)
    print("Unity Meta 文件自动生成器")
    print("=" * 60)
    print()
    
    count = process_directory(root_dir)
    
    # 处理ProjectSettings目录
    settings_dir = Path(__file__).parent / 'ProjectSettings'
    if settings_dir.exists():
        print("\n处理 ProjectSettings 目录...")
        for filepath in settings_dir.glob('*'):
            if filepath.is_file() and not filepath.name.endswith('.meta'):
                if create_meta_for_file(filepath):
                    count += 1
                    print(f"创建文件meta: ProjectSettings/{filepath.name}")
    
    print(f"\n总计创建 {count} 个meta文件")
    return 0

if __name__ == "__main__":
    sys.exit(main())
