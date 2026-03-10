#!/usr/bin/env python3

import argparse
import subprocess
import os
import sys
import shutil
import time

# 官方模型下载地址
DEFAULT_DOWNLOAD_DIR = "https://github.com/PX4/PX4-gazebo-models/archive/refs/heads/main.zip"

def download_models(model_store, overwrite=False):
    """准备模型资源库"""
    if not os.path.exists(model_store):
        os.makedirs(model_store)

    try:
        model_count = int(subprocess.check_output(f'find {model_store} -type f | wc -l', shell=True, text=True))
    except:
        model_count = 0

    if model_count > 0 and not overwrite:
        print(f"[*] 资源库已就绪: {model_store}")
        return

    print('[!] 正在下载官方模型包...')
    zip_path = os.path.join(model_store, 'resources.zip')
    os.system(f'curl -L -o {zip_path} {DEFAULT_DOWNLOAD_DIR}')
    shutil.unpack_archive(zip_path, model_store, 'zip')
    
    source_dir = os.path.join(model_store, 'PX4-gazebo-models-main')
    for folder in ['models', 'worlds']:
        src = os.path.join(source_dir, folder)
        dst = os.path.join(model_store, folder)
        if not os.path.exists(dst): os.makedirs(dst)
        os.system(f'cp -r {src}/* {dst}/ 2>/dev/null')
    
    os.system(f'rm {zip_path} && rm -rf {source_dir}')
    print('[+] 模型资源准备就绪。')

def main():
    parser = argparse.ArgumentParser(description='PX4 Gazebo 环境启动器')
    parser.add_argument('--world', help='世界名称', default="default")
    parser.add_argument('--model', help='无人机模型', default="x500")
    parser.add_argument('--source', help='优先级: px4 或 download', choices=['px4', 'download'], default="px4")
    parser.add_argument('--px4_path', help='PX4 源码路径', default="~/PX4-Autopilot")
    parser.add_argument('--no-drone', help='只启动环境，不加载飞机', action='store_true')
    args = parser.parse_args()

    # 1. 路径初始化
    px4_dir = os.path.abspath(os.path.expanduser(args.px4_path))
    model_store = os.path.expanduser("~/.simulation-gazebo")
    build_dir = os.path.join(px4_dir, "build/px4_sitl_default")
    px4_bin = os.path.join(build_dir, "bin/px4")

    # 2. 准备资源
    download_models(model_store)

    # 3. 构建环境变量 (这是手动添加飞机的基础)
    px4_models = os.path.join(px4_dir, "Tools/simulation/gz/models")
    px4_worlds = os.path.join(px4_dir, "Tools/simulation/gz/worlds")
    download_models_path = os.path.join(model_store, "models")
    download_worlds_path = os.path.join(model_store, "worlds")

    if args.source == 'px4':
        resource_path = [px4_models, px4_worlds, download_models_path, download_worlds_path]
    else:
        resource_path = [download_models_path, download_worlds_path, px4_models, px4_worlds]
    
    env = os.environ.copy()
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(resource_path)
    env["PX4_GZ_STANDALONE"] = "1" 

    # 4. 寻找世界文件
    world_file = ""
    for r_path in resource_path:
        if "worlds" in r_path:
            p = os.path.join(r_path, f"{args.world}.sdf")
            if os.path.exists(p):
                world_file = p
                break
    
    if not world_file: world_file = args.world

    # 5. 启动 Gazebo
    print(f"[*] 正在启动场景: {args.world}...")
    gz_process = subprocess.Popen(f"gz sim -r {world_file}", shell=True, env=env)

    if args.no_drone:
        print("[+] 纯环境模式已启动。现在你可以手动运行 PX4 命令来添加飞机了。")
        try:
            gz_process.wait()
        except KeyboardInterrupt:
            gz_process.terminate()
            os.system("pkill -9 gz")
        return

    # 6. 正常加载第一架飞机 (Instance 0)
    print(f"[*] 正在自动加载第一架无人机 (Instance 0)...")
    time.sleep(3)
    env["PX4_GZ_MODEL"] = args.model
    px4_cmd = [px4_bin, "-s", "etc/init.d-posix/rcS", "SYS_AUTOSTART=4001"]

    try:
        px4_process = subprocess.Popen(px4_cmd, env=env, cwd=build_dir)
        px4_process.wait()
    except KeyboardInterrupt:
        px4_process.terminate()
        gz_process.terminate()
        os.system("pkill -9 gz")

if __name__ == "__main__":
    main()
