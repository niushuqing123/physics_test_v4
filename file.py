# traverse_tree.py
import os
from pathlib import Path

def walk_tree(root_path, max_depth=2):
    root_path = Path(root_path)
    if not root_path.exists():
        raise FileNotFoundError(f"{root_path} 不存在")

    root_depth = len(root_path.resolve().parts)
    for dirpath, dirnames, filenames in os.walk(root_path):
        current_depth = len(Path(dirpath).resolve().parts) - root_depth
        if current_depth > max_depth:
            # 深度超过5时，不再继续当前目录的子目录
            dirnames[:] = []
            continue

        indent = "  " * current_depth
        print(f"{indent}{Path(dirpath).name}/")
        for f in sorted(filenames):
            print(f"{indent}  {f}")

        # 如果当前深度已经max_depth，不继续更深层级
        if current_depth == max_depth:
            dirnames[:] = []

if __name__ == "__main__":
    # 修改为你要遍历的目录
    root = r"d:\project\physics_test"
    walk_tree(root, max_depth=3)