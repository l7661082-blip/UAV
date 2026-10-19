#!/bin/bash
# det_color_conda.sh
# Wrapper: 激活 conda "demo" 环境，使用其 Python（含 CUDA 版 PyTorch）运行 det_color.py
# 相比直接用系统 python3，可以让 YOLO 推理走 GPU 而不是 CPU

# 找到 conda 初始化脚本（兼容 miniconda3 / anaconda3）
CONDA_SH=""
for candidate in \
    "$HOME/miniconda3/etc/profile.d/conda.sh" \
    "$HOME/anaconda3/etc/profile.d/conda.sh" \
    "/opt/conda/etc/profile.d/conda.sh" \
    "/opt/miniconda3/etc/profile.d/conda.sh"; do
    if [ -f "$candidate" ]; then
        CONDA_SH="$candidate"
        break
    fi
done

if [ -z "$CONDA_SH" ]; then
    echo "[det_color_conda] WARNING: conda init script not found, falling back to system python3"
    exec python3 "$(dirname "$0")/det_color.py" "$@"
fi

source "$CONDA_SH"
conda activate demo

echo "[det_color_conda] Python: $(which python3)"
echo "[det_color_conda] PyTorch CUDA: $(python3 -c 'import torch; print(torch.cuda.is_available())' 2>/dev/null)"

exec python3 "$(dirname "$0")/det_color.py" "$@"
