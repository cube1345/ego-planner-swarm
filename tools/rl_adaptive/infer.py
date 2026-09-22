"""训练好的 Q 网络推理：给定状态，选出最优融合参数。

展示训练产出如何接入 fusion 节点，替换手工 auto_tune 的评分函数。

用法：
    python tools/rl_adaptive/infer.py --model q_model.pt --state-file sample_state.json

    # 或在 fusion 节点里内联调用（把 auto_tune 的 utility 换成 Q 网络输出）：
    #   q = q_net(state)  # (N_ACTIONS,)
    #   best_action = q.argmax()
    #   min_probability, min_hits, dual_bonus = action_to_params(best_action)
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import torch

from config import N_ACTIONS, N_STATE, action_to_params, normalize_state
from train_q import QNetwork


def load_model(path: str, device: str = "cpu") -> QNetwork:
    model = QNetwork(N_STATE, N_ACTIONS)
    model.load_state_dict(torch.load(path, map_location=device))
    model.eval()
    return model


def select_action(model: QNetwork, raw_state: dict, device: str = "cpu"):
    """给定原始状态字典，返回 (action_index, (min_probability, min_hits, dual_bonus), q_values)。"""
    s = normalize_state(raw_state)
    x = torch.from_numpy(s).float().unsqueeze(0).to(device)
    with torch.no_grad():
        q = model(x).squeeze(0).cpu().numpy()
    action = int(np.argmax(q))
    return action, action_to_params(action), q


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--model", required=True)
    p.add_argument("--state-file", required=True, help="JSON，包含 normalize_state 需要的 raw 字段")
    p.add_argument("--device", default="cpu")
    args = p.parse_args()

    model = load_model(args.model, args.device)
    raw = json.loads(open(args.state_file).read())
    action, params, q = select_action(model, raw, args.device)
    print(f"best action = {action}")
    print(f"params = min_probability={params[0]:.3f}, min_hits={params[1]}, dual_bonus={params[2]:.2f}")
    print(f"top-5 q values = {np.sort(q)[::-1][:5].round(4)}")


if __name__ == "__main__":
    main()
