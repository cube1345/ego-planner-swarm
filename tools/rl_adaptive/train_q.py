"""离线 Q 函数训练：拟合「状态+参数 → 避障质量」的评分网络。

用法（在训练平台，如 GPU 服务器）：
    python tools/rl_adaptive/train_q.py \
        --data /path/to/adaptive_data.csv \
        --out /path/to/model.pt \
        --epochs 50 --batch-size 256 --lr 3e-4

输入 CSV 由本地 ROS2 仿真通过 tools/rl_adaptive/collect_data.py 采集，
列名见 config.py 的 CSV_COLUMNS。

网络结构：MLP，输入 state（N_STATE 维），输出 Q(s, a)（N_ACTIONS 维）。
训练目标：MSE(Q(s)[a], r)，其中 r 是闭环奖励（见 config.compute_reward）。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# 允许从任意目录运行，import 同目录的 config.py
sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import torch
import torch.nn as nn

from config import N_ACTIONS, N_STATE, CSV_COLUMNS, normalize_state, compute_reward


class QNetwork(nn.Module):
    """Q(s, a) 评分网络：输入状态，输出每个动作的分数。"""

    def __init__(self, state_dim: int, action_dim: int, hidden: int = 256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, action_dim),
        )

    def forward(self, state: torch.Tensor) -> torch.Tensor:
        return self.net(state)


def load_dataset(csv_path: Path):
    """从 CSV 加载 (s, a, r) 样本，返回 numpy 数组。

    如果 CSV 里没有直接的 reward 列，则用 config.compute_reward 从指标列实时计算。
    """
    import csv

    states: list[np.ndarray] = []
    actions: list[int] = []
    rewards: list[float] = []

    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            # 状态
            s = normalize_state(row)
            # 动作
            a = int(row.get("action", -1))
            if a < 0 or a >= N_ACTIONS:
                continue
            # 奖励：优先用 CSV 里预存的 reward 列，否则实时算
            if "reward" in row and row["reward"] not in ("", None):
                r = float(row["reward"])
            else:
                r = compute_reward(row)
            states.append(s)
            actions.append(a)
            rewards.append(r)

    if not states:
        raise ValueError(f"数据集为空：{csv_path}")

    X = np.stack(states).astype(np.float32)
    A = np.asarray(actions, dtype=np.int64)
    R = np.asarray(rewards, dtype=np.float32)
    return X, A, R


def train(args):
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    device = torch.device(args.device if torch.cuda.is_available() else "cpu")

    X, A, R = load_dataset(Path(args.data))
    n = X.shape[0]
    print(f"[data] samples={n} state_dim={N_STATE} action_dim={N_ACTIONS}")

    # 打乱 + 划分 train/val
    idx = np.random.permutation(n)
    n_val = int(n * args.val_ratio)
    val_idx, train_idx = idx[:n_val], idx[n_val:]

    X_t = torch.from_numpy(X[train_idx]).to(device)
    A_t = torch.from_numpy(A[train_idx]).to(device)
    R_t = torch.from_numpy(R[train_idx]).to(device)
    X_v = torch.from_numpy(X[val_idx]).to(device)
    A_v = torch.from_numpy(A[val_idx]).to(device)
    R_v = torch.from_numpy(R[val_idx]).to(device)

    model = QNetwork(N_STATE, N_ACTIONS, args.hidden).to(device)
    opt = torch.optim.Adam(model.parameters(), lr=args.lr)
    loss_fn = nn.MSELoss()

    best_val = float("inf")
    for epoch in range(args.epochs):
        model.train()
        # 按 batch 训练
        perm = torch.randperm(train_idx.shape[0], device=device)
        for i in range(0, perm.shape[0], args.batch_size):
            bidx = perm[i : i + args.batch_size]
            q_all = model(X_t[bidx])            # (B, N_ACTIONS)
            q = q_all.gather(1, A_t[bidx].unsqueeze(1)).squeeze(1)  # (B,)
            loss = loss_fn(q, R_t[bidx])
            opt.zero_grad()
            loss.backward()
            opt.step()

        # 验证
        model.eval()
        with torch.no_grad():
            q_v = model(X_v).gather(1, A_v.unsqueeze(1)).squeeze(1)
            val_loss = loss_fn(q_v, R_v).item()

        if (epoch + 1) % max(1, args.log_every) == 0 or epoch == 0:
            print(f"[epoch {epoch+1}/{args.epochs}] train_loss={loss.item():.4f} val_loss={val_loss:.4f}")

        if val_loss < best_val:
            best_val = val_loss
            torch.save(model.state_dict(), args.out)

    print(f"[done] best val_loss={best_val:.4f}, model saved to {args.out}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data", required=True, help="collect_data.py 输出的 CSV")
    p.add_argument("--out", default="q_model.pt")
    p.add_argument("--epochs", type=int, default=50)
    p.add_argument("--batch-size", type=int, default=256)
    p.add_argument("--lr", type=float, default=3e-4)
    p.add_argument("--hidden", type=int, default=256)
    p.add_argument("--val-ratio", type=float, default=0.1)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--device", default="cuda")
    p.add_argument("--log-every", type=int, default=5)
    args = p.parse_args()
    train(args)


if __name__ == "__main__":
    main()
