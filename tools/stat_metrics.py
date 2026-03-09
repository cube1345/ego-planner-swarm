import os
import matplotlib.pyplot as plt
import json
import glob

os.makedirs('../output2', exist_ok=True)

def parse_log_for_time(log_path):
    times = []
    with open(log_path, 'r', encoding='utf-8') as f:
        for line in f:
            if '扩展耗时' in line or 'resize耗时' in line:
                try:
                    t = float(line.strip().split()[-1])
                    times.append(t)
                except:
                    continue
    return times

def parse_block_util(summary_path):
    # 假设summary中有块利用率统计，如 block_used/block_total
    used, total = 0, 0
    with open(summary_path, 'r', encoding='utf-8') as f:
        for line in f:
            if 'block_used' in line:
                used = int(line.strip().split('=')[-1])
            if 'block_total' in line:
                total = int(line.strip().split('=')[-1])
    return used, total

def parse_mem_usage(mem_path):
    with open(mem_path, 'r', encoding='utf-8') as f:
        for line in f:
            if '内存占用' in line:
                return float(line.split(':')[-1])
    return 0.0

def plot_compare(vals1, vals2, labels, title, out_path):
    fig, ax = plt.subplots(figsize=(6,4))
    bar_width = 0.35
    ax.bar([0,1], vals1, width=bar_width, label='output1')
    ax.bar([0+bar_width,1+bar_width], vals2, width=bar_width, label='output2')
    ax.set_xticks([0+bar_width/2, 1+bar_width/2])
    ax.set_xticklabels(labels)
    ax.set_title(title)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print(f'saved {out_path}')
    plt.close(fig)

def main():
    # 路径
    output1 = '../output1'
    output2 = '../output2'
    # 内存占用
    mem2 = parse_mem_usage(os.path.join(output2, 'mem_usage.txt')) if os.path.exists(os.path.join(output2, 'mem_usage.txt')) else 0.0
    if os.path.exists(os.path.join(output1, 'mem_usage.txt')):
        mem1 = parse_mem_usage(os.path.join(output1, 'mem_usage.txt'))
        plot_compare([mem1, mem2], [mem1, mem2], ["output1", "output2"], "内存占用对比", os.path.join(output2, "mem_usage_compare.png"))
    else:
        plot_compare([mem2], [mem2], ["output2"], "内存占用", os.path.join(output2, "mem_usage.png"))

    # 块利用率
    if os.path.exists(os.path.join(output1, 'compare_fusion_summary.txt')):
        used1, total1 = parse_block_util(os.path.join(output1, 'compare_fusion_summary.txt'))
        util1 = used1/total1 if total1>0 else 0
    else:
        util1 = None
    used2, total2 = parse_block_util(os.path.join(output2, 'compare_fusion_summary.txt'))
    util2 = used2/total2 if total2>0 else 0
    if util1 is not None:
        plot_compare([util1, util2], [util1, util2], ["output1", "output2"], "块利用率对比", os.path.join(output2, "block_util_compare.png"))
    else:
        plot_compare([util2], [util2], ["output2"], "块利用率", os.path.join(output2, "block_util.png"))

    # 扩展/融合耗时
    if os.path.exists(os.path.join(output1, 'run_depth_only.log')):
        ext_time1 = parse_log_for_time(os.path.join(output1, 'run_depth_only.log'))
        avg1 = sum(ext_time1)/len(ext_time1) if ext_time1 else 0
    else:
        avg1 = None
    ext_time2 = parse_log_for_time(os.path.join(output2, 'run_depth_only.log'))
    avg2 = sum(ext_time2)/len(ext_time2) if ext_time2 else 0
    if avg1 is not None:
        plot_compare([avg1, avg2], [avg1, avg2], ["output1", "output2"], "扩展耗时对比", os.path.join(output2, "expand_time_compare.png"))
    else:
        plot_compare([avg2], [avg2], ["output2"], "扩展耗时", os.path.join(output2, "expand_time.png"))

    # 系统稳定性（可用异常次数/崩溃次数等指标，暂略）

if __name__ == '__main__':
    main()