#!/usr/bin/env python3
"""
ADRC 阵风 / 外力注入工具（PX4 + gz-sim 8, ApplyLinkWrench）

用法（先启动你的 PX4 + Gazebo，再运行本脚本）：
  # 1) 自检：确认 wrench topic 和机体名称
  python3 gust_inject.py --check

  # 2) 阵风阶跃：-8N 侧风，保持 3 秒后撤掉
  python3 gust_inject.py --force 8,0,0 --wave step --duration 3

  # 3) 阵风脉冲：-8N 打 0.5 秒
  python3 gust_inject.py --force 8,0,0 --wave pulse --duration 0.5

  # 4) 正弦扫风 0.5->5 Hz（找 ADRC 补偿回路谐振点）
  python3 gust_inject.py --force 6,0,0 --wave sine --freq 0.5 --freq_end 5 --duration 40

  # 5) 挂载突变：z 向力突然 -5N（模拟挂载变重）
  python3 gust_inject.py --force 0,0,-5 --wave step --duration 10

  # 6) 重复阵风序列（每个周期 N 秒，做 M 次）
  python3 gust_inject.py --force 8,0,0 --wave step --duration 3 --interval 6 --reps 5

  # 7) 完整测试序列（CSV 每行：时刻s 力x 力y 力z 持续s）
  #    基线悬停 -> 阵风阶跃 -> 恢复 -> 脉冲 -> 持续侧风
  python3 gust_inject.py --seq seq_adrc_basic.csv

注意：wrench 是“持续作用直到发零力”，所以脚本会自己发零力收尾；
      力与 Gazebo 风会叠加，别同时开两个同方向的扰动。
"""

import argparse
import csv
import math
import subprocess
import sys
import time


def gz_topic_list():
    try:
        out = subprocess.run(["gz", "topic", "-l"], capture_output=True, text=True, timeout=8)
        return out.stdout.splitlines()
    except Exception:
        return []


def detect_model():
    """从 gz model --list 里自动找飞行器模型（跳过 ground_plane / payload_box 等）。"""
    try:
        out = subprocess.run(["gz", "model", "--list"], capture_output=True, text=True, timeout=8)
        lines = [ln.strip() for ln in out.stdout.splitlines() if ln.strip()]
        models = []
        for ln in lines:
            if ln.startswith("- "):
                name = ln[2:].strip()
                if name and name != "ground_plane":
                    models.append(name)
        if not models:
            return None
        # 优先找常见飞行器关键词；否则取第一个
        for kw in ("x500", "iris", "standard_vtol", "rc_cessna", "px4vision", "omnicopter"):
            for m in models:
                if kw in m:
                    return m
        return models[0]
    except Exception:
        return None


def send_wrench(world, entity_name, fx, fy, fz, entity_type="MODEL"):
    """通过 persistent topic 施加持续力。entity_type: MODEL 或 LINK"""
    if entity_type == "MODEL":
        entity = f'{{name: "{entity_name}", type: MODEL}}'
    else:
        entity = f'{{name: "{entity_name}", type: LINK}}'
    payload = (
        f'entity: {entity} '
        f'wrench: {{force: {{x: {fx:.4f}, y: {fy:.4f}, z: {fz:.4f}}}}}'
    )
    subprocess.run(
        ["gz", "topic", "-t", f"/world/{world}/wrench/persistent",
         "-m", "gz.msgs.EntityWrench", "-p", payload],
        check=False,
    )


def clear_wrench(world, entity_name, entity_type="MODEL"):
    """清除 persistent 力：clear topic 用 gz.msgs.Entity（不是 EntityWrench）"""
    # gz.msgs.Entity 是顶层消息，text-format 不能包大括号（官方示例：
    #   gz topic -t ".../wrench/clear" -m gz.msgs.Entity -p "name: 'box', type: MODEL"）
    entity = f'name: "{entity_name}", type: {entity_type}'
    subprocess.run(
        ["gz", "topic", "-t", f"/world/{world}/wrench/clear",
         "-m", "gz.msgs.Entity", "-p", entity],
        check=False,
    )


def check_env(args):
    print("[check] 正在查找 wrench topic ...")
    topics = gz_topic_list()
    candidates = [t for t in topics if "wrench" in t]
    if not candidates:
        print("[check] 没找到 wrench topic。请确认：")
        print("       1) Gazebo 已启动（python Tools/simulation/gz/simulation-gazebo --world %s）" % args.world)
        print("       2) world 名与 --world 一致")
        sys.exit(1)
    print("[check] 找到 topic:", candidates)
    # 尝试在候选 topic 里找名字匹配当前 world 的
    exact = [t for t in candidates if f"/world/{args.world}/wrench" == t]
    if exact:
        print("[check] 匹配 --world %s 的 topic: %s" % (args.world, exact[0]))
    else:
        print("[check] 注意：没有 /world/%s/wrench，可用的有：%s" % (args.world, candidates))
    entity = args.entity or detect_model()
    if entity and args.entity is None:
        print("[check] 自动检测到飞行器模型：%s（可用 --entity 覆盖）" % entity)
    if args.target == "link":
        ent = args.link or ("%s::base_link" % entity) if entity else (args.link or "x500::base_link")
    else:
        ent = entity or "x500"
    etype = "LINK" if args.target == "link" else "MODEL"
    print("[check] 实体：%s（type=%s）" % (ent, etype))
    print("[check] 自检完成。发一条 0 力测试消息：")
    send_wrench(args.world, ent, 0, 0, 0, etype)
    clear_wrench(args.world, ent, etype)
    print("[check] 已发送并清除（若 Gazebo 终端无报错即通路正常）")


def main():
    ap = argparse.ArgumentParser(description="ADRC 阵风/外力注入（gz-sim 8 ApplyLinkWrench）")
    ap.add_argument("--world", default="default", help="Gazebo world 名（默认 default）")
    ap.add_argument("--entity", default=None, help="模型名（默认自动检测，如 x500_0）")
    ap.add_argument("--link", default=None, help="link 名（--target link 时用，默认自动用 <模型>::base_link）")
    ap.add_argument("--target", choices=["model", "link"], default="model",
                    help="model=整个模型（质心，模拟风/挂载，默认）；link=单个 link")
    ap.add_argument("--force", default="8,0,0", help="力向量 x,y,z（N，ENU 坐标）")
    ap.add_argument("--wave", choices=["step", "pulse", "sine"], default="step",
                    help="波形：step=阶跃保持后撤；pulse=短脉冲；sine=正弦扫频")
    ap.add_argument("--duration", type=float, default=3.0, help="作用时长 s（step 保持多久 / pulse 脉冲宽 / sine 扫频总时长）")
    ap.add_argument("--freq", type=float, default=0.5, help="sine 起始频率 Hz")
    ap.add_argument("--freq_end", type=float, default=5.0, help="sine 结束频率 Hz（扫频时用）")
    ap.add_argument("--interval", type=float, default=0.0, help="重复周期 s（0=不重复）")
    ap.add_argument("--reps", type=int, default=1, help="重复次数")
    ap.add_argument("--seq", default=None, help="测试序列 CSV：每行 t,fx,fy,fz,duration（t 相对序列起点，持续>0 到时自动 clear）")
    ap.add_argument("--check", action="store_true", help="只做自检，不发扰动")
    args = ap.parse_args()

    fx, fy, fz = (float(v) for v in args.force.split(","))
    if not args.check and args.entity is None:
        args.entity = detect_model()
        if not args.entity:
            print("[error] 无法自动检测模型名，请用 --entity 指定（可用 gz model --list 查看）")
            sys.exit(1)
        print("[info] 自动检测到飞行器模型：%s" % args.entity)
    entity = args.link if args.target == "link" else args.entity
    if args.target == "link" and args.link is None:
        entity = "%s::base_link" % args.entity
    etype = "LINK" if args.target == "link" else "MODEL"

    if args.check:
        check_env(args)
        return

    if args.seq:
        events = []
        with open(args.seq, newline="") as f:
            for row in csv.reader(f):
                if not row or row[0].strip().startswith("#"):
                    continue
                if len(row) < 5:
                    print("[warn] 忽略无效行：%s" % row)
                    continue
                events.append((float(row[0]), float(row[1]), float(row[2]),
                               float(row[3]), float(row[4])))
        if not events:
            print("[error] 序列文件为空或格式错误：%s" % args.seq)
            sys.exit(1)
        t0 = time.monotonic()
        print("[seq] 开始序列，共 %d 个事件（相对时间轴）" % len(events))
        for i, (t_ev, ex, ey, ez, dur) in enumerate(events):
            wait = t0 + t_ev - time.monotonic()
            if wait > 0:
                time.sleep(wait)
            if ex == 0 and ey == 0 and ez == 0:
                clear_wrench(args.world, entity, etype)
                print("[seq] t=%6.2fs 事件%2d：clear" % (t_ev, i + 1))
                if dur > 0:
                    time.sleep(max(dur - (time.monotonic() - (t0 + t_ev)), 0))
                continue
            send_wrench(args.world, entity, ex, ey, ez, etype)
            print("[seq] t=%6.2fs 事件%2d：力=(%6.1f,%6.1f,%6.1f)N 持续%.1fs"
                  % (t_ev, i + 1, ex, ey, ez, dur))
            if dur > 0:
                next_t = events[i + 1][0] if i + 1 < len(events) else float("inf")
                if t_ev + dur < next_t - 1e-4:
                    time.sleep(max(t_ev + dur - (time.monotonic() - t0), 0))
                    clear_wrench(args.world, entity, etype)
                    print("[seq] t=%6.2fs clear（事件%2d结束）" % (t_ev + dur, i + 1))
        print("[seq] 序列完成")
        return

    topics = gz_topic_list()
    if not any("/world/%s/wrench" % args.world == t for t in topics):
        print("[warn] 没找到 /world/%s/wrench，先自检：python3 gust_inject.py --check" % args.world)
        print("       可用 topic:", [t for t in topics if "wrench" in t] or "(无)")

    t0 = time.monotonic()
    n = 0
    while n < args.reps:
        t_rel = time.monotonic() - t0
        print(f"[t={t_rel:6.2f}s] 开始第 {n + 1}/{args.reps} 次：{args.wave} 力=({fx},{fy},{fz})N 时长={args.duration}s")

        if args.wave == "sine":
            # 连续正弦扫频（频率线性从 freq 扫到 freq_end）
            dt = 0.02
            steps = int(args.duration / dt)
            for i in range(steps):
                f_now = args.freq + (args.freq_end - args.freq) * i / max(steps - 1, 1)
                a = fx * math.sin(2 * math.pi * f_now * i * dt)
                send_wrench(args.world, entity, a, fy, fz, etype)
                time.sleep(dt)
            clear_wrench(args.world, entity, etype)
        elif args.wave == "pulse":
            send_wrench(args.world, entity, fx, fy, fz, etype)
            time.sleep(args.duration)
            clear_wrench(args.world, entity, etype)
        else:  # step
            send_wrench(args.world, entity, fx, fy, fz, etype)
            time.sleep(args.duration)
            clear_wrench(args.world, entity, etype)

        n += 1
        if args.interval > 0 and n < args.reps:
            time.sleep(max(args.interval - args.duration, 0))

    t_end = time.monotonic() - t0
    print(f"[done] 完成 {n} 次注入，总耗时 {t_end:.1f}s，已发零力收尾")
    print("       对照日志时间轴：从 QGC/ulog 的起飞时刻起算，第 1 次注入发生在约 t=%ds" % 0)


if __name__ == "__main__":
    main()
