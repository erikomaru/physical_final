#!/usr/bin/env python3
"""Convert /odom + /cmd_vel rosbag2 episodes to Pai/LeRobot vector features."""
import argparse
import importlib.util
from pathlib import Path

import numpy as np
import torch
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


def load_config(path):
    spec = importlib.util.spec_from_file_location("rover_features", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def collect(bag_path, goal_xy):
    found = {"/model/rover/odometry": [], "/model/rover/cmd_vel": [],
             "/model/rover/key_counts": []}
    with Reader(bag_path) as reader:
        conns = [c for c in reader.connections if c.topic in found]
        for conn, stamp, raw in reader.messages(connections=conns):
            msg = deserialize_cdr(raw, conn.msgtype)
            if conn.topic == "/model/rover/key_counts":
                found[conn.topic].append((stamp, list(msg.data)))
            elif conn.topic.endswith("odometry"):
                p, q = msg.pose.pose.position, msg.pose.pose.orientation
                yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y),
                                 1 - 2 * (q.y * q.y + q.z * q.z))
                state = [p.x, p.y, np.sin(yaw), np.cos(yaw)]
                env = [goal_xy[0] - p.x, goal_xy[1] - p.y]
                found[conn.topic].append((stamp, state, env))
            else:
                found[conn.topic].append((stamp, [msg.linear.x, msg.angular.z]))
    return found


def hold(samples, times, field):
    stamps = np.asarray([s[0] for s in samples], dtype=np.int64)
    values = np.asarray([s[field] for s in samples], dtype=np.float32)
    indices = np.maximum(0, np.searchsorted(stamps, times, side="right") - 1)
    return values[indices]


def split_time_ranges(*sample_groups, max_gap_ns):
    """Return continuous timestamp intervals so clock jumps aren't interpolated."""
    stamps = np.unique(np.concatenate([
        np.asarray([sample[0] for sample in group], dtype=np.int64)
        for group in sample_groups if group
    ]))
    if not len(stamps):
        return []
    cuts = np.flatnonzero(np.diff(stamps) > max_gap_ns) + 1
    return [(int(part[0]), int(part[-1])) for part in np.split(stamps, cuts) if len(part)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag-dir", required=True, type=Path)
    ap.add_argument("--output-dir", default="/home/ubuntu/dataset/lerobot_dataset/single_o_to_rydges_osm", type=Path)
    ap.add_argument("--features", default="configs/rover_features.py")
    ap.add_argument("--fps", type=int, default=20)
    ap.add_argument("--max-gap-seconds", type=float, default=1.0,
                    help="Split a rosbag into episodes when timestamps jump by more than this")
    args = ap.parse_args()
    cfg = load_config(args.features)
    bags = sorted({p.parent for p in args.bag_dir.rglob("*.db3")})
    if not bags:
        raise SystemExit(f"No rosbag2 .db3 files found under {args.bag_dir}")
    if args.output_dir.exists():
        raise SystemExit(
            f"Output already exists: {args.output_dir}. If an earlier conversion was interrupted, "
            "leave it in place and retry with a new --output-dir."
        )
    print(f"Converting {len(bags)} bag(s) into {args.output_dir}", flush=True)
    ds = LeRobotDataset.create(repo_id=cfg.repo_id, root=args.output_dir, fps=args.fps,
                               robot_type=cfg.robot_type, features=cfg.features, use_videos=False)
    ds.meta.add_task(cfg.task_name)
    step_ns = int(1e9 / args.fps)
    for bag in bags:
        print(f"Reading {bag}", flush=True)
        topics = collect(bag, cfg.goal_xy)
        obs, acts = topics["/model/rover/odometry"], topics["/model/rover/cmd_vel"]
        counts = topics["/model/rover/key_counts"]
        if counts:
            up, down, left, right = counts[-1][1]
            print(f"Key presses {bag.name}: up={up}, down={down}, left={left}, right={right}", flush=True)
        if len(obs) < 2 or not acts:
            print(f"Skip {bag}: missing odometry or commands", flush=True)
            continue
        ranges = split_time_ranges(obs, acts, max_gap_ns=int(args.max_gap_seconds * 1e9))
        saved = 0
        for segment, (lo, hi) in enumerate(ranges, 1):
            start, end = max(lo, obs[0][0], acts[0][0]), min(hi, obs[-1][0], acts[-1][0])
            times = np.arange(start, end, step_ns, dtype=np.int64)
            if len(times) < 2:
                continue
            states, envs = hold(obs, times, 1), hold(obs, times, 2)
            actions = hold(acts, times, 1)
            for i, (state, env, action) in enumerate(zip(states, envs, actions)):
                if i and i % 1000 == 0:
                    print(f"Writing {i}/{len(times)} frames from {bag.name} segment {segment}", flush=True)
                ds.add_frame({"observation.state": torch.tensor(state),
                              "observation.environment_state": torch.tensor(env),
                              "action": torch.tensor(action), "task": cfg.task_name})
            ds.save_episode()
            saved += 1
            print(f"Saved {len(times)} frames from {bag.name} segment {segment}", flush=True)
        if not saved:
            print(f"Skip {bag}: no continuous segment had at least two frames", flush=True)
        del topics, obs, acts, counts, ranges
    print(f"LeRobot dataset: {ds.root}")


if __name__ == "__main__":
    main()
