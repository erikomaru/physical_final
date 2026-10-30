#!/usr/bin/env python3
"""Train Pai's LeRobot ACT policy on the converted mobile-base dataset."""
import argparse
import importlib.util
from pathlib import Path

import torch
import yaml
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.common.datasets.utils import dataset_to_policy_features
from lerobot.common.policies.act.configuration_act import ACTConfig
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.configs.types import FeatureType


def load_py(path):
    spec = importlib.util.spec_from_file_location("rover_features", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="configs/rover_train_config.yaml")
    ap.add_argument("--dataset-root", type=Path, help="Override the local dataset_root in the YAML config")
    ap.add_argument("--output-directory", type=Path, help="Override the checkpoint output directory")
    args = ap.parse_args()
    cfg = yaml.safe_load(Path(args.config).read_text())
    data_root = args.dataset_root or Path(cfg["dataset_root"])
    out = args.output_directory or Path(cfg["output_directory"])
    info_path = data_root / "meta" / "info.json"
    if not info_path.is_file():
        raise SystemExit(
            f"Local LeRobot metadata is missing: {info_path}\n"
            "The bag conversion may have been interrupted, or dataset_root points to the wrong directory. "
            "Finish conversion into this exact directory before training; no Hub login is needed for a local dataset."
        )
    feature_cfg = load_py(cfg["config_path"])
    features = dataset_to_policy_features(feature_cfg.features)
    output_features = {k: v for k, v in features.items() if v.type is FeatureType.ACTION}
    input_features = {k: v for k, v in features.items() if k not in output_features}
    metadata = LeRobotDatasetMetadata(cfg["repo_id"], root=data_root)
    dataset = LeRobotDataset(cfg["repo_id"], root=data_root,
                             delta_timestamps=cfg["delta_timestamps"])
    if len(dataset) < 1:
        raise SystemExit(f"Dataset is empty: {data_root}")
    policy = ACTPolicy(
        ACTConfig(input_features=input_features, output_features=output_features,
                  chunk_size=cfg["chunk_size"], n_action_steps=cfg["n_action_steps"]),
        dataset_stats=metadata.stats,
    ).to(torch.device(cfg["device"]))
    policy.train()
    loader = torch.utils.data.DataLoader(
        dataset, num_workers=cfg["num_workers"], batch_size=cfg["batch_size"],
        shuffle=True, drop_last=False,
    )
    optimizer = torch.optim.Adam(policy.parameters(), lr=cfg["learning_rate"])
    out.mkdir(parents=True, exist_ok=True)
    step = 0
    while step < cfg["training_steps"]:
        for batch in loader:
            batch = {k: (v.to(cfg["device"]) if isinstance(v, torch.Tensor) else v)
                     for k, v in batch.items()}
            batch["observation.state"] = batch["observation.state"][:, -1, :]
            batch["observation.environment_state"] = batch["observation.environment_state"][:, -1, :]
            loss, _ = policy.forward(batch)
            loss.backward()
            optimizer.step()
            optimizer.zero_grad()
            if step % cfg["log_freq"] == 0:
                print(f"step {step}: loss={loss.item():.4f}", flush=True)
            step += 1
            if step >= cfg["training_steps"]:
                break
    policy.save_pretrained(out)
    print(f"Saved ACT policy: {out}")


if __name__ == "__main__":
    main()
