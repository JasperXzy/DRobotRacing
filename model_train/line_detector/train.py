#!/usr/bin/env python3
"""Prepare, train, and export the ResNet18 line detector.

Label format: one row per image, ``x y valid`` in the 640x224 crop coordinate
system. The model predicts ``x_norm, y_norm, valid_logit``. Runtime code can
recover crop pixels with the original demo formula:

  x = (x_norm * 112 + 112) * 640 / 224
  y = 224 - (y_norm * 112 + 112)
  confidence = sigmoid(valid_logit)
"""

from __future__ import annotations

import argparse
import inspect
import json
import math
import random
import shutil
import time
from dataclasses import dataclass
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DATASET_ROOT = Path("/home/jasperxzy/Datasets/D_Robot/Line_Detector/v1")
DEFAULT_PROJECT = SCRIPT_DIR / "runs"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
IMAGENET_MEAN = [0.485, 0.456, 0.406]
IMAGENET_STD = [0.229, 0.224, 0.225]


@dataclass(frozen=True)
class LabeledImage:
    image: Path
    label: Path


@dataclass(frozen=True)
class Label:
    x: float
    y: float
    valid: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--project", type=Path, default=DEFAULT_PROJECT)
    parser.add_argument("--name", default="resnet18_v1")
    parser.add_argument("--weights", type=Path, default=None)
    parser.add_argument("--export-output", type=Path, default=None)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--batch", type=int, default=32)
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--train-ratio", type=float, default=0.8)
    parser.add_argument("--input-size", type=int, default=224)
    parser.add_argument(
        "--opset",
        type=int,
        default=11,
        help="ONNX opset for export. OpenExplorer ResNet18 samples use opset 11.",
    )
    parser.add_argument("--label-source-width", type=float, default=640.0)
    parser.add_argument("--label-source-height", type=float, default=224.0)
    parser.add_argument("--confidence-loss-weight", type=float, default=1.0)
    parser.add_argument("--coord-loss-weight", type=float, default=1.0)
    parser.add_argument("--color-jitter", type=float, default=0.3)
    parser.add_argument("--random-hflips", action="store_true")
    parser.add_argument(
        "--no-pretrained",
        action="store_true",
        help="Train ResNet18 from scratch instead of torchvision ImageNet weights.",
    )
    parser.add_argument(
        "--link-method",
        choices=("symlink", "copy"),
        default="symlink",
        help="How flat source files are materialized under images/train,val and labels/train,val.",
    )
    parser.add_argument(
        "--force-resplit",
        action="store_true",
        help="Recreate images/train,val and labels/train,val from flat images/ and labels/ files.",
    )
    parser.add_argument("--prepare", action="store_true", help="Create or validate train/val split metadata.")
    parser.add_argument("--train", action="store_true", help="Train the ResNet18 line detector.")
    parser.add_argument("--export", action="store_true", help="Export best.pt, or --weights, to BPU-friendly ONNX.")
    parser.add_argument("--all", action="store_true", help="Run --prepare, --train, and --export.")
    return parser.parse_args()


def split_flags(args: argparse.Namespace) -> tuple[bool, bool, bool]:
    if args.all or not (args.prepare or args.train or args.export):
        return True, True, True
    return args.prepare, args.train, args.export


def set_seed(seed: int) -> None:
    random.seed(seed)
    try:
        import numpy as np
        import torch

        np.random.seed(seed)
        torch.manual_seed(seed)
        if torch.cuda.is_available():
            torch.cuda.manual_seed_all(seed)
    except ImportError:
        pass


def list_images(directory: Path) -> list[Path]:
    if not directory.is_dir():
        return []
    return sorted(
        path
        for path in directory.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )


def clean_split_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)
    for child in path.iterdir():
        if child.is_symlink() or child.is_file():
            child.unlink()
        elif child.is_dir():
            raise RuntimeError(f"Refusing to remove nested directory in generated split: {child}")


def materialize_file(src: Path, dst: Path, method: str) -> None:
    if method == "symlink":
        dst.symlink_to(src)
    elif method == "copy":
        shutil.copy2(src, dst)
    else:
        raise ValueError(f"Unsupported link method: {method}")


def read_label(label_path: Path) -> Label:
    content = label_path.read_text(encoding="utf-8").strip()
    parts = content.split()
    if len(parts) != 3:
        raise ValueError(f"{label_path} must contain exactly 3 fields: x y valid")

    try:
        x = float(parts[0])
        y = float(parts[1])
        valid = float(parts[2])
    except ValueError as exc:
        raise ValueError(f"{label_path} has non-numeric label fields: {content}") from exc

    if not math.isfinite(valid) or not (
        math.isclose(valid, 0.0, abs_tol=1e-6) or math.isclose(valid, 1.0, abs_tol=1e-6)
    ):
        raise ValueError(f"{label_path} valid flag must be 0 or 1, got {valid}")
    valid = 1.0 if valid > 0.5 else 0.0

    if valid:
        if not math.isfinite(x) or not math.isfinite(y):
            raise ValueError(f"{label_path} positive sample has non-finite coordinates: {content}")
    else:
        x = 0.0
        y = 0.0

    return Label(x=x, y=y, valid=valid)


def validate_label(
    label: Label,
    label_path: Path,
    label_source_width: float,
    label_source_height: float,
) -> None:
    if not label.valid:
        return
    if not 0.0 <= label.x < label_source_width:
        raise ValueError(f"{label_path} x={label.x} is outside [0, {label_source_width})")
    if not 0.0 <= label.y < label_source_height:
        raise ValueError(f"{label_path} y={label.y} is outside [0, {label_source_height})")


def split_pairs(dataset_root: Path, split: str) -> list[LabeledImage]:
    image_dir = dataset_root / "images" / split
    label_dir = dataset_root / "labels" / split
    images = list_images(image_dir)
    labels = sorted(label_dir.glob("*.txt")) if label_dir.is_dir() else []
    labels_by_stem = {label.stem: label for label in labels}

    pairs: list[LabeledImage] = []
    missing: list[Path] = []
    for image in images:
        label = labels_by_stem.get(image.stem)
        if label is None:
            missing.append(image)
        else:
            pairs.append(LabeledImage(image=image, label=label))

    image_stems = {image.stem for image in images}
    orphan_labels = [label for label in labels if label.stem not in image_stems]
    if missing or orphan_labels:
        lines = []
        if missing:
            lines.append("images without labels:")
            lines.extend(f"  {path}" for path in missing[:20])
        if orphan_labels:
            lines.append("labels without images:")
            lines.extend(f"  {path}" for path in orphan_labels[:20])
        raise RuntimeError("\n".join(lines))

    return pairs


def flat_pairs(dataset_root: Path) -> list[LabeledImage]:
    images_dir = dataset_root / "images"
    labels_dir = dataset_root / "labels"
    images = list_images(images_dir)
    pairs: list[LabeledImage] = []
    missing: list[Path] = []
    for image in images:
        label = labels_dir / f"{image.stem}.txt"
        if label.exists():
            pairs.append(LabeledImage(image=image, label=label))
        else:
            missing.append(image)

    if missing:
        formatted = "\n".join(str(path) for path in missing[:20])
        raise RuntimeError(f"Found flat image files without labels:\n{formatted}")
    return pairs


def summarize_pairs(
    pairs: list[LabeledImage],
    args: argparse.Namespace,
) -> dict[str, int]:
    positives = 0
    negatives = 0
    for pair in pairs:
        label = read_label(pair.label)
        validate_label(label, pair.label, args.label_source_width, args.label_source_height)
        positives += int(label.valid == 1.0)
        negatives += int(label.valid == 0.0)
    return {"images": len(pairs), "positive": positives, "negative": negatives}


def write_dataset_manifest(args: argparse.Namespace, train_pairs: list[LabeledImage], val_pairs: list[LabeledImage]) -> Path:
    dataset_root = args.dataset_root.resolve()
    manifest_path = dataset_root / "line_detector.json"
    payload = {
        "path": str(dataset_root),
        "train": "images/train",
        "val": "images/val",
        "label_format": "x y valid",
        "model_output": "x_norm y_norm valid_logit",
        "input_size": [args.input_size, args.input_size],
        "label_source_size": [args.label_source_width, args.label_source_height],
        "normalization": {
            "x_norm": "2 * x / label_source_width - 1",
            "y_norm": "1 - 2 * y / label_source_height",
            "valid_logit": "sigmoid(logit) is line confidence",
        },
        "splits": {
            "train": summarize_pairs(train_pairs, args),
            "val": summarize_pairs(val_pairs, args),
        },
    }
    manifest_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    return manifest_path


def existing_split_ready(dataset_root: Path) -> bool:
    return bool(split_pairs(dataset_root, "train")) and bool(split_pairs(dataset_root, "val"))


def prepare_split(args: argparse.Namespace) -> Path:
    dataset_root = args.dataset_root.resolve()
    images_dir = dataset_root / "images"
    labels_dir = dataset_root / "labels"
    if not images_dir.is_dir() or not labels_dir.is_dir():
        raise FileNotFoundError(f"Expected images/ and labels/ under {dataset_root}")

    if not args.force_resplit and existing_split_ready(dataset_root):
        train_pairs = split_pairs(dataset_root, "train")
        val_pairs = split_pairs(dataset_root, "val")
    else:
        train_ratio = args.train_ratio
        if not 0.0 < train_ratio < 1.0:
            raise ValueError("--train-ratio must be between 0 and 1")

        pairs = flat_pairs(dataset_root)
        if not pairs:
            raise RuntimeError(
                f"No flat image/label pairs found under {dataset_root}. "
                "Existing split directories can be validated without --force-resplit."
            )

        rng = random.Random(args.seed)
        rng.shuffle(pairs)
        train_count = round(len(pairs) * train_ratio)
        train_pairs = pairs[:train_count]
        val_pairs = pairs[train_count:]

        for split_dir in (
            images_dir / "train",
            images_dir / "val",
            labels_dir / "train",
            labels_dir / "val",
        ):
            clean_split_dir(split_dir)

        for split, selected in (("train", train_pairs), ("val", val_pairs)):
            for pair in selected:
                materialize_file(pair.image, images_dir / split / pair.image.name, args.link_method)
                materialize_file(pair.label, labels_dir / split / pair.label.name, args.link_method)

        train_pairs = split_pairs(dataset_root, "train")
        val_pairs = split_pairs(dataset_root, "val")

    manifest_path = write_dataset_manifest(args, train_pairs, val_pairs)
    train_summary = summarize_pairs(train_pairs, args)
    val_summary = summarize_pairs(val_pairs, args)
    print(f"dataset: {dataset_root}")
    print(f"manifest: {manifest_path}")
    print(f"train: {train_summary}")
    print(f"val: {val_summary}")
    return manifest_path


def normalize_xy(label: Label, label_source_width: float, label_source_height: float) -> tuple[float, float]:
    if not label.valid:
        return 0.0, 0.0
    x_norm = 2.0 * label.x / label_source_width - 1.0
    y_norm = 1.0 - 2.0 * label.y / label_source_height
    return x_norm, y_norm


def denormalize_xy(
    x_norm: float,
    y_norm: float,
    label_source_width: float,
    label_source_height: float,
) -> tuple[float, float]:
    x = (x_norm + 1.0) * 0.5 * label_source_width
    y = (1.0 - y_norm) * 0.5 * label_source_height
    return x, y


class LineDetectorDataset:
    def __init__(
        self,
        dataset_root: Path,
        split: str,
        input_size: int,
        label_source_width: float,
        label_source_height: float,
        random_hflips: bool,
        color_jitter: float,
    ) -> None:
        import torch
        import torchvision.transforms as transforms

        self.torch = torch
        self.transforms = transforms
        self.dataset_root = dataset_root
        self.split = split
        self.input_size = input_size
        self.label_source_width = label_source_width
        self.label_source_height = label_source_height
        self.random_hflips = random_hflips
        self.pairs = split_pairs(dataset_root, split)
        if not self.pairs:
            raise RuntimeError(f"No samples found in split '{split}' under {dataset_root}")
        self.color_jitter = (
            transforms.ColorJitter(color_jitter, color_jitter, color_jitter, color_jitter)
            if color_jitter > 0.0
            else None
        )

    def __len__(self) -> int:
        return len(self.pairs)

    def __getitem__(self, idx: int):
        import PIL.Image
        import torchvision.transforms.functional as F

        pair = self.pairs[idx]
        image = PIL.Image.open(pair.image).convert("RGB")
        label = read_label(pair.label)
        validate_label(label, pair.label, self.label_source_width, self.label_source_height)
        x_norm, y_norm = normalize_xy(label, self.label_source_width, self.label_source_height)

        if self.random_hflips and label.valid and random.random() > 0.5:
            image = F.hflip(image)
            x_norm = -x_norm

        if self.color_jitter is not None:
            image = self.color_jitter(image)
        image = F.resize(image, (self.input_size, self.input_size))
        image = F.to_tensor(image)
        image = F.normalize(image, IMAGENET_MEAN, IMAGENET_STD)

        target_xy = self.torch.tensor([x_norm, y_norm], dtype=self.torch.float32)
        target_valid = self.torch.tensor([label.valid], dtype=self.torch.float32)
        return image, target_xy, target_valid


def build_model(pretrained: bool):
    import torch
    import torchvision.models as models

    try:
        from torchvision.models import ResNet18_Weights

        weights = ResNet18_Weights.DEFAULT if pretrained else None
        model = models.resnet18(weights=weights)
    except (ImportError, TypeError):
        model = models.resnet18(pretrained=pretrained)
    model.fc = torch.nn.Linear(model.fc.in_features, 3)
    return model


def resolve_device(device_arg: str):
    import torch

    if device_arg == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(device_arg)


def run_dir(args: argparse.Namespace) -> Path:
    return args.project.resolve() / args.name


def best_weights_path(args: argparse.Namespace) -> Path:
    return run_dir(args) / "weights" / "best.pt"


def load_state_dict(path: Path):
    import torch

    try:
        return torch.load(path, map_location="cpu", weights_only=True)
    except TypeError:
        return torch.load(path, map_location="cpu")


def maybe_load_weights(model, weights: Path | None) -> None:
    if weights is None:
        return
    weights = weights.expanduser().resolve()
    if not weights.is_file():
        raise FileNotFoundError(f"Weights not found: {weights}")
    state = load_state_dict(weights)
    if isinstance(state, dict) and "model" in state:
        state = state["model"]
    model.load_state_dict(state)
    print(f"loaded weights: {weights}")


def train_one_epoch(model, loader, optimizer, device, args: argparse.Namespace) -> dict[str, float]:
    import torch
    import torch.nn as nn

    coord_loss_fn = nn.MSELoss()
    conf_loss_fn = nn.BCEWithLogitsLoss()

    model.train()
    totals = {"loss": 0.0, "coord_loss": 0.0, "conf_loss": 0.0}
    batches = 0
    for images, target_xy, target_valid in loader:
        images = images.to(device)
        target_xy = target_xy.to(device)
        target_valid = target_valid.to(device).squeeze(1)

        optimizer.zero_grad()
        outputs = model(images)
        pred_xy = outputs[:, :2]
        pred_valid_logit = outputs[:, 2]

        valid_mask = target_valid == 1.0
        coord_loss = torch.tensor(0.0, device=device)
        if valid_mask.any():
            coord_loss = coord_loss_fn(pred_xy[valid_mask], target_xy[valid_mask])
        conf_loss = conf_loss_fn(pred_valid_logit, target_valid)
        loss = args.coord_loss_weight * coord_loss + args.confidence_loss_weight * conf_loss

        loss.backward()
        optimizer.step()

        totals["loss"] += float(loss.detach().cpu())
        totals["coord_loss"] += float(coord_loss.detach().cpu())
        totals["conf_loss"] += float(conf_loss.detach().cpu())
        batches += 1

    return {key: value / max(batches, 1) for key, value in totals.items()}


def evaluate(model, loader, device, args: argparse.Namespace) -> dict[str, float]:
    import torch
    import torch.nn as nn

    coord_loss_fn = nn.MSELoss()
    conf_loss_fn = nn.BCEWithLogitsLoss()
    model.eval()

    totals = {
        "loss": 0.0,
        "coord_loss": 0.0,
        "conf_loss": 0.0,
        "valid_accuracy": 0.0,
        "coord_mae_px": 0.0,
    }
    batches = 0
    valid_points = 0
    correct_valid = 0
    total_valid_flags = 0

    with torch.no_grad():
        for images, target_xy, target_valid in loader:
            images = images.to(device)
            target_xy = target_xy.to(device)
            target_valid = target_valid.to(device).squeeze(1)

            outputs = model(images)
            pred_xy = outputs[:, :2]
            pred_valid_logit = outputs[:, 2]

            valid_mask = target_valid == 1.0
            coord_loss = torch.tensor(0.0, device=device)
            if valid_mask.any():
                coord_loss = coord_loss_fn(pred_xy[valid_mask], target_xy[valid_mask])
            conf_loss = conf_loss_fn(pred_valid_logit, target_valid)
            loss = args.coord_loss_weight * coord_loss + args.confidence_loss_weight * conf_loss

            pred_valid = (torch.sigmoid(pred_valid_logit) >= 0.5).float()
            correct_valid += int((pred_valid == target_valid).sum().cpu())
            total_valid_flags += int(target_valid.numel())

            if valid_mask.any():
                pred_valid_xy = pred_xy[valid_mask].detach().cpu().tolist()
                target_valid_xy = target_xy[valid_mask].detach().cpu().tolist()
                for pred, target in zip(pred_valid_xy, target_valid_xy):
                    pred_x, pred_y = denormalize_xy(
                        pred[0], pred[1], args.label_source_width, args.label_source_height
                    )
                    target_x, target_y = denormalize_xy(
                        target[0], target[1], args.label_source_width, args.label_source_height
                    )
                    totals["coord_mae_px"] += (abs(pred_x - target_x) + abs(pred_y - target_y)) * 0.5
                    valid_points += 1

            totals["loss"] += float(loss.cpu())
            totals["coord_loss"] += float(coord_loss.cpu())
            totals["conf_loss"] += float(conf_loss.cpu())
            batches += 1

    averaged = {key: value / max(batches, 1) for key, value in totals.items() if key != "coord_mae_px"}
    averaged["valid_accuracy"] = correct_valid / max(total_valid_flags, 1)
    averaged["coord_mae_px"] = totals["coord_mae_px"] / max(valid_points, 1)
    return averaged


def train_model(args: argparse.Namespace) -> Path:
    import torch
    import torch.optim as optim

    device = resolve_device(args.device)
    run_path = run_dir(args)
    weights_dir = run_path / "weights"
    weights_dir.mkdir(parents=True, exist_ok=True)

    train_dataset = LineDetectorDataset(
        args.dataset_root.resolve(),
        "train",
        args.input_size,
        args.label_source_width,
        args.label_source_height,
        random_hflips=args.random_hflips,
        color_jitter=args.color_jitter,
    )
    val_dataset = LineDetectorDataset(
        args.dataset_root.resolve(),
        "val",
        args.input_size,
        args.label_source_width,
        args.label_source_height,
        random_hflips=False,
        color_jitter=0.0,
    )
    train_loader = torch.utils.data.DataLoader(
        train_dataset,
        batch_size=args.batch,
        shuffle=True,
        num_workers=args.workers,
        pin_memory=device.type == "cuda",
    )
    val_loader = torch.utils.data.DataLoader(
        val_dataset,
        batch_size=args.batch,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=device.type == "cuda",
    )

    model = build_model(pretrained=not args.no_pretrained)
    maybe_load_weights(model, args.weights)
    model = model.to(device)
    optimizer = optim.Adam(model.parameters(), lr=args.lr)

    best_loss = float("inf")
    best_path = weights_dir / "best.pt"
    last_path = weights_dir / "last.pt"
    history: list[dict[str, float | int]] = []

    print(f"device: {device}")
    print(f"run: {run_path}")
    print(f"train samples: {len(train_dataset)} val samples: {len(val_dataset)}")

    for epoch in range(1, args.epochs + 1):
        start = time.time()
        train_metrics = train_one_epoch(model, train_loader, optimizer, device, args)
        val_metrics = evaluate(model, val_loader, device, args)
        elapsed = time.time() - start

        torch.save(model.state_dict(), last_path)
        row = {
            "epoch": epoch,
            "elapsed_sec": elapsed,
            **{f"train_{key}": value for key, value in train_metrics.items()},
            **{f"val_{key}": value for key, value in val_metrics.items()},
        }
        history.append(row)
        (run_path / "results.json").write_text(json.dumps(history, indent=2), encoding="utf-8")

        print(
            f"epoch {epoch:03d}/{args.epochs} "
            f"train_loss={train_metrics['loss']:.6f} "
            f"val_loss={val_metrics['loss']:.6f} "
            f"val_coord_mae_px={val_metrics['coord_mae_px']:.3f} "
            f"val_valid_acc={val_metrics['valid_accuracy']:.4f} "
            f"time={elapsed:.1f}s"
        )
        if val_metrics["loss"] < best_loss:
            best_loss = val_metrics["loss"]
            torch.save(model.state_dict(), best_path)
            print(f"saved best: {best_path}")

    return best_path


def default_export_output(weights_path: Path) -> Path:
    return weights_path.with_name(f"{weights_path.stem}_bpu.onnx")


def export_onnx(args: argparse.Namespace, trained_weights: Path | None) -> Path:
    import torch

    weights_path = trained_weights or args.weights or best_weights_path(args)
    weights_path = weights_path.expanduser().resolve()
    if not weights_path.is_file():
        raise FileNotFoundError(f"Weights not found: {weights_path}")

    output_path = (args.export_output or default_export_output(weights_path)).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    model = build_model(pretrained=False)
    maybe_load_weights(model, weights_path)
    model.eval()
    dummy = torch.randn(1, 3, args.input_size, args.input_size)
    export_kwargs = {
        "export_params": True,
        "opset_version": args.opset,
        "do_constant_folding": True,
        "input_names": ["input"],
        "output_names": ["output"],
        "dynamic_axes": None,
    }
    if "dynamo" in inspect.signature(torch.onnx.export).parameters:
        # PyTorch 2.9+ defaults to the new exporter, which emits opset 18 first
        # and then tries to downgrade. Horizon/OpenExplorer samples expect a
        # direct static opset 11 export, so keep the legacy exporter here.
        export_kwargs["dynamo"] = False

    torch.onnx.export(model, dummy, str(output_path), **export_kwargs)
    print(f"bpu onnx: {output_path}")
    return output_path


def main() -> None:
    args = parse_args()
    set_seed(args.seed)
    do_prepare, do_train, do_export = split_flags(args)

    if do_prepare:
        prepare_split(args)

    best_weights: Path | None = None
    if do_train:
        best_weights = train_model(args)

    if do_export:
        export_onnx(args, best_weights)


if __name__ == "__main__":
    main()
