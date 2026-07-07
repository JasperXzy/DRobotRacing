#!/usr/bin/env python3
"""Prepare, train, and export the YOLO26 perception detector."""

from __future__ import annotations

import argparse
import random
import shutil
from collections import Counter
from pathlib import Path


DEFAULT_DATASET_ROOT = Path("/home/jasperxzy/Datasets/D_Robot/Perception_Detector/v1")
DEFAULT_PROJECT = Path("/home/jasperxzy/Racing/D_Robot/model_train/perception_detector/runs")
DEFAULT_CLASS_NAMES = ["construction_cone", "parking_sign", "qr_code"]
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--project", type=Path, default=DEFAULT_PROJECT)
    parser.add_argument("--name", default="yolo26n_v1")
    parser.add_argument("--model", default="yolo26n.pt")
    parser.add_argument("--weights", type=Path, default=None)
    parser.add_argument("--export-output", type=Path, default=None)
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", default="16")
    parser.add_argument("--device", default="0")
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--train-ratio", type=float, default=0.75)
    parser.add_argument(
        "--link-method",
        choices=("symlink", "copy"),
        default="symlink",
        help="How split files are materialized under images/train,val and labels/train,val.",
    )
    parser.add_argument("--prepare", action="store_true", help="Create train/val split files and dataset YAML.")
    parser.add_argument("--train", action="store_true", help="Train YOLO26.")
    parser.add_argument("--export", action="store_true", help="Export best.pt, or --weights, to BPU-friendly ONNX.")
    parser.add_argument("--all", action="store_true", help="Run --prepare, --train, and --export.")
    return parser.parse_args()


def split_flags(args: argparse.Namespace) -> tuple[bool, bool, bool]:
    if args.all or not (args.prepare or args.train or args.export):
        return True, True, True
    return args.prepare, args.train, args.export


def list_flat_images(images_dir: Path) -> list[Path]:
    return sorted(
        path
        for path in images_dir.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )


def clean_split_dir(path: Path) -> None:
    path.mkdir(parents=True, exist_ok=True)
    for child in path.iterdir():
        if child.is_dir():
            raise RuntimeError(f"Refusing to remove nested directory in generated split: {child}")
        child.unlink()


def materialize_file(src: Path, dst: Path, method: str) -> None:
    if method == "symlink":
        dst.symlink_to(src)
    elif method == "copy":
        shutil.copy2(src, dst)
    else:
        raise ValueError(f"Unsupported link method: {method}")


def read_class_names(dataset_root: Path) -> list[str]:
    classes_path = dataset_root / "classes.txt"
    if not classes_path.exists():
        return DEFAULT_CLASS_NAMES
    names = [line.strip() for line in classes_path.read_text().splitlines() if line.strip()]
    return names or DEFAULT_CLASS_NAMES


def image_label_pairs(dataset_root: Path) -> list[tuple[Path, Path]]:
    images_dir = dataset_root / "images"
    labels_dir = dataset_root / "labels"
    images = list_flat_images(images_dir)
    pairs: list[tuple[Path, Path]] = []
    missing_labels: list[Path] = []

    for image_path in images:
        label_path = labels_dir / f"{image_path.stem}.txt"
        if label_path.exists():
            pairs.append((image_path, label_path))
        else:
            missing_labels.append(image_path)

    if missing_labels:
        formatted = "\n".join(str(path) for path in missing_labels[:20])
        raise RuntimeError(
            "Found image files without labels. Delete them before preparing the split:\n"
            f"{formatted}"
        )

    return pairs


def write_dataset_yaml(dataset_root: Path, names: list[str]) -> Path:
    yaml_path = dataset_root / "perception_detector.yaml"
    names_block = "\n".join(f"  {idx}: {name}" for idx, name in enumerate(names))
    yaml_path.write_text(
        "\n".join(
            [
                f"path: {dataset_root}",
                "train: images/train",
                "val: images/val",
                "",
                "names:",
                names_block,
                "",
            ]
        )
    )
    return yaml_path


def count_boxes(labels: list[Path]) -> Counter[str]:
    counts: Counter[str] = Counter()
    for label in labels:
        for line in label.read_text().splitlines():
            parts = line.split()
            if parts:
                counts[parts[0]] += 1
    return counts


def prepare_split(args: argparse.Namespace) -> Path:
    dataset_root = args.dataset_root.resolve()
    images_dir = dataset_root / "images"
    labels_dir = dataset_root / "labels"
    if not images_dir.is_dir() or not labels_dir.is_dir():
        raise FileNotFoundError(f"Expected images/ and labels/ under {dataset_root}")

    train_ratio = args.train_ratio
    if not 0.0 < train_ratio < 1.0:
        raise ValueError("--train-ratio must be between 0 and 1")

    pairs = image_label_pairs(dataset_root)
    if not pairs:
        raise RuntimeError(f"No labeled images found under {dataset_root}")

    rng = random.Random(args.seed)
    rng.shuffle(pairs)
    train_count = round(len(pairs) * train_ratio)
    train_pairs = pairs[:train_count]
    val_pairs = pairs[train_count:]

    split_dirs = [
        images_dir / "train",
        images_dir / "val",
        labels_dir / "train",
        labels_dir / "val",
    ]
    for split_dir in split_dirs:
        clean_split_dir(split_dir)

    for split, split_pairs in (("train", train_pairs), ("val", val_pairs)):
        for image_path, label_path in split_pairs:
            materialize_file(image_path, images_dir / split / image_path.name, args.link_method)
            materialize_file(label_path, labels_dir / split / label_path.name, args.link_method)

    names = read_class_names(dataset_root)
    yaml_path = write_dataset_yaml(dataset_root, names)

    train_boxes = count_boxes([label for _, label in train_pairs])
    val_boxes = count_boxes([label for _, label in val_pairs])
    print(f"dataset: {dataset_root}")
    print(f"yaml: {yaml_path}")
    print(f"split: train={len(train_pairs)} val={len(val_pairs)} ratio={train_ratio:.2f}")
    print(f"classes: {dict(enumerate(names))}")
    print(f"train boxes: {dict(sorted(train_boxes.items()))}")
    print(f"val boxes: {dict(sorted(val_boxes.items()))}")
    return yaml_path


def train_model(args: argparse.Namespace, data_yaml: Path) -> Path:
    from ultralytics import YOLO

    model = YOLO(args.model)
    results = model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=int(args.batch) if str(args.batch).isdigit() else args.batch,
        device=args.device,
        workers=args.workers,
        project=str(args.project),
        name=args.name,
        seed=args.seed,
        exist_ok=True,
    )
    best = Path(results.save_dir) / "weights" / "best.pt"
    if not best.exists():
        raise FileNotFoundError(f"Training finished, but best.pt was not found at {best}")
    print(f"best weights: {best}")
    return best


def bpu_detect_forward(self, x):
    """YOLO26 Detect.forward override for BPU-friendly raw NHWC outputs."""
    res = []

    if hasattr(self, "one2one_cv2") and hasattr(self, "one2one_cv3"):
        box_layers = self.one2one_cv2
        cls_layers = self.one2one_cv3
    else:
        box_layers = self.cv2
        cls_layers = self.cv3

    for i in range(self.nl):
        scores = cls_layers[i](x[i]).permute(0, 2, 3, 1)
        bboxes = box_layers[i](x[i]).permute(0, 2, 3, 1)
        res.append(scores)
        res.append(bboxes)

    return res


def default_export_output(weights_path: Path) -> Path:
    return weights_path.with_name(f"{weights_path.stem}_bpu.onnx")


def export_onnx(args: argparse.Namespace, weights: Path | None) -> Path:
    from ultralytics import YOLO
    from ultralytics.nn.modules import Detect

    weights_path = weights or args.weights or args.project / args.name / "weights" / "best.pt"
    weights_path = weights_path.resolve()
    if not weights_path.exists():
        raise FileNotFoundError(f"Weights not found: {weights_path}")

    output_path = (args.export_output or default_export_output(weights_path)).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    model = YOLO(str(weights_path))
    original_forward = Detect.forward
    Detect.forward = bpu_detect_forward
    print("Applying BPU YOLO26 Detect export patch: raw NHWC [Cls, Box] outputs.")
    try:
        exported = model.export(
            format="onnx",
            imgsz=args.imgsz,
            dynamic=False,
            opset=19,
            simplify=False,
            device=args.device,
        )
    finally:
        Detect.forward = original_forward

    if not exported:
        raise RuntimeError("BPU ONNX export failed: model.export returned no output path")

    exported_path = Path(exported).resolve()
    if exported_path != output_path:
        shutil.move(str(exported_path), str(output_path))

    print(f"bpu onnx: {output_path}")
    return output_path


def main() -> None:
    args = parse_args()
    do_prepare, do_train, do_export = split_flags(args)

    data_yaml = args.dataset_root / "perception_detector.yaml"
    if do_prepare:
        data_yaml = prepare_split(args)
    elif not data_yaml.exists():
        raise FileNotFoundError(f"Dataset YAML does not exist, run --prepare first: {data_yaml}")

    best_weights: Path | None = None
    if do_train:
        best_weights = train_model(args, data_yaml)

    if do_export:
        export_onnx(args, best_weights)


if __name__ == "__main__":
    main()
