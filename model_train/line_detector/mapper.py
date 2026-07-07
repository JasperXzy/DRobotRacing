#!/usr/bin/env python3
"""Convert the exported ResNet18 line detector ONNX model to an HBM model.

This follows the OpenExplorer v3.7 ResNet18 sample style:
``hb_compile --config config.yaml`` with NV12 runtime input, RGB/NCHW training
input, and ImageNet mean/std preprocessing folded into the compiled model.
"""

from __future__ import annotations

import argparse
import logging
import shutil
import subprocess
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DATASET_ROOT = Path("/home/jasperxzy/Datasets/D_Robot/Line_Detector/v1")
DEFAULT_RUN_NAME = "resnet18_v1"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
MARCH_TO_NAME = {
    "nash-e": "nashe",
    "nash-m": "nashm",
    "nash-p": "nashp",
}
IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


logging.basicConfig(
    level=logging.INFO,
    format="[%(name)s] [%(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
LOGGER = logging.getLogger("LINE_MAPPER")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--onnx",
        type=Path,
        default=SCRIPT_DIR / "runs" / DEFAULT_RUN_NAME / "weights" / "best_bpu.onnx",
        help="BPU-friendly ONNX exported by train.py --export.",
    )
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument(
        "--cal-images",
        type=Path,
        default=SCRIPT_DIR / "cal_images",
        help="Calibration image directory. Default: ./cal_images.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for the final .hbm file and hb_compile.log. Default: the ONNX run directory.",
    )
    parser.add_argument(
        "--output-prefix",
        default=None,
        help="Optional HBM file prefix. Default: line_detector_resnet18_<march>_<width>x<height>_nv12.",
    )
    parser.add_argument(
        "--march",
        choices=sorted(MARCH_TO_NAME),
        default="nash-e",
        help="Target architecture. RDK S100 uses nash-e; S100P uses nash-m.",
    )
    parser.add_argument(
        "--calibration-type",
        choices=("default", "mix", "kl", "max", "load", "skip"),
        default="default",
    )
    parser.add_argument(
        "--optimize-level",
        choices=("O0", "O1", "O2", "O3"),
        default="O2",
    )
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--cal-sample-num", type=int, default=100)
    parser.add_argument("--no-cal-sample", action="store_true")
    parser.add_argument(
        "--workspace",
        type=Path,
        default=None,
        help="Temporary workspace. Default: <output-dir>/.mapper_workspace.",
    )
    parser.add_argument("--save-cache", action="store_true")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Generate calibration data and config.yaml, but do not run hb_compile.",
    )
    return parser.parse_args()


def require_tool(name: str) -> None:
    if shutil.which(name) is None:
        raise RuntimeError(
            f"{name} was not found. Run this script inside the OpenExplorer v3.7 AI toolchain environment."
        )


def resolve_path(path: Path) -> Path:
    return path.expanduser().resolve()


def infer_run_dir(onnx_path: Path) -> Path:
    if onnx_path.parent.name == "weights":
        return onnx_path.parent.parent
    return onnx_path.parent


def _dim_value(dim) -> int | None:
    value = getattr(dim, "dim_value", 0)
    return int(value) if value else None


def get_onnx_input_size(onnx_path: Path) -> tuple[int, int]:
    try:
        import onnx

        model = onnx.load(str(onnx_path))
        inputs = [value for value in model.graph.input if value.type.tensor_type.shape.dim]
        if len(inputs) != 1:
            raise RuntimeError(f"Expected exactly one ONNX input, got {len(inputs)}")
        dims = [_dim_value(dim) for dim in inputs[0].type.tensor_type.shape.dim]
    except ImportError:
        import onnxruntime as ort

        session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
        inputs = session.get_inputs()
        if len(inputs) != 1:
            raise RuntimeError(f"Expected exactly one ONNX input, got {len(inputs)}")
        dims = inputs[0].shape

    if len(dims) != 4:
        raise RuntimeError(f"Expected NCHW ONNX input, got shape {dims}")
    height, width = dims[2], dims[3]
    if not isinstance(height, int) or not isinstance(width, int):
        raise RuntimeError(f"ONNX input height/width must be static, got shape {dims}")
    return width, height


def list_calibration_images(cal_images: Path) -> list[Path]:
    if not cal_images.is_dir():
        raise FileNotFoundError(f"Calibration image directory does not exist: {cal_images}")
    images = sorted(
        path
        for path in cal_images.iterdir()
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )
    if not images:
        raise RuntimeError(f"No calibration images found in {cal_images}")
    return images


def choose_calibration_images(images: list[Path], args: argparse.Namespace) -> list[Path]:
    import numpy as np

    if args.no_cal_sample or len(images) <= args.cal_sample_num:
        chosen = images
    else:
        rng = np.random.default_rng(args.seed)
        indices = sorted(rng.choice(len(images), size=args.cal_sample_num, replace=False).tolist())
        chosen = [images[index] for index in indices]

    if len(chosen) < 20:
        LOGGER.warning("Only %d calibration images selected; 20-100 is recommended.", len(chosen))
    elif len(chosen) > 100:
        LOGGER.warning("%d calibration images selected; conversion may be slower than necessary.", len(chosen))
    return chosen


def prepare_workspace(workspace: Path, save_cache: bool) -> tuple[Path, Path]:
    if workspace.exists() and not save_cache:
        shutil.rmtree(workspace)
    workspace.mkdir(parents=True, exist_ok=True)

    cal_data_dir = workspace / "calibration_data_rgb"
    if cal_data_dir.exists():
        shutil.rmtree(cal_data_dir)
    cal_data_dir.mkdir(parents=True, exist_ok=True)

    bpu_output_dir = workspace / "model_output"
    if bpu_output_dir.exists():
        shutil.rmtree(bpu_output_dir)
    bpu_output_dir.mkdir(parents=True, exist_ok=True)
    return cal_data_dir, bpu_output_dir


def preprocess_image(image_path: Path, width: int, height: int):
    import numpy as np
    from PIL import Image

    image = Image.open(image_path).convert("RGB")
    image = image.resize((width, height), Image.BILINEAR)
    array = np.asarray(image, dtype=np.float32) / 255.0
    array = (array - np.asarray(IMAGENET_MEAN, dtype=np.float32)) / np.asarray(IMAGENET_STD, dtype=np.float32)
    return np.transpose(array, (2, 0, 1)).astype(np.float32)


def write_calibration_data(images: list[Path], cal_data_dir: Path, width: int, height: int) -> None:
    import numpy as np

    for index, image_path in enumerate(images):
        tensor = preprocess_image(image_path, width, height)
        np.save(cal_data_dir / f"{index:04d}_{image_path.stem}.npy", tensor)

    if not any(cal_data_dir.glob("*.npy")):
        raise RuntimeError(f"No calibration .npy files were generated under {cal_data_dir}")


def build_output_prefix(args: argparse.Namespace, width: int, height: int) -> str:
    if args.output_prefix:
        return args.output_prefix
    return f"line_detector_resnet18_{MARCH_TO_NAME[args.march]}_{width}x{height}_nv12"


def write_config(
    args: argparse.Namespace,
    cal_data_dir: Path,
    bpu_output_dir: Path,
    output_prefix: str,
) -> Path:
    config_path = args.workspace / "config.yaml"
    config_path.write_text(
        f"""model_parameters:
  onnx_model: '{args.onnx}'
  march: '{args.march}'
  working_dir: '{bpu_output_dir}'
  output_model_file_prefix: '{output_prefix}'
input_parameters:
  input_name: ''
  input_type_rt: 'nv12'
  input_type_train: 'rgb'
  input_layout_train: 'NCHW'
  input_shape: ''
  norm_type: 'data_mean_and_scale'
  mean_value: '123.675 116.28 103.53'
  scale_value: '0.01712475 0.017507 0.01742919'
calibration_parameters:
  cal_data_dir: '{cal_data_dir}'
  calibration_type: '{args.calibration_type}'
compiler_parameters:
  optimize_level: '{args.optimize_level}'
""",
        encoding="utf-8",
    )
    return config_path


def run_hb_compile(config_path: Path, workspace: Path) -> None:
    subprocess.run(["hb_compile", "--config", str(config_path)], cwd=workspace, check=True)


def move_outputs(args: argparse.Namespace, bpu_output_dir: Path, output_prefix: str) -> Path:
    args.output_dir.mkdir(parents=True, exist_ok=True)

    source_hbm = bpu_output_dir / f"{output_prefix}.hbm"
    if not source_hbm.exists():
        raise FileNotFoundError(f"hb_compile finished, but HBM was not found: {source_hbm}")

    final_hbm = args.output_dir / source_hbm.name
    if final_hbm.exists():
        final_hbm.unlink()
    shutil.move(str(source_hbm), str(final_hbm))

    log_source = args.workspace / "hb_compile.log"
    if log_source.exists():
        log_dest = args.output_dir / "hb_compile.log"
        if log_dest.exists():
            log_dest.unlink()
        shutil.move(str(log_source), str(log_dest))

    return final_hbm


def main() -> None:
    args = parse_args()
    args.onnx = resolve_path(args.onnx)
    args.dataset_root = resolve_path(args.dataset_root)
    args.cal_images = resolve_path(args.cal_images)
    args.output_dir = resolve_path(args.output_dir) if args.output_dir else infer_run_dir(args.onnx)
    args.workspace = resolve_path(args.workspace) if args.workspace else args.output_dir / ".mapper_workspace"

    if not args.onnx.is_file():
        raise FileNotFoundError(f"ONNX file does not exist: {args.onnx}")
    if not args.dry_run:
        require_tool("hb_compile")

    width, height = get_onnx_input_size(args.onnx)
    images = choose_calibration_images(list_calibration_images(args.cal_images), args)
    cal_data_dir, bpu_output_dir = prepare_workspace(args.workspace, args.save_cache)
    output_prefix = build_output_prefix(args, width, height)

    LOGGER.info("ONNX: %s", args.onnx)
    LOGGER.info("input: %dx%d", width, height)
    LOGGER.info("calibration images: %d from %s", len(images), args.cal_images)
    LOGGER.info("workspace: %s", args.workspace)
    LOGGER.info("output prefix: %s", output_prefix)

    write_calibration_data(images, cal_data_dir, width, height)
    config_path = write_config(args, cal_data_dir, bpu_output_dir, output_prefix)
    LOGGER.info("config: %s", config_path)

    if args.dry_run:
        LOGGER.info("Dry run complete. hb_compile was not executed.")
        return

    run_hb_compile(config_path, args.workspace)
    final_hbm = move_outputs(args, bpu_output_dir, output_prefix)
    LOGGER.info("HBM: %s", final_hbm)

    if not args.save_cache:
        shutil.rmtree(args.workspace)
        LOGGER.info("Workspace cleaned.")


if __name__ == "__main__":
    main()
