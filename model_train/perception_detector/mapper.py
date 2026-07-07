#!/usr/bin/env python3
"""Convert the exported YOLO26 detector ONNX model to an RDK S100 HBM model."""

from __future__ import annotations

import argparse
import logging
import shutil
import subprocess
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_RUN_NAME = "yolo26n_v1"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
MARCH_TO_NAME = {
    "nash-e": "nashe",
    "nash-m": "nashm",
    "nash-p": "nashp",
}


logging.basicConfig(
    level=logging.INFO,
    format="[%(name)s] [%(asctime)s.%(msecs)03d] [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
LOGGER = logging.getLogger("PERCEPTION_MAPPER")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--onnx",
        type=Path,
        default=SCRIPT_DIR / "runs" / DEFAULT_RUN_NAME / "weights" / "best_bpu.onnx",
        help="BPU-friendly ONNX exported by train.py --export.",
    )
    parser.add_argument(
        "--cal-images",
        type=Path,
        default=SCRIPT_DIR / "cal_images",
        help="Calibration image directory. Use 20-50 representative jpg/png images.",
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
        help="Optional HBM file prefix. Default: yolo26_detect_<march>_<width>x<height>_nv12.",
    )
    parser.add_argument(
        "--march",
        choices=sorted(MARCH_TO_NAME),
        default="nash-e",
        help="Target architecture. RDK S100 uses nash-e; S100P uses nash-m.",
    )
    parser.add_argument(
        "--quantized",
        choices=("int8", "int16"),
        default="int8",
        help="Quantization precision.",
    )
    parser.add_argument(
        "--calibration-type",
        choices=("max", "kl", "default"),
        default="max",
        help="Activation calibration method. Use max to avoid default modelwise search crashes.",
    )
    parser.add_argument("--jobs", type=int, default=16, help="hb_compile parallel jobs.")
    parser.add_argument(
        "--optimize-level",
        choices=("O0", "O1", "O2"),
        default="O2",
        help="Nash compiler optimization level.",
    )
    parser.add_argument("--seed", type=int, default=42, help="Calibration image sampling seed.")
    parser.add_argument(
        "--cal-sample-num",
        type=int,
        default=20,
        help="Number of calibration images to sample when the directory has more images.",
    )
    parser.add_argument(
        "--no-cal-sample",
        action="store_true",
        help="Use all calibration images instead of sampling --cal-sample-num images.",
    )
    parser.add_argument(
        "--workspace",
        type=Path,
        default=None,
        help="Temporary workspace. Default: <output-dir>/.mapper_workspace.",
    )
    parser.add_argument(
        "--save-cache",
        action="store_true",
        help="Keep the temporary workspace after conversion.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Generate calibration data and config.yaml, but do not run hb_compile.",
    )
    return parser.parse_args()


def require_tool(name: str) -> None:
    if shutil.which(name) is None:
        raise RuntimeError(
            f"{name} was not found. Run this script inside the RDK S100 OpenExplore Docker/toolchain environment."
        )


def resolve_path(path: Path) -> Path:
    return path.expanduser().resolve()


def infer_run_dir(onnx_path: Path) -> Path:
    if onnx_path.parent.name == "weights":
        return onnx_path.parent.parent
    return onnx_path.parent


def get_onnx_input_size(onnx_path: Path) -> tuple[int, int]:
    import onnxruntime as ort

    session = ort.InferenceSession(str(onnx_path), providers=["CPUExecutionProvider"])
    inputs = session.get_inputs()
    if len(inputs) != 1:
        raise RuntimeError(f"Expected exactly one ONNX input, got {len(inputs)}")

    shape = inputs[0].shape
    if len(shape) != 4:
        raise RuntimeError(f"Expected NCHW ONNX input, got shape {shape}")

    height, width = shape[2], shape[3]
    if not isinstance(height, int) or not isinstance(width, int):
        raise RuntimeError(f"ONNX input height/width must be static, got shape {shape}")
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
        LOGGER.warning(
            "Only %d calibration images selected; RDK examples recommend 20-50 images.",
            len(chosen),
        )
    elif len(chosen) > 50:
        LOGGER.warning(
            "%d calibration images selected; conversion may be slower than necessary.",
            len(chosen),
        )
    return chosen


def prepare_workspace(workspace: Path, save_cache: bool) -> tuple[Path, Path]:
    if workspace.exists() and not save_cache:
        shutil.rmtree(workspace)
    workspace.mkdir(parents=True, exist_ok=True)

    cal_data_dir = workspace / "calibration_data_rgb_f32"
    if cal_data_dir.exists():
        shutil.rmtree(cal_data_dir)
    cal_data_dir.mkdir(parents=True, exist_ok=True)

    bpu_output_dir = workspace / "bpu_model_output"
    if bpu_output_dir.exists():
        shutil.rmtree(bpu_output_dir)
    bpu_output_dir.mkdir(parents=True, exist_ok=True)
    return cal_data_dir, bpu_output_dir


def write_calibration_data(images: list[Path], cal_data_dir: Path, width: int, height: int) -> None:
    import cv2
    import numpy as np

    for image_path in images:
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            LOGGER.warning("Skipping unreadable calibration image: %s", image_path)
            continue

        tensor = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        tensor = cv2.resize(tensor, (width, height), interpolation=cv2.INTER_LINEAR)
        tensor = np.transpose(tensor, (2, 0, 1))
        tensor = np.expand_dims(tensor, axis=0).astype(np.float32) / 255.0
        np.save(cal_data_dir / f"{image_path.stem}.npy", tensor)

    if not any(cal_data_dir.glob("*.npy")):
        raise RuntimeError(f"No calibration .npy files were generated under {cal_data_dir}")


def build_output_prefix(args: argparse.Namespace, width: int, height: int) -> str:
    if args.output_prefix:
        return args.output_prefix
    return f"yolo26_detect_{MARCH_TO_NAME[args.march]}_{width}x{height}_nv12"


def build_quant_config(quantized: str) -> str:
    if quantized != "int16":
        return ""
    return """
  quant_config: {
        "model_config": {
            "all_node_type": "int16",
            "model_output_type": "int16",
        }
    }"""


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
  layer_out_dump: False
  working_dir: '{bpu_output_dir}'
  output_model_file_prefix: '{output_prefix}'
input_parameters:
  input_name: ''
  input_type_rt: 'nv12'
  input_type_train: 'rgb'
  input_layout_train: 'NCHW'
  norm_type: 'data_scale'
  scale_value: 0.003921568627451
calibration_parameters:
  cal_data_dir: '{cal_data_dir}'
  cal_data_type: 'float32'
  calibration_type: '{args.calibration_type}'{build_quant_config(args.quantized)}
compiler_parameters:
  extra_params: {{'input_no_padding': True, 'output_no_padding': True}}
  jobs: {args.jobs}
  compile_mode: 'latency'
  debug: true
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
