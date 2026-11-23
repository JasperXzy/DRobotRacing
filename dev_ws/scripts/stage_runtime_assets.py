#!/usr/bin/env python3
"""Stage the competition model and any local assets from the reference workspace."""

import argparse
import filecmp
import os
from pathlib import Path
import shutil
import tempfile


PERCEPTION_MODEL = Path('models/yolo26m_detect_nashe_640x640_nv12_resize_default.hbm')
LINE_MODEL = Path('models/line_detector_resnet18_nashe_224x224_nv12.hbm')
VLM_ENV = Path('src/vlm_tts_demo/config/vlm_api.env')


def check_existing(source, destination):
    if destination.exists() or destination.is_symlink():
        if not destination.is_file() or not filecmp.cmp(source, destination, shallow=False):
            raise FileExistsError(f'已有不同内容的文件：{destination}')
        return True
    return False


def stage_asset(source, destination):
    destination.parent.mkdir(parents=True, exist_ok=True)
    if check_existing(source, destination):
        return

    with tempfile.NamedTemporaryFile(
        dir=destination.parent, suffix=destination.suffix, delete=False,
    ) as file:
        temporary = Path(file.name)
        try:
            with source.open('rb') as original:
                shutil.copyfileobj(original, file)
            file.close()
            if destination.suffix != '.env':
                shutil.copymode(source, temporary)
            try:
                os.link(temporary, destination)
            except FileExistsError:
                check_existing(source, destination)
        finally:
            temporary.unlink()


def main():
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference', type=Path, nargs='?', default=workspace.parent / 'test/dev_ws')
    reference = parser.parse_args().reference.resolve()

    try:
        if not (reference / PERCEPTION_MODEL).is_file():
            raise FileNotFoundError(f'比赛感知模型缺失：{reference / PERCEPTION_MODEL}')
        assets = [asset for asset in (PERCEPTION_MODEL, LINE_MODEL, VLM_ENV)
                  if (reference / asset).is_file()]
        for asset in assets:
            check_existing(reference / asset, workspace / asset)
        for asset in assets:
            stage_asset(reference / asset, workspace / asset)
            print(f'已校验 {asset}')
    except OSError as error:
        parser.exit(1, f'{error}\n')


if __name__ == '__main__':
    main()
