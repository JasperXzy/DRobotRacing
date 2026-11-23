#!/usr/bin/env python3
"""
png_to_pgm.py — 将 PNG 标注层叠加到 PGM 地图上，在 RViz2 中显示并设禁行区

用法:
  python png_to_pgm.py                    # 将 PNG 中绿色+白色区域叠加到 PGM 设为禁行区
  python png_to_pgm.py --color-filter all # 所有非黑色像素都设为禁行区 (旧行为)
  python png_to_pgm.py --color-filter green  # 仅绿色区域
  python png_to_pgm.py --color-filter white  # 仅白色区域
  python png_to_pgm.py --interactive      # 交互式在地图上选择禁行区 (需要 matplotlib)
  python png_to_pgm.py --png-only         # 仅将 PNG 转为独立 PGM (不叠加现有PGM)
  python png_to_pgm.py --restore          # 恢复备份的原始 PGM

工作原理:
  1. 读取 my_map.pgm (当前导航地图)
  2. 读取 my_map.png (标注图片)
  3. 将 PNG 缩放对齐到 PGM 尺寸 (359x358)
  4. 默认: PNG 中的绿色和白色像素 → PGM 中对应位置设为 0 (禁行/占据)
     可通过 --color-filter 选择过滤模式:
       green+white (默认): 仅绿色+白色 → 禁行区
       all:               所有非黑色像素 → 禁行区
       green:             仅绿色像素 → 禁行区
       white:             仅白色像素 → 禁行区
  5. 额外禁行区多边形 (NO_GO_ZONES) → 也设为 0
  6. 保存 my_map.pgm，自动备份原文件为 my_map.pgm.bak

禁行区格式 (世界坐标, 米):
  YAML: resolution=0.025, origin=[-2.62, -2.96, 0]
  地图范围: x=[-2.62, 6.355], y=[-2.96, 5.99]
  换算: px = (wx - origin_x) / resolution
        py = (wy - origin_y) / resolution
"""

import os
import sys
import struct
import argparse
import shutil

# ============================================================
# 路径配置
# ============================================================
BASE = os.path.dirname(os.path.abspath(__file__))
PNG_PATH = os.path.join(BASE, "my_map.png")
PGM_PATH = os.path.join(BASE, "my_map.pgm")
YAML_PATH = os.path.join(BASE, "my_map.yaml")


# ============================================================
# YAML 参数
# ============================================================
def load_yaml_params(yaml_path):
    params = {}
    with open(yaml_path, 'r') as f:
        for line in f:
            line = line.strip()
            if ':' in line:
                k, v = line.split(':', 1)
                k, v = k.strip(), v.strip()
                if k == 'resolution':
                    params['resolution'] = float(v)
                elif k == 'origin':
                    v = v.strip('[]').split(',')
                    params['origin'] = [float(x) for x in v]
    return params


# ============================================================
# [*] 禁行区定义 — 在这里添加多边形 (世界坐标, 米) [*]
# ============================================================
# 地图范围:
#   x: [-2.62, 6.355]   (359px * 0.025m/px = 8.975m)
#   y: [-2.96, 5.99]    (358px * 0.025m/px = 8.95m)
#
# 每个禁行区是一个多边形顶点列表 [(x1,y1), (x2,y2), ...]
NO_GO_ZONES = [
    # 示例: 矩形禁行区
    # [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)],
    #
    # 示例: 三角形禁行区
    # [(-1.0, -1.0), (1.0, -1.0), (0.0, 1.0)],
    #
    # ==== 在这里添加你的禁行区 ====
]


# ============================================================
# PGM 读写
# ============================================================
def read_pgm(path):
    """读取 P5 PGM, 返回 (2D-array, width, height, maxval)"""
    import numpy as np
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxval = int(f.readline().strip())
        data = np.frombuffer(f.read(), dtype=np.uint8).reshape((h, w))
    return data, w, h, maxval


def save_pgm(data, path, maxval=255):
    """保存为 P5 二进制 PGM"""
    h, w = data.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n{w} {h}\n{maxval}\n'.encode('ascii'))
        f.write(data.tobytes())
    print(f"  已保存 PGM: {path} ({w}x{h}, maxval={maxval})")


# ============================================================
# PNG 读取 (纯 Python, 无外部依赖)
# ============================================================
def read_png_rgba(path):
    """读取 PNG 返回 (pixels_2d_array, w, h) — 每个像素是 (R,G,B,A)"""
    import zlib

    with open(path, 'rb') as f:
        assert f.read(8) == b'\x89PNG\r\n\x1a\n', "Not a valid PNG"

        w = h = bit_depth = color_type = None
        idat_chunks = []

        while True:
            lb = f.read(4)
            if len(lb) < 4: break
            length = struct.unpack('>I', lb)[0]
            ct = f.read(4)
            data = f.read(length)
            f.read(4)  # CRC

            if ct == b'IHDR':
                w = struct.unpack('>I', data[0:4])[0]
                h = struct.unpack('>I', data[4:8])[0]
                bit_depth = data[8]
                color_type = data[9]
            elif ct == b'IDAT':
                idat_chunks.append(data)
            elif ct == b'IEND':
                break

        assert w and h, "No IHDR found"

        raw = zlib.decompress(b''.join(idat_chunks))

        # 通道数
        ch_map = {0: 1, 2: 3, 4: 2, 6: 4}
        channels = ch_map.get(color_type, 3)
        bpp = channels * (bit_depth // 8)
        stride = w * bpp + 1

        pixels = []
        for y in range(h):
            row_start = y * stride
            row = raw[row_start + 1 : row_start + stride]

            row_px = []
            for x in range(w):
                idx = x * bpp
                if channels == 1:
                    row_px.append((row[idx], row[idx], row[idx], 255))
                elif channels == 2:
                    row_px.append((row[idx], row[idx], row[idx], row[idx+1]))
                elif channels == 3:
                    row_px.append((row[idx], row[idx+1], row[idx+2], 255))
                else:  # 4 (RGBA)
                    row_px.append((row[idx], row[idx+1], row[idx+2], row[idx+3]))
            pixels.append(row_px)

    return pixels, w, h


def is_green_pixel(r, g, b):
    """判断是否为绿色像素: G 通道主导, R/B 较低"""
    # 纯绿色或绿色调: G > 128 且 G 明显大于 R 和 B
    if g < 30:
        return False
    # G 至少是 R 的 1.5 倍, 至少是 B 的 1.5 倍
    if g > r * 1.5 and g > b * 1.5:
        return True
    return False


def is_white_pixel(r, g, b):
    """判断是否为白色/灰色像素: 三通道都较高且接近"""
    # 三个通道都较亮且彼此接近
    if r < 180 or g < 180 or b < 180:
        return False
    # 通道间差异不大于 60
    if abs(int(r) - int(g)) < 60 and abs(int(g) - int(b)) < 60 and abs(int(r) - int(b)) < 60:
        return True
    return False


def png_to_mask(pixels, w, h, target_w, target_h, color_filter='green+white'):
    """将 PNG 转为二值 mask: True = 禁行区内容, False = 忽略

    color_filter 选项:
        'green+white' — 仅绿色和白色区域设为禁行区 (默认)
        'all'         — 所有非黑色像素设为禁行区 (旧行为)
        'green'       — 仅绿色区域
        'white'       — 仅白色区域
    """
    import numpy as np
    from PIL import Image

    gray = np.zeros((h, w), dtype=np.uint8)
    for y in range(h):
        row = pixels[y]
        for x in range(w):
            r, g, b, a = row[x]

            if color_filter == 'all':
                # 旧行为: 任意通道非零即内容
                if r > 0 or g > 0 or b > 0:
                    gray[y, x] = 255
                else:
                    gray[y, x] = 0
            elif color_filter == 'green':
                if is_green_pixel(r, g, b):
                    gray[y, x] = 255
                else:
                    gray[y, x] = 0
            elif color_filter == 'white':
                if is_white_pixel(r, g, b):
                    gray[y, x] = 255
                else:
                    gray[y, x] = 0
            else:  # 'green+white' (默认)
                if is_green_pixel(r, g, b) or is_white_pixel(r, g, b):
                    gray[y, x] = 255
                else:
                    gray[y, x] = 0

    # 缩放到目标尺寸
    img = Image.fromarray(gray)
    img = img.resize((target_w, target_h), Image.LANCZOS)
    scaled = np.array(img, dtype=np.uint8)

    # 阈值化: > 64 视为内容 (LANCZOS 会产生中间灰度)
    mask = scaled > 64
    return mask


# ============================================================
# 禁行区绘制
# ============================================================
def world_to_pixel(wx, wy, resolution, origin):
    px = int((wx - origin[0]) / resolution)
    py = int((wy - origin[1]) / resolution)
    return px, py


def apply_no_go_zones(data, zones, resolution, origin, w, h):
    """在 PGM 数据上标记禁行区 (像素值 0 = 占据)"""
    import numpy as np
    from PIL import Image, ImageDraw

    if not zones:
        return data

    img = Image.fromarray(data)
    draw = ImageDraw.Draw(img)

    for i, zone in enumerate(zones):
        pts = [world_to_pixel(x, y, resolution, origin) for (x, y) in zone]
        pts = [(max(0, min(w-1, x)), max(0, min(h-1, y))) for (x, y) in pts]
        draw.polygon(pts, fill=0)
        print(f"  禁行区 #{i+1}: 世界 {zone}")
        print(f"            像素 {pts}")

    return np.array(img, dtype=np.uint8)


# ============================================================
# 交互式选择
# ============================================================
def interactive_select_zones(gray_data, resolution, origin):
    """matplotlib 交互式选禁行区"""
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(gray_data, cmap='gray', origin='lower')
    ax.set_title("Define no-go zone polygons\n"
                 "Left: add vertex | Right: close polygon | Middle: exit")
    ax.set_xlabel(f"X pixel (world: [{origin[0]:.1f}, "
                  f"{origin[0] + gray_data.shape[1]*resolution:.1f}]m)")
    ax.set_ylabel(f"Y pixel (world: [{origin[1]:.1f}, "
                  f"{origin[1] + gray_data.shape[0]*resolution:.1f}]m)")

    zones_world = []
    current_zone = []
    current_world = []

    def onclick(event):
        nonlocal current_zone, current_world
        if event.inaxes != ax:
            return
        if event.button == 1:
            px, py = int(event.xdata), int(event.ydata)
            wx = px * resolution + origin[0]
            wy = py * resolution + origin[1]
            current_zone.append((px, py))
            current_world.append((wx, wy))
            ax.plot(px, py, 'ro', markersize=4)
            fig.canvas.draw()
            print(f"  + vertex: px({px},{py}) <-> world({wx:.2f},{wy:.2f})")
        elif event.button == 3:
            if len(current_zone) >= 3:
                current_zone.append(current_zone[0])
                current_world.append(current_world[0])
                xs, ys = zip(*current_zone)
                ax.plot(xs, ys, 'r-', linewidth=2)
                ax.fill(xs, ys, alpha=0.3, color='red')
                fig.canvas.draw()
                zones_world.append(current_world[:-1])
                print(f"  [OK] polygon: {current_world[:-1]}")
                current_zone, current_world = [], []
            else:
                print("  [!] Need at least 3 vertices")
        elif event.button == 2:
            plt.close()

    fig.canvas.mpl_connect('button_press_event', onclick)
    plt.show()
    return zones_world


# ============================================================
# 统计
# ============================================================
def print_stats(data, label=""):
    import numpy as np
    h, w = data.shape
    occ = np.sum(data == 0)
    free = np.sum(data == 254)
    unk = np.sum(data == 205)
    other = h * w - occ - free - unk
    print(f"\n  [{label}] total={h*w}")
    print(f"    occupied(0):     {occ:6d} ({occ/(h*w)*100:.1f}%)")
    print(f"    free(254):      {free:6d} ({free/(h*w)*100:.1f}%)")
    print(f"    unknown(205):   {unk:6d} ({unk/(h*w)*100:.1f}%)")
    if other:
        print(f"    other:           {other:6d} ({other/(h*w)*100:.1f}%)")


# ============================================================
# 主流程
# ============================================================
def main():
    parser = argparse.ArgumentParser(
        description='Overlay PNG annotations onto PGM map, with no-go zones')
    parser.add_argument('--interactive', '-i', action='store_true',
                        help='Interactive no-go zone selection')
    parser.add_argument('--png-only', action='store_true',
                        help='Convert PNG to standalone PGM (no base PGM overlay)')
    parser.add_argument('--restore', action='store_true',
                        help='Restore original PGM from backup')
    parser.add_argument('--keep-annotations', action='store_true',
                        help='Keep PNG annotations as gray instead of black (visual only)')
    parser.add_argument('--color-filter', default='green+white',
                        choices=['green+white', 'all', 'green', 'white'],
                        help='Color filter for no-go zones (default: green+white)')
    args = parser.parse_args()

    # --- restore ---
    if args.restore:
        bak = PGM_PATH + '.bak'
        if os.path.exists(bak):
            shutil.copy(bak, PGM_PATH)
            print(f"[OK] Restored {PGM_PATH} from backup")
        else:
            print(f"[X] No backup found: {bak}")
        return

    # --- load YAML ---
    if not os.path.exists(YAML_PATH):
        print(f"[X] YAML not found: {YAML_PATH}")
        sys.exit(1)
    yaml = load_yaml_params(YAML_PATH)
    resolution = yaml['resolution']
    origin = yaml['origin']
    print(f"YAML: resolution={resolution}, origin={origin}")

    # --- load base PGM ---
    if os.path.exists(PGM_PATH) and not args.png_only:
        base_pgm, pgm_w, pgm_h, maxval = read_pgm(PGM_PATH)
        print(f"Base PGM: {pgm_w}x{pgm_h}")
        print_stats(base_pgm, "before")
    else:
        import numpy as np
        pgm_w, pgm_h = 359, 358
        base_pgm = np.full((pgm_h, pgm_w), 205, dtype=np.uint8)  # all unknown
        print(f"No base PGM, creating {pgm_w}x{pgm_h} unknown map")

    # --- load PNG and create mask ---
    if os.path.exists(PNG_PATH):
        print(f"\nLoading PNG: {PNG_PATH}")
        png_pixels, png_w, png_h = read_png_rgba(PNG_PATH)
        print(f"  PNG: {png_w}x{png_h}")

        print(f"  Creating mask (filter={args.color_filter}) and scaling {png_w}x{png_h} -> {pgm_w}x{pgm_h}...")
        mask = png_to_mask(png_pixels, png_w, png_h, pgm_w, pgm_h, args.color_filter)
        mask_pixels = int(mask.sum())
        print(f"  Mask pixels (content): {mask_pixels} "
              f"({mask_pixels/(pgm_w*pgm_h)*100:.1f}% of map)")
    else:
        print(f"[!] PNG not found: {PNG_PATH}, skipping PNG overlay")
        import numpy as np
        mask = np.zeros((pgm_h, pgm_w), dtype=bool)

    # --- merge: apply PNG mask to PGM ---
    import numpy as np
    result = base_pgm.copy()

    if mask.any():
        fill_val = 0  # 0 = occupied = no-go
        if args.keep_annotations:
            fill_val = 128  # mid gray for visual distinction
        result[mask] = fill_val
        print(f"  PNG overlay applied: {mask_pixels} pixels -> value {fill_val}")

    # --- collect no-go zones ---
    zones = list(NO_GO_ZONES)

    if args.interactive:
        print("\n[Interactive mode] Click on map to define no-go zones")
        interactive_zones = interactive_select_zones(result, resolution, origin)
        zones.extend(interactive_zones)

    # --- apply no-go zones ---
    if zones:
        print(f"\nApplying {len(zones)} no-go zone(s)...")
        result = apply_no_go_zones(result, zones, resolution, origin, pgm_w, pgm_h)
    else:
        print("\n[!] No no-go zones defined. Edit NO_GO_ZONES in script or use --interactive")

    # --- backup & save ---
    if os.path.exists(PGM_PATH):
        bak_path = PGM_PATH + '.bak'
        if not os.path.exists(bak_path):
            shutil.copy(PGM_PATH, bak_path)
            print(f"\nBackup: {PGM_PATH} -> {bak_path}")
        else:
            print(f"\nBackup exists: {bak_path} (not overwritten)")

    save_pgm(result, PGM_PATH)

    # --- final stats ---
    print_stats(result, "after")
    print(f"\n[OK] Done! PGM {pgm_w}x{pgm_h}, YAML unchanged (resolution={resolution}, origin={origin})")


if __name__ == '__main__':
    main()
