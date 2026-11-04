"""Convert emulator PPM frames to PNGs plus a contact sheet.

Usage: sheet.py <framedir> <sheet.png> [cols]
"""
import sys
from pathlib import Path
from PIL import Image

framedir = Path(sys.argv[1])
out = Path(sys.argv[2])
cols = int(sys.argv[3]) if len(sys.argv) > 3 else 4
frames = sorted(framedir.glob("f*.ppm"))
if not frames:
    sys.exit(f"no frames in {framedir}")
imgs = []
for f in frames:
    im = Image.open(f).convert("RGB")
    im.save(f.with_suffix(".png"))
    imgs.append(im)
w, h = imgs[0].size
rows = (len(imgs) + cols - 1) // cols
sheet = Image.new("RGB", (cols * w, rows * h), (10, 10, 16))
for i, im in enumerate(imgs):
    sheet.paste(im, ((i % cols) * w, (i // cols) * h))
sheet.save(out)
print(f"{len(imgs)} frames -> {out} ({sheet.size[0]}x{sheet.size[1]})")
