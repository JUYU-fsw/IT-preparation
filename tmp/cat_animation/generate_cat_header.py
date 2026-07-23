from pathlib import Path

from PIL import Image


source_dir = Path(r"D:\ai embody\TI preparation\tmp\cat_animation\frames")
output_file = Path(
    r"D:\ai embody\TI preparation\project\MSPM0G3507_Car\project\code\cat_animation.h"
)

frames = []
for image_file in sorted(source_dir.glob("frame_*.png")):
    image = Image.open(image_file).convert("1")
    frame = []
    for page in range(8):
        for x in range(128):
            value = 0
            for bit in range(8):
                if image.getpixel((x, page * 8 + bit)):
                    value |= 1 << bit
            frame.append(value)
    frames.append(frame)

lines = [
    "#ifndef _cat_animation_h_",
    "#define _cat_animation_h_",
    "",
    '#include "zf_common_typedef.h"',
    "",
    f"#define CAT_ANIMATION_FRAME_COUNT ({len(frames)})",
    "#define CAT_ANIMATION_FRAME_SIZE  (1024)",
    "",
    f"static const uint8 cat_animation_frames[{len(frames)}][1024] =",
    "{",
]

for frame in frames:
    lines.append("    {")
    for start in range(0, 1024, 16):
        row = ", ".join(f"0x{value:02X}" for value in frame[start : start + 16])
        lines.append(f"        {row},")
    lines.append("    },")

lines.extend(["};", "", "#endif", ""])
output_file.write_text("\n".join(lines), encoding="utf-8")

print(f"frames={len(frames)}")
print(f"binary_bytes={len(frames) * 1024}")
print(f"header_bytes={output_file.stat().st_size}")
