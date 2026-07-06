"""
Convert VOC2012/VOC2017 format annotations to YOLO training txt format.

Usage:
    python voc2txt.py /path/to/voc/folder

The input folder should contain both images and XML annotation files.
Output will be saved to a new folder with '_txt' suffix alongside the original folder.
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from shutil import copy2

# ===================== Configuration =====================

# Input folder containing VOC images and XML annotations
INPUT_DIR = "/home/dcx/gazebo_room"

# Whether to skip objects marked as difficult=1
SKIP_DIFFICULT = True

# Predefined class list (in order). If non-empty, class-to-ID mapping follows this order.
# If empty, classes will be auto-scanned from XML files and sorted alphabetically.
CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",'checkboard'
]

# Supported image extensions
IMG_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".tif", ".webp"}

# ==========================================================


def parse_voc_xml(xml_path):
    """Parse a VOC XML annotation file and extract objects with bndbox and image size."""
    tree = ET.parse(xml_path)
    root = tree.getroot()

    # Get image filename from XML
    filename = root.find("filename").text

    # Get image size
    size = root.find("size")
    width = int(size.find("width").text)
    height = int(size.find("height").text)

    if width <= 0 or height <= 0:
        print(f"  [Warning] Invalid image size ({width}x{height}) in {xml_path}, skipping")
        return filename, width, height, []

    # Parse objects
    objects = []
    for obj in root.iter("object"):
        name = obj.find("name").text
        difficult = obj.find("difficult")
        # Skip difficult objects if flag is set
        if difficult is not None and int(difficult.text) == 1:
            continue

        bndbox = obj.find("bndbox")
        xmin = float(bndbox.find("xmin").text)
        ymin = float(bndbox.find("ymin").text)
        xmax = float(bndbox.find("xmax").text)
        ymax = float(bndbox.find("ymax").text)

        # Validate bbox
        if xmax <= xmin or ymax <= ymin:
            print(f"  [Warning] Invalid bbox ({xmin},{ymin},{xmax},{ymax}) in {xml_path}, skipping object")
            continue

        objects.append({"name": name, "xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax})

    return filename, width, height, objects


def voc_to_yolo(xmin, ymin, xmax, ymax, img_width, img_height):
    """Convert VOC bbox (xmin,ymin,xmax,ymax) to YOLO format (x_center, y_center, width, height) normalized."""
    x_center = (xmin + xmax) / 2.0 / img_width
    y_center = (ymin + ymax) / 2.0 / img_height
    width = (xmax - xmin) / img_width
    height = (ymax - ymin) / img_height

    # Clamp values to [0, 1] to avoid issues with edge cases
    x_center = max(0.0, min(1.0, x_center))
    y_center = max(0.0, min(1.0, y_center))
    width = max(0.0, min(1.0, width))
    height = max(0.0, min(1.0, height))

    return x_center, y_center, width, height


def build_class_names(xml_files):
    """Scan all XML files and build a sorted class name list."""
    class_set = set()
    for xml_file in xml_files:
        try:
            tree = ET.parse(xml_file)
            root = tree.getroot()
            for obj in root.iter("object"):
                name = obj.find("name").text
                difficult = obj.find("difficult")
                if difficult is not None and int(difficult.text) == 1:
                    continue
                class_set.add(name)
        except Exception as e:
            print(f"  [Warning] Error parsing {xml_file}: {e}")
            continue

    return sorted(class_set)


def convert_voc_to_yolo(input_dir, skip_difficult=True):
    """Convert VOC annotations in input_dir to YOLO txt format."""
    input_dir = Path(input_dir).resolve()
    if not input_dir.is_dir():
        print(f"[Error] Input directory does not exist: {input_dir}")
        sys.exit(1)

    # Find all XML files
    xml_files = sorted(input_dir.glob("*.xml"))
    if not xml_files:
        print(f"[Error] No XML files found in {input_dir}")
        sys.exit(1)

    print(f"[Info] Found {len(xml_files)} XML files in {input_dir}")

    # Build class mapping: use predefined CLASSES if set, otherwise auto-scan from XML
    if CLASSES:
        class_names = CLASSES
        print(f"[Info] Using predefined classes ({len(class_names)}): {class_names}")
    else:
        print("[Info] Scanning XML files to build class mapping...")
        class_names = build_class_names(xml_files)
        if not class_names:
            print("[Error] No valid objects found in any XML files")
            sys.exit(1)
        print(f"[Info] Found {len(class_names)} classes: {class_names}")

    class_to_id = {name: idx for idx, name in enumerate(class_names)}

    # Create output directory
    output_dir = input_dir.parent / f"{input_dir.name}_txt"
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"[Info] Output directory: {output_dir}")

    # Save class names file
    classes_file = output_dir / "classes.txt"
    with open(classes_file, "w") as f:
        for name in class_names:
            f.write(f"{name}\n")
    print(f"[Info] Saved class mapping to {classes_file}")

    # Process each XML file
    converted_count = 0
    error_count = 0
    missing_image_count = 0

    for xml_file in xml_files:
        try:
            filename, img_w, img_h, objects = parse_voc_xml(xml_file)
            if not objects:
                continue

            # Determine the stem (filename without extension)
            stem = Path(filename).stem

            # Find the corresponding image file
            img_path = None
            for ext in IMG_EXTENSIONS:
                candidate = input_dir / f"{stem}{ext}"
                if candidate.exists():
                    img_path = candidate
                    break
                # Also try the exact filename from XML
                candidate2 = input_dir / filename
                if candidate2.exists():
                    img_path = candidate2
                    break

            # Write YOLO txt file
            txt_path = output_dir / f"{stem}.txt"
            with open(txt_path, "w") as f:
                for obj in objects:
                    class_id = class_to_id[obj["name"]]
                    x_c, y_c, w, h = voc_to_yolo(obj["xmin"], obj["ymin"], obj["xmax"], obj["ymax"], img_w, img_h)
                    f.write(f"{class_id} {x_c:.6f} {y_c:.6f} {w:.6f} {h:.6f}\n")

            # Copy image to output directory
            if img_path is not None:
                copy2(img_path, output_dir / img_path.name)
            else:
                print(f"  [Warning] Image not found for '{filename}', label saved but no image copied")
                missing_image_count += 1

            converted_count += 1

        except Exception as e:
            print(f"  [Error] Failed to process {xml_file}: {e}")
            error_count += 1
            continue

    # Summary
    print("\n[Summary]")
    print(f"  XML files processed : {converted_count}")
    print(f"  Errors              : {error_count}")
    print(f"  Missing images      : {missing_image_count}")
    print(f"  Classes             : {len(class_names)}")
    print(f"  Output folder       : {output_dir}")

    if converted_count > 0:
        print("\n[Info] Conversion completed successfully!")
    else:
        print("\n[Warning] No files were converted. Check the input directory.")


# ===================== Run =====================

if __name__ == "__main__":
    convert_voc_to_yolo(INPUT_DIR, skip_difficult=SKIP_DIFFICULT)
