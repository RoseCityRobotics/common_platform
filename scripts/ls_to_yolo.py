import json, os, shutil, pathlib, glob, re, sys
from PIL import Image

# === EDIT THESE IF NEEDED ===
LS_SOURCE = "data/annotations"               # directory OR a single JSON export file
IMG_ROOT  = "data/images_raw"           # where your original images live (optional)
OUT_IMG   = "data/images"
OUT_LBL   = "data/labels"
# ============================

os.makedirs(OUT_IMG, exist_ok=True)
os.makedirs(OUT_LBL, exist_ok=True)

def resolve_ls_image_path(img_ref: str) -> str:
    """
    Handles LS local-files URLs like: /data/local-files/?d=home/rcr/teleop_data/images/0001.jpg
    and returns a best-guess absolute/relative filesystem path.
    """
    if not img_ref:
        return ""
    # If it’s like /data/local-files/?d=home/... turn into /home/...
    m = re.search(r"\?d=([^#]+)$", img_ref)
    if m:
        p = m.group(1)
        # ensure leading slash
        if not p.startswith("/"):
            p = "/" + p
        return p
    return img_ref

def bbox_xywh_to_yolo(x, y, w, h, W, H):
    xc = (x + w/2) / W
    yc = (y + h/2) / H
    ww = w / W
    hh = h / H
    return xc, yc, ww, hh

def load_tasks(ls_source):
    if os.path.isdir(ls_source):
        files = sorted(glob.glob(os.path.join(ls_source, "*")))
        tasks = []
        for fp in files:
            try:
                with open(fp, "r") as f:
                    tasks.append(json.load(f))
            except Exception as e:
                print(f"⚠️  Skipping {fp}: {e}")
        return tasks
    else:
        with open(ls_source, "r") as f:
            data = json.load(f)
        # allow single object or list
        return data if isinstance(data, list) else [data]

tasks = load_tasks(LS_SOURCE)

class_to_id = {}
def get_class_id(name):
    if name not in class_to_id:
        class_to_id[name] = len(class_to_id)
    return class_to_id[name]

stats = {
    "total_items": 0,
    "with_any_result": 0,
    "with_boxes": 0,
    "images_copied": 0,
    "missing_images": 0,
    "empty_results": 0
}

def iter_results(t):
    """
    Yield (result_entry, original_width, original_height) across schema variants:
    A) Top-level 'result' (your sample)
    B) t['annotations'][i]['result'] (classic LS export)
    """
    # Variant A: top-level
    if isinstance(t.get("result"), list):
        yield from ((r, t.get("original_width") or t.get("originalWidth"), t.get("original_height") or t.get("originalHeight")) for r in t["result"])
        return
    # Variant B: under annotations
    anns = t.get("annotations") or t.get("completions") or []
    for ann in anns:
        res = ann.get("result", [])
        for r in res:
            yield r, ann.get("original_width") or ann.get("originalWidth"), ann.get("original_height") or ann.get("originalHeight")

num_ok = 0
for t in tasks:
    stats["total_items"] += 1

    # Pull image reference from common places
    img_ref = None
    if "task" in t and isinstance(t["task"], dict):
        img_ref = (t["task"].get("data", {}) or {}).get("image")
    if not img_ref:
        img_ref = (t.get("data", {}) or {}).get("image")
    img_ref = resolve_ls_image_path(img_ref)

    # Choose source image path
    src_img = ""
    if img_ref and os.path.exists(img_ref):
        src_img = img_ref
    elif img_ref:
        base = os.path.basename(img_ref)
        candidate = os.path.join(IMG_ROOT, base)
        if os.path.exists(candidate):
            src_img = candidate

    # If still missing, try looser search inside IMG_ROOT
    if not src_img and IMG_ROOT and os.path.isdir(IMG_ROOT):
        base = os.path.basename(img_ref) if img_ref else None
        if base:
            hits = glob.glob(os.path.join(IMG_ROOT, "**", base), recursive=True)
            if hits:
                src_img = hits[0]

    # Prepare output names
    stem = pathlib.Path(os.path.basename(img_ref) if img_ref else f"item_{stats['total_items']:06d}").stem
    dst_img = os.path.join(OUT_IMG, stem + ".jpg")  # we’ll keep extension later if needed
    dst_lbl = os.path.join(OUT_LBL, f"{stem}.txt")

    # Figure out results
    results = list(iter_results(t))
    if results:
        stats["with_any_result"] += 1
    else:
        stats["empty_results"] += 1

    yolo_lines = []
    # Determine image size
    # Prefer reading from file for ground truth W,H; fallback to any provided dimensions
    W = H = None
    if src_img and os.path.exists(src_img):
        try:
            with Image.open(src_img) as im:
                W, H = im.size
        except Exception as e:
            print(f"⚠️  Could not open image {src_img}: {e}")

    # Process results
    for r, ow, oh in results:
        if r.get("type") not in ("rectanglelabels", "labels", "rectangle"):
            continue
        val = r.get("value", {})
        labels = val.get("rectanglelabels") or val.get("labels") or []
        if not labels:
            continue
        cls_id = get_class_id(labels[0])

        x = val.get("x"); y = val.get("y"); w = val.get("width"); h = val.get("height")
        if None in (x, y, w, h):
            continue

        # Determine image W,H
        _W = W or ow or oh  # try available values
        _H = H or oh or ow
        if not (W and H) and (ow and oh):
            W, H = ow, oh
        if not (W and H) and isinstance(_W, (int,float)) and isinstance(_H, (int,float)):
            W, H = int(_W), int(_H)

        if not (W and H) and src_img and os.path.exists(src_img):
            with Image.open(src_img) as im:
                W, H = im.size

        if not (W and H):
            print(f"⚠️  Missing image size for {stem}, skipping a box.")
            continue

        # LS uses percent or fraction or pixels; normalize
        if 0 <= w <= 1 and 0 <= h <= 1 and 0 <= x <= 1 and 0 <= y <= 1:
            x *= W; y *= H; w *= W; h *= H
        elif 0 < w <= 100 and 0 < h <= 100 and 0 <= x <= 100 and 0 <= y <= 100:
            x = x/100.0 * W; y = y/100.0 * H; w = w/100.0 * W; h = h/100.0 * H
        # else assume already pixels

        xc, yc, ww, hh = bbox_xywh_to_yolo(x, y, w, h, W, H)
        # clamp
        xc = min(max(xc, 0.0), 1.0)
        yc = min(max(yc, 0.0), 1.0)
        ww = min(max(ww, 0.0), 1.0)
        hh = min(max(hh, 0.0), 1.0)
        yolo_lines.append(f"{cls_id} {xc:.6f} {yc:.6f} {ww:.6f} {hh:.6f}")

    if yolo_lines:
        stats["with_boxes"] += 1

    # Write label file (empty file is ok for negatives)
    with open(dst_lbl, "w") as f:
        if yolo_lines:
            f.write("\n".join(yolo_lines) + "\n")

    # Copy image (preserve extension)
    if src_img and os.path.exists(src_img):
        ext = pathlib.Path(src_img).suffix.lower() or ".jpg"
        dst_img = os.path.join(OUT_IMG, stem + ext)
        if not os.path.exists(dst_img):
            shutil.copy2(src_img, dst_img)
            stats["images_copied"] += 1
    else:
        stats["missing_images"] += 1
        if not src_img:
            print(f"⚠️  No source image found for {stem} (img_ref={img_ref})")
        else:
            print(f"⚠️  Missing image file: {src_img}")

    num_ok += 1

# Summarize classes
names = [None]*len(class_to_id)
for k,v in class_to_id.items():
    names[v] = k

print("Converted items:", num_ok)
print("Class mapping:", {v:k for k,v in class_to_id.items()})
print("Paste this into data.yaml as names:", names)
print("Stats:", stats)

