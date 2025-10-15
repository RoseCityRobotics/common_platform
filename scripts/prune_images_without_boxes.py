#!/usr/bin/env python3
import argparse, os, json, glob, re, pathlib, shutil

def resolve_ls_image_path(img_ref: str) -> str:
    """
    Handle Label Studio local-files URLs like:
    /data/local-files/?d=home/rcr/teleop_data/images/0001.jpg  ->  /home/rcr/teleop_data/images/0001.jpg
    Otherwise return as-is.
    """
    if not img_ref:
        return ""
    m = re.search(r"\?d=([^#]+)$", img_ref)
    if m:
        p = m.group(1)
        if not p.startswith("/"):
            p = "/" + p
        return p
    return img_ref

def json_has_rectangle(result_entry: dict) -> bool:
    t = result_entry.get("type")
    if t not in ("rectanglelabels", "labels", "rectangle"):
        return False
    v = result_entry.get("value", {})
    # must have a rectangle and at least one label
    if not ({"x","y","width","height"} <= set(v.keys())):
        return False
    labels = v.get("rectanglelabels") or v.get("labels") or []
    return bool(labels)

def iter_results(task: dict):
    """
    Yield each result entry from common LS schemas:
      A) top-level "result": [...]
      B) inside "annotations"[i]["result"]
    """
    if isinstance(task.get("result"), list):
        for r in task["result"]:
            yield r
        return
    for ann in task.get("annotations") or task.get("completions") or []:
        for r in ann.get("result", []):
            yield r

def collect_positive_stems(ann_dir: str) -> set[str]:
    """Return set of image stems (without extension) that have at least one rectangle box."""
    stems = set()
    files = sorted(glob.glob(os.path.join(ann_dir, "*")))
    for fp in files:
        try:
            with open(fp, "r") as f:
                task = json.load(f)
        except Exception:
            continue

        # find image reference
        img_ref = None
        if "task" in task and isinstance(task["task"], dict):
            img_ref = (task["task"].get("data", {}) or {}).get("image")
        if not img_ref:
            img_ref = (task.get("data", {}) or {}).get("image")
        img_ref = resolve_ls_image_path(img_ref)

        base = os.path.basename(img_ref) if img_ref else ""
        stem = pathlib.Path(base).stem if base else None

        # any rectangles?
        has_box = any(json_has_rectangle(r) for r in iter_results(task))
        if has_box and stem:
            stems.add(stem)
    return stems

def main():
    ap = argparse.ArgumentParser(description="Delete or move images without rectangle annotations.")
    ap.add_argument("--images-dir", default="data/images", help="Directory containing images.")
    ap.add_argument("--annotations-dir", default="annotations", help="Directory of Label Studio JSON files.")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--delete", action="store_true", help="Delete unmatched images.")
    mode.add_argument("--move-to", default=None, help="Move unmatched images into this directory instead of deleting.")
    ap.add_argument("--extensions", default=".jpg,.jpeg,.png,.bmp,.tif,.tiff,.webp", help="Comma-separated list of image extensions to consider.")
    ap.add_argument("--dry-run", action="store_true", default=False, help="Preview only (no changes) when this flag is present.")
    # ap.add_argument("--dry-run", action="store_true", default=True, help="Show actions without changing files (default).")
    args = ap.parse_args()

    exts = {e.strip().lower() if e.strip().startswith(".") else "."+e.strip().lower()
            for e in args.extensions.split(",") if e.strip()}

    positives = collect_positive_stems(args.annotations_dir)

    # gather candidate images
    imgs = []
    for ext in exts:
        imgs.extend(glob.glob(os.path.join(args.images_dir, f"*{ext}")))
    imgs = sorted(imgs)

    to_prune = []
    kept = 0
    for img in imgs:
        stem = pathlib.Path(img).stem
        if stem in positives:
            kept += 1
        else:
            to_prune.append(img)

    print(f"Found {len(positives)} stems with rectangles.")
    print(f"Images scanned: {len(imgs)}  |  keep: {kept}  |  prune: {len(to_prune)}")

    if args.dry_run:
        print("\nDRY RUN: no files will be modified. First 20 to prune:")
        for p in to_prune[:20]:
            print("  -", p)
        print("\nUse --delete to remove or --move-to NEG_DIR to move. You can also pass --dry-run=False to actually execute.")
        return

    # ensure target dir if moving
    if args.move_to:
        os.makedirs(args.move_to, exist_ok=True)

    # apply action
    for p in to_prune:
        if args.move_to:
            dst = os.path.join(args.move_to, os.path.basename(p))
            print(f"MOVE {p} -> {dst}")
            shutil.move(p, dst)
        elif args.delete:
            print(f"DELETE {p}")
            os.remove(p)
        else:
            # default safety: if dry-run is off but no mode chosen, do nothing
            print(f"(No action) {p}")

if __name__ == "__main__":
    main()
