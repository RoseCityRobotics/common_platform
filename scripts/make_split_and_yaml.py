import os, random, shutil, yaml, glob, pathlib

NAMES = ["Purple ball", "Green ball"]
IMAGES = "data/images"
LABELS = "data/labels"
OUT = "data"  # will create data/train, data/val
TRAIN_RATIO = 0.8

# Fill these manually if you already know class names.
# If empty, we try to infer max class id and make placeholder names.
NAMES = []  # e.g., ["person","bike","car"]

random.seed(0)
paths = sorted(glob.glob(os.path.join(IMAGES, "*.*")))
random.shuffle(paths)

n_train = int(len(paths) * TRAIN_RATIO)
train_imgs = set(paths[:n_train])

def pair_label(img):
    stem = pathlib.Path(img).stem
    return os.path.join(LABELS, stem + ".txt")

for split in ("train","val"):
    os.makedirs(f"data/{split}/images", exist_ok=True)
    os.makedirs(f"data/{split}/labels", exist_ok=True)

for img in paths:
    lbl = pair_label(img)
    split = "train" if img in train_imgs else "val"
    shutil.copy2(img, f"data/{split}/images/{os.path.basename(img)}")
    if os.path.exists(lbl):
        shutil.copy2(lbl, f"data/{split}/labels/{os.path.basename(lbl)}")
    else:
        # Create empty label if missing, to keep counts aligned
        open(f"data/{split}/labels/{pathlib.Path(img).stem}.txt","w").close()

# Infer class count if names not provided
nc = 0
for lbl in glob.glob("data/*/labels/*.txt"):
    with open(lbl) as f:
        for line in f:
            if not line.strip(): continue
            cid = int(line.split()[0])
            nc = max(nc, cid+1)

if not NAMES:
    NAMES = [f"class_{i}" for i in range(nc)]

data_yaml = {
    "path": "data",
    "train": "train/images",
    "val":   "val/images",
    "nc": len(NAMES),
    "names": NAMES
}

with open("data.yaml","w") as f:
    yaml.safe_dump(data_yaml, f, sort_keys=False)

print("Wrote data.yaml with", len(NAMES), "classes")

