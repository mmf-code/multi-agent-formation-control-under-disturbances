import argparse
import glob
import shutil
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description="Move unwanted run folders to trash under outputs/simulations/dynamics2d/_trash")
    ap.add_argument("--day", required=True, help="YYYYMMDD")
    ap.add_argument("--keep-label-contains", default="pd_vel,pid_vel,pidf_", help="Comma-separated substrings to keep; others moved")
    ap.add_argument("--root", default=str(Path("outputs/simulations/dynamics2d")))
    args = ap.parse_args()

    root = Path(args.root)
    day_dir = root / args.day
    keep_subs = [s.strip() for s in args.keep_label_contains.split(",") if s.strip()]
    trash_dir = root / "_trash" / args.day
    trash_dir.mkdir(parents=True, exist_ok=True)

    pattern = str(day_dir / "run_*")
    moved = 0
    for p in [Path(x) for x in glob.glob(pattern)]:
        name = p.name
        label = name
        if name.startswith("run_") and "__" in name:
            label = name.split("__", 1)[1]
        keep = any(sub in label for sub in keep_subs)
        if not keep:
            shutil.move(str(p), str(trash_dir / name))
            moved += 1
    print(f"Moved {moved} folders to {trash_dir}")


if __name__ == "__main__":
    main()

