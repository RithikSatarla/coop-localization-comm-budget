"""Download and extract UTIAS MR.CLAM datasets 1-4 into data/.

Official dataset page: https://asrl.utias.utoronto.ca/datasets/mrclam/index.html
Archives are served over FTP from ftp://asrl3.utias.utoronto.ca/MRCLAM/ as
MRCLAM1.zip ... MRCLAM9.zip (note: the page text says "Dataset N" but the
files on the FTP server are named MRCLAM<N>.zip). Each archive extracts to a
folder named MRCLAM_Dataset<N>, except dataset 4 which extracts to
MRSLAM_Dataset4 (a naming quirk of the original archive).

Each dataset folder contains 17 files:
  Barcodes.dat              subject #, barcode #
  Landmark_Groundtruth.dat  subject #, x [m], y [m], x std-dev [m], y std-dev [m]
  Robot<N>_Groundtruth.dat  time [s], x [m], y [m], orientation [rad]
  Robot<N>_Odometry.dat     time [s], forward velocity [m/s], angular velocity [rad/s]
  Robot<N>_Measurement.dat  time [s], measured subject barcode #, range [m], bearing [rad]
Robots are subjects 1-5, landmarks are subjects 6-20.

Usage:
    python scripts/download_mrclam.py            # datasets 1-4
    python scripts/download_mrclam.py 1 2        # a subset

The download uses urllib (FTP support is in the standard library), so no
external tools are needed. If the FTP server is unreachable, the script fails
loudly rather than silently substituting a mirror.
"""
from __future__ import annotations

import shutil
import sys
import urllib.request
import zipfile
from pathlib import Path

FTP_BASE = "ftp://asrl3.utias.utoronto.ca/MRCLAM/"
# Compressed sizes listed by the FTP server (bytes); used as an integrity check.
EXPECTED_SIZES = {1: 6118042, 2: 7670940, 3: 8091370, 4: 6300417, 9: 11328005}
ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "data"


def dataset_folder(data_dir: Path, k: int) -> Path | None:
    """Return the extracted folder for dataset k, tolerating the MRSLAM_ prefix."""
    for name in (f"MRCLAM_Dataset{k}", f"MRSLAM_Dataset{k}"):
        p = data_dir / name
        if p.is_dir():
            return p
    return None


def download(k: int, data_dir: Path) -> Path:
    data_dir.mkdir(parents=True, exist_ok=True)
    url = f"{FTP_BASE}MRCLAM{k}.zip"
    dest = data_dir / f"MRCLAM{k}.zip"
    expected = EXPECTED_SIZES.get(k)
    if dest.exists() and (expected is None or dest.stat().st_size == expected):
        print(f"[dataset {k}] archive already present: {dest}")
        return dest
    print(f"[dataset {k}] downloading {url}")
    tmp = dest.with_suffix(".zip.part")
    with urllib.request.urlopen(url, timeout=120) as resp, open(tmp, "wb") as fh:
        shutil.copyfileobj(resp, fh)
    size = tmp.stat().st_size
    if expected is not None and size != expected:
        tmp.unlink(missing_ok=True)
        raise RuntimeError(
            f"dataset {k}: downloaded {size} bytes, expected {expected}; aborting"
        )
    tmp.replace(dest)
    print(f"[dataset {k}] saved {size} bytes")
    return dest


def extract(k: int, archive: Path, data_dir: Path) -> Path:
    folder = dataset_folder(data_dir, k)
    if folder is not None and len(list(folder.glob("*.dat"))) == 17:
        print(f"[dataset {k}] already extracted: {folder}")
        return folder
    with zipfile.ZipFile(archive) as zf:
        zf.extractall(data_dir)
    folder = dataset_folder(data_dir, k)
    if folder is None:
        raise RuntimeError(f"dataset {k}: extraction did not produce a dataset folder")
    n = len(list(folder.glob("*.dat")))
    if n != 17:
        raise RuntimeError(f"dataset {k}: expected 17 .dat files, found {n} in {folder}")
    print(f"[dataset {k}] extracted 17 files to {folder}")
    return folder


def main(argv: list[str]) -> int:
    ids = [int(a) for a in argv[1:]] or [1, 2, 3, 4]
    for k in ids:
        archive = download(k, DATA_DIR)
        extract(k, archive, DATA_DIR)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
