#!/usr/bin/env python3
"""Run EasyOCR character inference over saved projection PNG samples."""

import argparse
import os
import re

import numpy as np
from PIL import Image

from digit_classifier.inference import DigitClassifier


LABEL_RE = re.compile(r"(?:digit_(\d)|letter_([A-Za-z]))")


def load_grayscale(path):
    image = Image.open(path).convert("L").resize((64, 64))
    return np.asarray(image, dtype=np.float32) / 255.0


def expected_label(path):
    match = LABEL_RE.search(os.path.basename(path))
    if not match:
        return None
    return (match.group(1) or match.group(2)).upper()


def main():
    parser = argparse.ArgumentParser(description="Batch-test EasyOCR on saved projection PNGs.")
    parser.add_argument("image_dir", nargs="?", default="digit_images")
    parser.add_argument("--gpu", action="store_true", help="Use EasyOCR GPU mode if available")
    parser.add_argument(
        "--labels",
        default="alphanumeric",
        help="EasyOCR label set: digits, letters, alphanumeric, or a custom subset such as ABCX0123",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.image_dir):
        print(f"No image directory found at {args.image_dir}")
        print("Collect a character first with magnetometer_reader.py, or pass a different directory.")
        return

    paths = [
        os.path.join(args.image_dir, name)
        for name in sorted(os.listdir(args.image_dir))
        if name.lower().endswith(".png")
    ]
    if not paths:
        print(f"No PNG files found in {args.image_dir}")
        return

    classifier = DigitClassifier(gpu=args.gpu, labels=args.labels)

    correct = 0
    labeled = 0
    for path in paths:
        image = load_grayscale(path)
        result = classifier.predict(image)
        expected = expected_label(path)
        is_correct = expected is not None and result["label"] == expected
        if expected is not None:
            labeled += 1
            correct += int(is_correct)

        expected_text = "?" if expected is None else str(expected)
        marker = "OK" if is_correct else "--"
        print(
            f"{marker} {os.path.basename(path)} "
            f"expected={expected_text} predicted={result['label']} "
            f"confidence={result['confidence']:.3f}"
        )

    if labeled:
        print(f"\nAccuracy on labeled filenames: {correct}/{labeled} = {correct / labeled * 100:.1f}%")


if __name__ == "__main__":
    main()
