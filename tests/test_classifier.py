#!/usr/bin/env python3
"""Smoke test the EasyOCR character classifier without sensor hardware."""

import os as _os, sys as _sys
_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
for _p in (_ROOT, _os.path.join(_ROOT, 'tools')):
    if _p not in _sys.path:
        _sys.path.insert(0, _p)

import numpy as np

from digit_classifier.inference import DigitClassifier


def main():
    classifier = DigitClassifier(gpu=False, labels="alphanumeric")

    # White background, dark stroke, matching pose_to_digit_image().
    image = np.ones((64, 64), dtype=np.float32)
    image[15:50, 30:34] = 0.1

    result = classifier.predict(image)
    print(f"label={result['label']} confidence={result['confidence']:.3f}")


if __name__ == "__main__":
    main()
