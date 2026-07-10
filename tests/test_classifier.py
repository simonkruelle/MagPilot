#!/usr/bin/env python3
"""Smoke test the EasyOCR character classifier without sensor hardware."""

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
