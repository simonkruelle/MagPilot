#!/usr/bin/env python3
"""Pretrained EasyOCR inference for magnetometer digit projection images."""

from collections import deque

import numpy as np
from PIL import Image, ImageOps


class DigitClassifier:
    def __init__(self, languages=None, gpu=False, upscale=4):
        try:
            import easyocr
        except ImportError as exc:
            raise ImportError(
                "EasyOCR is not installed. Install dependencies with "
                "`pip install -r requirements.txt`."
            ) from exc

        self.languages = languages or ['en']
        self.gpu = gpu
        self.upscale = upscale
        self.reader = easyocr.Reader(self.languages, gpu=self.gpu, verbose=False)

    def predict(self, image: np.ndarray) -> dict:
        ocr_image = self._prepare_image(image)
        height, width = ocr_image.shape
        results = self.reader.recognize(
            ocr_image,
            horizontal_list=[[0, width, 0, height]],
            free_list=[],
            allowlist='0123456789',
            detail=1,
            paragraph=False,
            batch_size=1,
            decoder='greedy',
        )

        label, confidence = self._best_digit(results)
        probabilities = self._probabilities(label, confidence)
        return {
            'label': label,
            'confidence': float(confidence),
            'probabilities': probabilities,
            'raw_results': results,
        }

    def predict_smoothed(self, image: np.ndarray, history: list, window_size: int = 5) -> dict:
        result = self.predict(image)
        probabilities = result['probabilities']

        if isinstance(history, deque):
            history.append(probabilities)
            recent = list(history)[-window_size:]
        else:
            history.append(probabilities)
            del history[:-window_size]
            recent = history

        smoothed = np.mean(np.stack(recent, axis=0), axis=0)
        label = int(np.argmax(smoothed))
        confidence = float(smoothed[label])
        return {
            'label': label,
            'confidence': confidence,
            'probabilities': smoothed,
            'raw_results': result.get('raw_results', []),
        }

    def _prepare_image(self, image: np.ndarray) -> np.ndarray:
        image = np.asarray(image, dtype=np.float32)
        if image.ndim != 2:
            raise ValueError(f"Expected a 2D grayscale image, got shape {image.shape}")

        image = np.clip(image, 0.0, 1.0)
        uint8_image = (image * 255).astype(np.uint8)
        pil_image = Image.fromarray(uint8_image, mode='L')

        if self.upscale > 1:
            width, height = pil_image.size
            pil_image = pil_image.resize(
                (width * self.upscale, height * self.upscale),
                Image.Resampling.BICUBIC,
            )

        pil_image = ImageOps.autocontrast(pil_image)
        return np.asarray(pil_image)

    @staticmethod
    def _best_digit(results):
        best_digit = None
        best_confidence = 0.0

        for _, text, confidence in results:
            digits = [char for char in str(text) if char.isdigit()]
            if not digits:
                continue
            confidence = float(confidence)
            if confidence > best_confidence:
                best_digit = int(digits[0])
                best_confidence = confidence

        if best_digit is None:
            return 0, 0.0
        return best_digit, best_confidence

    @staticmethod
    def _probabilities(label, confidence):
        confidence = float(np.clip(confidence, 0.0, 1.0))
        probabilities = np.full(10, (1.0 - confidence) / 9.0, dtype=np.float32)
        probabilities[int(label)] = confidence
        return probabilities
