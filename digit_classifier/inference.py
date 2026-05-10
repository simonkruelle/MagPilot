#!/usr/bin/env python3
"""Pretrained EasyOCR inference for magnetometer character projection images."""

from collections import deque

import numpy as np
from PIL import Image, ImageOps


LABEL_PRESETS = {
    'digits': '0123456789',
    'letters': 'ABCDEFGHIJKLMNOPQRSTUVWXYZ',
    'alphanumeric': '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ',
}


class DigitClassifier:
    def __init__(self, languages=None, gpu=False, upscale=4, labels='alphanumeric', cpu_threads=1):
        try:
            import easyocr
        except ImportError as exc:
            raise ImportError(
                "EasyOCR is not installed. Install dependencies with "
                "`pip install -r requirements.txt`."
            ) from exc

        self._limit_cpu_threads(cpu_threads)
        self.languages = languages or ['en']
        self.gpu = gpu
        self.upscale = upscale
        self.set_labels(labels)
        self.reader = easyocr.Reader(self.languages, gpu=self.gpu, verbose=False)

    def set_labels(self, labels):
        self.labels = self._resolve_labels(labels)
        self.allowlist = ''.join(self.labels)
        self.label_to_index = {label: index for index, label in enumerate(self.labels)}

    @staticmethod
    def _limit_cpu_threads(cpu_threads):
        if cpu_threads is None or int(cpu_threads) <= 0:
            return

        cpu_threads = int(cpu_threads)
        try:
            import torch

            torch.set_num_threads(cpu_threads)
            try:
                torch.set_num_interop_threads(cpu_threads)
            except RuntimeError:
                pass
        except Exception:
            pass

        try:
            import cv2

            cv2.setNumThreads(cpu_threads)
        except Exception:
            pass

    def predict(self, image: np.ndarray) -> dict:
        ocr_image = self._prepare_image(image)
        height, width = ocr_image.shape
        results = self.reader.recognize(
            ocr_image,
            horizontal_list=[[0, width, 0, height]],
            free_list=[],
            allowlist=self.allowlist,
            detail=1,
            paragraph=False,
            batch_size=1,
            decoder='greedy',
        )

        probabilities = self._scores_from_results(results)
        label, confidence = self._best_label(probabilities)
        return {
            'label': label,
            'confidence': float(confidence),
            'probabilities': probabilities,
            'labels': self.labels,
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
        label, confidence = self._best_label(smoothed)
        return {
            'label': label,
            'confidence': confidence,
            'probabilities': smoothed,
            'labels': self.labels,
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
    def _resolve_labels(labels):
        if labels is None:
            labels = 'alphanumeric'
        if isinstance(labels, str):
            labels = LABEL_PRESETS.get(labels, labels)

        resolved = []
        for label in labels:
            label = str(label).strip().upper()
            if len(label) != 1:
                raise ValueError(f"Classifier labels must be single characters, got {label!r}")
            if label and label not in resolved:
                resolved.append(label)

        if not resolved:
            raise ValueError("Classifier must have at least one label")
        return tuple(resolved)

    def _scores_from_results(self, results):
        scores = np.zeros(len(self.labels), dtype=np.float32)

        for _, text, confidence in results:
            confidence = float(np.clip(confidence, 0.0, 1.0))
            for char in str(text).upper():
                index = self.label_to_index.get(char)
                if index is not None:
                    scores[index] = max(scores[index], confidence)

        return scores

    def _best_label(self, scores):
        if len(scores) == 0:
            return None, 0.0

        index = int(np.argmax(scores))
        confidence = float(scores[index])
        if confidence <= 0.0:
            return None, 0.0
        return self.labels[index], confidence
