import cv2
import easyocr
import torch

class EasyOCREngine:
    def __init__(self, lang: str = "korean", use_gpu: bool = True, logger=None):
        device = 'cuda' if (use_gpu and torch.cuda.is_available()) else 'cpu'
        if use_gpu and device == 'cpu' and logger is not None:
            logger.warn("use_gpu requested but CUDA not available; falling back to CPU.")

        lang_param = lang.lower() if isinstance(lang, str) else 'korean'
        if 'korean' in lang_param or lang_param in ('ko', 'kr'):
            langs = ['ko']
        elif 'english' in lang_param or lang_param in ('en',):
            langs = ['en']
        else:
            langs = [lang]

        if logger is not None:
            logger.info(f"Initializing EasyOCR (langs={langs}, device={device})...")

        self.reader = easyocr.Reader(lang_list=langs, gpu=(device == 'cuda'))

        if logger is not None:
            logger.info("✅ EasyOCR initialized successfully!")

    def readtext(self, bgr_img, contrast_ths: float, adjust_contrast: float, text_threshold: float, low_text: float):
        rgb = cv2.cvtColor(bgr_img, cv2.COLOR_BGR2RGB)
        return self.reader.readtext(
            rgb,
            detail=1,
            contrast_ths=contrast_ths,
            adjust_contrast=adjust_contrast,
            text_threshold=text_threshold,
            low_text=low_text,
        )

