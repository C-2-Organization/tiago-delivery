import cv2

def apply_roi(img, use_roi: bool, roi_x1: float, roi_y1: float, roi_x2: float, roi_y2: float):
    if not use_roi:
        return img
    h, w = img.shape[:2]
    x1 = int(roi_x1 * w)
    y1 = int(roi_y1 * h)
    x2 = int(roi_x2 * w)
    y2 = int(roi_y2 * h)
    return img[y1:y2, x1:x2]

def preprocess(img, use_preprocess: bool, use_clahe: bool, clahe_clip: float, clahe_tile_grid_size: int, denoise_blur_k: int):
    if not use_preprocess:
        return img

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if use_clahe:
        clahe = cv2.createCLAHE(
            clipLimit=clahe_clip,
            tileGridSize=(clahe_tile_grid_size, clahe_tile_grid_size)
        )
        gray = clahe.apply(gray)

    k = max(1, int(denoise_blur_k))
    if k % 2 == 0:
        k += 1
    if k > 1:
        gray = cv2.GaussianBlur(gray, (k, k), 0)

    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
