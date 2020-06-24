import cv2 as cv
import pyzbar.pyzbar as zbar
from pyzbar.pyzbar import ZBarSymbol


def detect(img):
    print('zbar')
    height = img.shape[0]
    width = img.shape[1]

    results = zbar.decode(img, symbols=[ZBarSymbol.QRCODE])
    observations = []
    for r in results:
        observations.append({
            "data":
            r.data,
            "bbox": {
                "top": r.rect.top / height,
                "right": (r.rect.left + r.rect.width) / width,
                "bottom": (r.rect.top + r.rect.height) / height,
                "left": r.rect.left / width
            },
            "polygon": [{
                "x": r.polygon[0].x / width,
                "y": r.polygon[0].y / height
            }, {
                "x": r.polygon[1].x / width,
                "y": r.polygon[1].y / height
            }, {
                "x": r.polygon[2].x / width,
                "y": r.polygon[2].y / height
            }, {
                "x": r.polygon[3].x / width,
                "y": r.polygon[3].y / height
            }]
        })
    return observations
