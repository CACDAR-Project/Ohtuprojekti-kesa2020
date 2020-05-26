import numpy as np

from pyzbar import pyzbar
from pyzbar.pyzbar import ZBarSymbol

from typing import Tuple, List


# Pyzbar supported symbols(types of codes):
# from pyzbar.pyzbar import ZBarSymbol
# print([s for s in ZBarSymbol])
# ZBarSymbol.NONE
# ZBarSymbol.PARTIAL
# ZBarSymbol.EAN2
# ZBarSymbol.EAN5
# ZBarSymbol.EAN8
# ZBarSymbol.UPCE
# ZBarSymbol.ISBN10
# ZBarSymbol.UPCA
# ZBarSymbol.EAN13
# ZBarSymbol.ISBN13
# ZBarSymbol.COMPOSITE
# ZBarSymbol.I25
# ZBarSymbol.DATABAR
# ZBarSymbol.DATABAR_EXP
# ZBarSymbol.CODABAR
# ZBarSymbol.CODE39
# ZBarSymbol.PDF417
# ZBarSymbol.QRCODE
# ZBarSymbol.CODE93
# ZBarSymbol.CODE128
class PyzbarDetector():
    'Class that scans an opencv formatted np.array frame (image) for symbols, using pyzbar-library.'

    def __init__(self, only_qr_codes: bool = True) -> None:
        '''Constructor, accepts one parameter. With default value only scans for QR-codes.
        Set to False to scan for every supported symbol
        '''
        # Recognize only QRcodes
        if only_qr_codes:
            # Only scan for QR-codes
            self.symbols_to_recognize = [ZBarSymbol.QRCODE]
        else:
            # Scan for all supported symbols
            self.symbols_to_recognize = ZBarSymbol

        self.codes = None

    def scan_frame(self, frame: np.ndarray) -> None:
        'Scan a frame for codes'
        # pyzbar.decode() also supports PIL.Image objects if needed.
        self.codes = pyzbar.decode(frame, symbols=self.symbols_to_recognize)

    def get_codes(self) -> List[pyzbar.Decoded]:
        'Return a list containing all the codes recognized as pyzbar.Decoded objects'
        return self.codes

    def get_rectangles_coords(
            self) -> Tuple[Tuple[Tuple[int, int], Tuple[int, int]]]:
        '''Returns a 3d tuple containing one tuple for every code coords in the form that
        opencv can draw rectangles: (((xtopleft,ytopleft),(xbottomright,ybottomright)), ((..),(..)). ..)
        '''
        return tuple(
            map(
                lambda code: ((code.rect.left, code.rect.top),
                              (code.rect.left + code.rect.width, code.rect.top
                               + code.rect.height)), self.codes))

    def get_polygons(self) -> np.array:
        '''
        Returns an np.array containing points for every polygon that is found in the frame.
        The array is formatted so that opencv can draw it.
        '''
        #points = list()
        #for code in self.codes:
        #    polygon = list()
        #    for p in code.polygon:
        #        polygon.append((p.x, p.y))
        #    points.append(np.array(polygon, np.int32).reshape(-1,1,2))
        #return np.array(points, np.int32)

        # The same algorithm as an onliner, perhaps faster thanks to tuples vs. lists?
        # Not easy to understad though, so the original code is left as reference
        return np.array(
            tuple(
                map(lambda c: tuple(map(lambda p: (p.x, p.y), c.polygon)),
                    self.codes)), np.int32)

    def get_texts(self) -> Tuple[Tuple[str, Tuple[int, int]]]:
        '''Returns an tuple containing a string and coords for every code that is found in
        the frame: ( '(QRCODE) blabla', (x:int, y:int) )'''
        #codes = list()
        #for code in self.codes:
        #    codes.append((f'({code.type}): {code.data.decode("utf-8")}', (code.rect.left, code.rect.top)))
        #return codes
        return tuple(
            map(
                lambda c: (f'({c.type}): {c.data.decode("utf-8")}',
                           (c.rect.left, c.rect.top)), self.codes))

    def get_detected_codes_amount(self):
        return len(self.codes)