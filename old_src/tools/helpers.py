'''This module provides various helper functions'''

from typing import Union


def str_convert(val: str) -> Union[str, bool, float, int]:
    '''Helper function that takes an argument of type string and if the arguments contains a number,
    that can be converted to int or float, returns the converted number. If the arguments contains
    True or False it will return an boolean. If the argument is not of type string None will be returned.
    Otherwise a new string object containing the same string will be returned.
    '''
    if not isinstance(val, str):
        return None

    try:
        if 'true' == val.lower(): return True
        if 'false' == val.lower(): return False
        return float(val) if '.' in val else int(val)
    except ValueError:
        return str(val)
