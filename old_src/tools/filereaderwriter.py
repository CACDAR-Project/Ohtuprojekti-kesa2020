'''This module provides functions for handling files.'''

import os
import sys

from typing import List
from pathlib import Path


def read_file_lines(file_path: str) -> List[str]:
    '''Reads one file, strips newline endings and returns a list with the rows of the file.
    If the file can't be opened an error message will be printed and an empty list will be returned.
    '''
    if not check_file_exists(file_path):
        print(f'Error while trying to read file: \'{file_path}\'.')
        return []
    with open(file_path, 'r') as f:
        lines = f.read().splitlines()
    return lines


def check_directory_exists(path: str) -> bool:
    '''Checks if the path provided as argument exists and is a directory.'''
    return os.path.isdir(path)


def check_file_exists(path: str) -> bool:
    '''Checks if the path provided as argument exists and is a file.'''
    return os.path.isfile(path)


def create_directory(path: str) -> bool:
    '''Creates a directory with all subdirectories.
    Raises exception and quits the application if directory could not be created.'''
    try:
        os.makedirs(path,
                    exist_ok=True)  # Dont raise an exception if dir exists
    except OSError as e:
        print(f'Error while trying to create directory \'{path}\'.')
        print(e)
        print('Exiting..')
        sys.exit()

    return check_directory_exists(path)


def build_path(*args: str) -> str:
    '''Helper function that takes an arbitrary number of strings and joins them all to build a proper filepath.'''
    return Path('/'.join(args))
