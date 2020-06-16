#!/usr/bin/env python3.7
## Script to be run inside virtual environment to check that all needed libraries are available.
import sys
from pkg_resources import parse_version

# Certain combination of tensorflow and numpy produces a long list of warnings
# Silence them all!!!! This needs to be done before importing tensorflow..!
import warnings
warnings.filterwarnings('ignore', category=FutureWarning)

# Dict with all the required libraries and their minimum supported version.
libraries = {
    'cv2':          '4.1.0',
    'tensorflow':   '1.14.0',
    'numpy':        '1.18',
    'pyzbar':       '0.1.7'
}


def main():
    # Boolean that is true if all dependencies are fullfilled.
    all_required = True

    for library, required in libraries.items():
        # Checking versions
        try:
            lib = __import__(library)
        except Exception:
            lib = None
    
        if lib:
            version = lib.__version__
            if parse_version(version) < parse_version(required):
                print(f'You have version {parse_version(version)} of {library}, while the minimum required is {required}.')
                all_required = False
            #else:
            #    # Print found usable libraries and their version
            #    print(library, version, 'Good!')
        else:
            print(library, 'was not found, please install it.')
            all_required = False

    # Exit with error code if we have unfulfilled dependencies.
    if not all_required:
        print('You have unmet library dependencies, please fix them before continuing.')
        sys.exit(1)
    else:
        print('You have all the correct libraries installed!')


if __name__ == '__main__':
    main()
