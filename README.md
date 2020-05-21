[![CircleCI](https://circleci.com/gh/kordaniel/Ohtuprojekti-kesa2020.svg?style=svg)](https://circleci.com/gh/kordaniel/Ohtuprojekti-kesa2020) [![codecov](https://codecov.io/gh/kordaniel/Ohtuprojekti-kesa2020/branch/master/graph/badge.svg)](https://codecov.io/gh/kordaniel/Ohtuprojekti-kesa2020) [![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)

## Definition of done
  * All task features are implemented
  * Code is formatted with yapf
  * All non trivial code should be tested
  * All code should be well documented, at least with a docstring for every block
  * All code should use type hinting

## Commands for virtual environment
[pipenv](https://github.com/pypa/pipenv) is used for managing dependencies

#### Creating virtual environment for development
`pipenv install --dev`

#### Activate virtual environment
`pipenv shell`
or `pipenv run <command>` to run a single command in the environment
#### More
[pipenv usage](https://github.com/pypa/pipenv#-usage)

## Start main.py
When the application is run, it checks for various needed directories and if they are not found, creates them. Therefore it should always be run from the root directory.
`python src/main.py`

## Run OpenCV face detection in main
from video:
`pipenv run -- python main.py -v`
from image file:
`pipenv run -- python main.py -i` + number 0-4
Exit with ctrl + c

# Arguments for live_detect.py

## Define tensorflow model (default: detect.tflite)

`-m modelname.tflite`

## Define labelmap for model (default: labelmap.txt)

`-l labelmap.txt`

## Define a webcamera's ID (default: 0)
`-c camera_id`

# Testing
Run all tests from root folder:
```console
python -m unittest
```
Add -v flag after `unittest` for verbose output  

Run tests and generate coverage report
```console
coverage run -m unittest discover
coverage html
```

# Create test

Create test file named `test_<module_to_test>.py` and empty file called `__init__.py` side by side in `tests` folder or any of it's subfolders.

# Formatting

Run [yapf](https://github.com/google/yapf/) before commits `pipenv run yapf -ri .`  
Use `pipenv run yapf -rd .` to print diff of changes if needed.

[pep8 style guide](https://www.python.org/dev/peps/pep-0008/)
