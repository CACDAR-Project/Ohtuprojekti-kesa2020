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

## Structure of the application
```bash
application/
├── app.py          - The main loop lies here
├── config
│    └── settings   - All settings go in here
├── __main__.py     - This file starts the application
├── resources       - All resources should be put in here
│   ├── data
│   └── images
└── tests           - All tests live here
```

All the source code should be placed inside the directory `application`. All directories should be made into packages with modules by putting an file `__init__.py` into every directory.  

When importing modules into another modules, use the full package/module path: `application.package<.module>`. For example as in `app.py`: `from application.conf.configuration import Configuration`.

## Running and testing the application
All commands should be run from the base directory.  
After activating the virtual environment run:  

```console
python -m application
```
to run the application. Use the `-h` flag for a help message.
```console
python -m application -h
```

Tests can be run with:  
```
python -m unittest <-v>
```
Use the `-v` flag for verbose output.

Run tests and generate coverage report:
```console
coverage run -m unittest discover
coverage html
```

## Create test

Create an test file named `test_<module_to_test>.py` inside `application/tests/<package>` along with an `__init.py__` for every package.

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


# Formatting

Run [yapf](https://github.com/google/yapf/) before commits `pipenv run yapf -ri .`  
Use `pipenv run yapf -rd .` to print diff of changes if needed.

[pep8 style guide](https://www.python.org/dev/peps/pep-0008/)
