[![CircleCI](https://circleci.com/gh/Konenako/Ohtuprojekti-kesa2020.svg?style=svg)](https://circleci.com/gh/Konenako/Ohtuprojekti-kesa2020) [![codecov](https://codecov.io/gh/Konenako/Ohtuprojekti-kesa2020/branch/master/graph/badge.svg)](https://codecov.io/gh/Konenako/Ohtuprojekti-kesa2020) [![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)

## Documentation

[Backlog](https://github.com/Konenako/Ohtuprojekti-kesa2020/projects)

## Running Docker with script file

`./docker_runner.sh`

Kills containers that are open, builds and runs containers and attaches new terminals to them.


## Commands for virtual environment
[poetry](https://github.com/python-poetry/poetry) is used for managing dependencies

#### Creating virtual environment for development
`poetry install`

#### Activate virtual environment
`poetry shell`
or `poetry run <command>` to run a single command in the environment

#### Additional dependencies
Some libraries must be separately installed, current list:
 * zbar/libzbar0 

# Formatting

Run [yapf](https://github.com/google/yapf/) before commits `poetry run yapf -ri .`  
Use `poetry run yapf -rd .` to print diff of changes if needed.

[pep8 style guide](https://www.python.org/dev/peps/pep-0008/)
