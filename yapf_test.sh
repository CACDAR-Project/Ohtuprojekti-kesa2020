#!/bin/sh

YAPF_FAIL=$(poetry run yapf -rd .);
echo ${YAPF_FAIL}
if [ "$YAPF_FAIL" != "" ]; then
    echo 'YAPF FAILED!'
    #poetry run yapf -ri .
    #git config --global user.email "app@circleci.com"
    #git config --global user.name "CircleCI"
    #git add .
    #echo "export COMMIT_MESSAGE=\"$(git log --format=oneline -n 1 $CIRCLE_SHA1)\"" >> ~/.bashrc
    #git commit -m "${COMMIT_MESSAGE} formatted by CircleCI" -m "[ci skip]"
    #git push --set-upstream origin ${CIRCLE_BRANCH}
fi