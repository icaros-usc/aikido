#!/usr/bin/env bash

set -ex

# Install OMPL package from PRL PPA
if [ $(lsb_release -sc) = "xenial" ]; then
  sudo apt-add-repository -y ppa:personalrobotics/ppa
  sudo apt update
  sudo apt install -y libompl-dev # OMPL (>= 1.2.1)
fi


cd "${HOME}/workspace"
cp -r "${TRAVIS_BUILD_DIR}" src
./scripts/internal-distro.py --workspace=src distribution.yml --repository "${REPOSITORY}" ${REQUIRED_ONLY}

if [ $BUILD_NAME = TRUSTY_FULL_DEBUG ]; then
  sudo apt-get install clang-format-3.8
fi
