# Uses gcc8 for glibc 2.28
ARG FROM=conanio/gcc10:latest
FROM ${FROM} AS lanelet2_conan_deps

ENV DEBIAN_FRONTEND noninteractive

# install requirements for python installation
RUN sudo -E apt-get update \
  && sudo -E apt-get install -y libssl-dev zlib1g-dev \
  libbz2-dev libreadline-dev libsqlite3-dev curl \
  libncursesw5-dev xz-utils libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev

# setup python environment
ARG PY_VERSION=3.12
RUN pyenv update \
  && pyenv install -s $PY_VERSION \
  && pyenv global $PY_VERSION
RUN pip install "conan~=2.8.0" catkin_pkg "numpy<2.0" wheel auditwheel cmake

# install patchelf
RUN wget https://github.com/NixOS/patchelf/releases/download/0.17.2/patchelf-0.17.2-x86_64.tar.gz \
  && tar -xzf patchelf-0.17.2-x86_64.tar.gz \
  && sudo -E ln -s $HOME/bin/patchelf /bin/patchelf \
  && patchelf --version


FROM lanelet2_conan_deps as lanelet2_conan_src

# checkout code
RUN mkdir src
WORKDIR /home/conan/src
COPY --chown=1000:1001 . lanelet2/

FROM lanelet2_conan_src as lanelet2_conan

# compile
ARG CONAN_ARGS=""
ARG PLATFORM="manylinux_2_31_x86_64"
WORKDIR /home/conan/src/lanelet2
RUN conan profile detect \
  && conan create . --format=json --build=missing -o "&:build_wheel=True" -o "&:platform=${PLATFORM}" ${CONAN_ARGS} > conaninfo.json

# obtain wheel
FROM lanelet2_conan as lanelet2_conan_with_pip_wheel

SHELL ["/bin/bash", "-c"]
WORKDIR /home/conan
RUN LANELET2_PACKAGE_DIR=$(python3 -c "import json; f=open('src/lanelet2/conaninfo.json'); data=json.load(f); ll2=next(d for d in data['graph']['nodes'].values() if 'lanelet2' in d['ref']); print(f'{ll2[\"package_folder\"]}/wheel')") \
  && cp -r $LANELET2_PACKAGE_DIR dist

# to extract the wheel manually run:
# $ docker run --rm -v /path/to/some/local/folder:/dist <image> /bin/bash -c 'sudo cp dist/lanelet2-<...>.whl /dist/.'