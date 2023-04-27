# Uses gcc8 for glibc 2.28
FROM conanio/gcc8:latest AS lanelet2_conan_deps

ENV DEBIAN_FRONTEND noninteractive

# install requirements for python installation
RUN sudo -E apt-get update \
  && sudo -E apt-get install -y libssl-dev zlib1g-dev \
    libbz2-dev libreadline-dev libsqlite3-dev curl \
    libncursesw5-dev xz-utils libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev

# setup python environment
ARG PYTHON_VERSION=3.8
RUN pyenv install $PYTHON_VERSION \
  && pyenv global $PYTHON_VERSION
RUN pip install "conan==1.59.0" catkin_pkg numpy wheel auditwheel

# install patchelf
RUN wget https://github.com/NixOS/patchelf/releases/download/0.17.2/patchelf-0.17.2-x86_64.tar.gz \
  && tar -xzf patchelf-0.17.2-x86_64.tar.gz \
  && sudo -E ln -s $HOME/bin/patchelf /bin/patchelf \
  && patchelf --version

# setup conan for python dependencies via bincrafters remote
RUN conan remote add bincrafters https://bincrafters.jfrog.io/artifactory/api/conan/public-conan \
  && conan config set general.revisions_enabled=1


FROM lanelet2_conan_deps as lanelet2_conan_src

# checkout code
RUN mkdir src
WORKDIR /home/conan/src
COPY --chown=conan:conan-1001 . lanelet2/

FROM lanelet2_conan_src as lanelet2_conan

# compile
WORKDIR /home/conan/src/lanelet2
RUN conan source . \
  && conan create . lanelet2/stable --build=missing
RUN export LANELET2_VERSION=$(conan inspect . --raw version) \
  && echo "Lanelet2 version: $LANELET2_VERSION" \
  && conan install lanelet2/$LANELET2_VERSION@lanelet2/stable --build=missing -g virtualenv

FROM lanelet2_conan as lanelet2_conan_with_pip_wheel

SHELL ["/bin/bash", "-c"]
WORKDIR /home/conan
RUN source /home/conan/src/lanelet2/activate.sh \
  && LANELET2_PACKAGE_DIR=$(python -c "import lanelet2; from pathlib import Path; print(Path(lanelet2.__file__).parent)") \
  && cp -r $LANELET2_PACKAGE_DIR .
RUN export LANELET2_VERSION=$(conan inspect ./src/lanelet2 --raw version) \
  && echo "Lanelet2 version: $LANELET2_VERSION" \
  && sed 's/{{ version }}/'"$LANELET2_VERSION"'/' /home/conan/src/lanelet2/lanelet2_python/setup.py.template > /home/conan/setup.py

RUN source /home/conan/src/lanelet2/activate.sh \
  && pip wheel -w broken-dist/ . \
  && auditwheel repair -w dist/ --plat manylinux_2_28_x86_64 broken-dist/*.whl

# to extract the wheel run:
# $ docker run -it -v /path/to/some/local/folder:/dist lanelet2:3.8 /bin/bash
# then inside the container:
# $ sudo cp dist/lanelet2-1.2.0-cp38-cp38-manylinux_2_28_x86_64.whl /dist/.
