FROM atlassian/default-image:4

RUN apt-get update && \
    apt-get install -y git curl zip unzip tar cmake build-essential libxmu-dev libxi-dev libgl-dev libxinerama-dev libxcursor-dev xorg-dev libglu1-mesa-dev pkg-config libboost-all-dev liblpsolve55-dev lp-solve libeigen3-dev && \
    git clone https://github.com/Microsoft/vcpkg.git /vcpkg && \
    sh /vcpkg/bootstrap-vcpkg.sh && \
    export CMAKE_MAKE_PROGRAM="/usr/bin/make" && \
    export CMAKE_CXX_COMPILER="/usr/bin/g++"

WORKDIR /app

COPY . /app

RUN ls && mkdir build && \
    cmake -B build/ -S . -DCMAKE_TOOLCHAIN_FILE="/vcpkg/scripts/buildsystems/vcpkg.cmake" && \
    make -C build/ -j8

WORKDIR /app/build

CMD ["ctest", "--verbose"]
