FROM slam:base

ARG BRANCH=development
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get upgrade -y

RUN useradd -m user && yes password | passwd user

RUN echo "== Start Debug build == " && \
cd VisualSeollem && \
git remote update && \
git fetch --all && \
git checkout main && \
git pull && \
git branch && \
mkdir build_debug && cd build_debug && \
cmake -DCMAKE_BUILD_TYPE=Debug -GNinja .. && ninja

RUN echo "== Start Release build == " && \
cd VisualSeollem && \
git remote update && \
git fetch --all && \
git checkout main && \
git pull && \
git branch && \
mkdir build_release && cd build_release && \
cmake -DCMAKE_BUILD_TYPE=Release -GNinja .. && ninja

RUN echo "== KITTI Examples == " && \
cd VisualSeollem && \
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTI03.yaml ./dataset/sequences/04

