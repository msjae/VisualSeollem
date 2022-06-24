FROM slam:base

ARG BRANCH=development
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && apt-get upgrade -y

RUN useradd -m user && yes password | passwd user

RUN echo "== Start Debug build == " && \
cd /slam/VisualSeollem && \
git remote update && \
git fetch --all && \
git checkout ci && \
git pull && \
git branch && \
mkdir build_debug && cd build_debug && \
sh build.sh

RUN echo "== KITTI Examples == " && \
cd /slam/VisualSeollem && \
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTI03.yaml ./dataset/sequences/04

