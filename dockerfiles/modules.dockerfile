FROM slam:base

ARG BRANCH=development
ARG DEBIAN_FRONTEND=noninteractive

RUN useradd -m user && yes password | passwd user
RUN apt-get update -y && apt-get upgrade -y && apt-get install libusb-1.0-0-dev && apt install libspdlog-dev

RUN echo "== Start build == " && \
cd /slam/VisualSeollem && \
git remote update && \
git fetch --all && \
git checkout ci && \
git pull && \
git branch && \
sh build.sh
