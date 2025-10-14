FROM ubuntu:22.04

# 초기 프로그램 설치
RUN apt-get update --fix-missing
RUN apt install sudo
RUN apt install apt-utils -y
RUN apt update

RUN ln -snf /usr/share/zoneinfo/$CONTAINER_TIMEZONE /etc/localtime && echo $CONTAINER_TIMEZONE > /etc/timezone

RUN sudo apt install apt-utils nano -y
RUN sudo apt install -y python3
RUN sudo apt install -y python3-yaml
RUN sudo apt install -y liblz4-tool cpio ssh
RUN sudo apt install -y make
RUN sudo apt install -y flex bison libssl-dev python3 nano minicom zstd
RUN sudo apt install -y wget xz-utils xxd flex bison build-essential bc flex bison
RUN sudo apt install -y qemu-user-static
RUN sudo apt install -y gcc-arm-linux-gnueabihf
RUN sudo apt install -y libxml2-utils
RUN sudo apt install -y dosfstools
RUN sudo apt install -y libssl-dev
RUN sudo apt install -y device-tree-compiler
RUN sudo apt install -y git python3 python3-setuptools python3-pip
RUN sudo apt install -y gnupg ca-certificates
RUN sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF
RUN echo "deb https://download.mono-project.com/repo/ubuntu stable-focal main" | sudo tee /etc/apt/sources.list.d/mono-official-stable.list
RUN sudo apt update
RUN sudo apt install -y mono-devel
RUN sudo apt install -y netcat
RUN sudo apt install -y libxml2-utils simg2img abootimg sshpass kmod lbzip2
RUN sudo apt install python3-sphinx -y
RUN sudo apt install -y binfmt-support
RUN sudo apt install -y rsync
RUN sudo dpkg-reconfigure qemu-user-static
RUN sudo apt install qemu-user-static

# robotis 사용자 추가
RUN adduser --disabled-password --gecos "" robotis
RUN usermod -g sudo robotis
RUN passwd -d robotis

USER robotis

WORKDIR /home/robotis

