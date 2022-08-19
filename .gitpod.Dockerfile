FROM gitpod/workspace-full

RUN curl --output phase.deb -L https://github.com/i3drobotics/phase/releases/download/v0.1.2-12/phase-v0.1.2-12-ubuntu-20.04-x86_64.deb && \
    sudo apt update && \
    sudo apt install -y ./phase.deb && \
    sudo rm -rf ./phase.deb && \
    sudo apt-get install -y patchelf doxygen
