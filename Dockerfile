FROM arm64v8/ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
WORKDIR /ifm
RUN apt-get -y update \
    && apt-get -y upgrade \
    && apt-get -y install build-essential \
                          cmake \
                          wget \
    && wget https://github.com/ifm/ifm3d/releases/download/v1.6.12/ifm3d-ubuntu-22.04-arm64-1.6.12.deb \
    && dpkg -i ifm3d-ubuntu-22.04-arm64-1.6.12.deb
COPY . .
RUN cmake -S Jute_Flat_Angle/project_src -B Jute_Flat_Angle/build \
    && cmake --build Jute_Flat_Angle/build
ENTRYPOINT [ "./Jute_Flat_Angle/build/flat_angles" ]
CMD [ "192.168.254.13", "192.168.254.16", "255.255.255.0", "192.168.254.254", "ram-mechanical.com", "ifm3d-docker-container", "88:a2:9e:02:53:5f" ]
