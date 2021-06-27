FROM ubuntu:20.04

COPY /home/vishnu/multiplekalman/cariad  /usr/src/cariad

WORKDIR /usr/src/cariad


RUN apt-get update && apt-get -y install cmake protobuf-compiler
RUN cd ./Server/build/ && cmake .. &&  make 
RUN cd ./Client/build/ && cmake.. && make

CMD ./Server/build/SensorServer  && ./Client/build/ObjectFusion




