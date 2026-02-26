
#Use a multi-part build so we can have a development setup which mounts the code into the container
#as well as a deployment setup which copies it in.

FROM osrf/ros:noetic-desktop-full AS ros-underwater-sim-dependencies
LABEL authors="branch"

RUN apt update

RUN apt-get install vim gdb -y

#Dependencies
RUN apt-get install libboost-system-dev libboost-filesystem-dev -y
RUN apt-get install libhdf5-dev libcurl4-gnutls-dev wget -y
RUN DEBIAN_FRONTEND=noninteractive apt-get install libboost-random-dev libeigen3-dev libcppunit-dev cmake git -y


#NetCDF
WORKDIR ~
RUN wget https://github.com/Unidata/netcdf-c/archive/refs/tags/v4.7.4.tar.gz
RUN tar -zxvf v4.7.4.tar.gz
WORKDIR ./netcdf-c-4.7.4
RUN mkdir build
WORKDIR ./build
RUN cmake ../ -DCMAKE_BUILD_TYPE=RELEASE
RUN make install
WORKDIR ../..
RUN rm -rf ./netcdf-c-4.7.4
RUN rm v4.7.4.tar.gz


RUN wget https://github.com/Unidata/netcdf-cxx4/archive/refs/tags/v4.3.1.tar.gz
RUN tar -zxvf v4.3.1.tar.gz
WORKDIR ./netcdf-cxx4-4.3.1/
RUN mkdir build
WORKDIR ./build
RUN cmake ../ -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_C_FLAGS=-I/usr/include/hdf5/serial/
RUN make install
WORKDIR ../..
RUN rm -rf ./netcdf-cxx4-4.3.1
RUN rm v4.3.1.tar.gz

RUN apt install python3-pip -y
RUN pip3 install netcdf4

#cpptoml
RUN git clone https://github.com/skystrife/cpptoml.git
WORKDIR ./cpptoml
RUN mkdir build
WORKDIR ./build
RUN cmake ../
RUN make install
WORKDIR ../..
RUN rm -rf cpptoml


#Download and install the ocean model interface
WORKDIR /libs
RUN git clone https://github.com/nasa-jpl/ocean-model-interfaces.git
RUN mkdir /libs/ocean-model-interfaces/build
WORKDIR /libs/ocean-model-interfaces/build
RUN cmake ..
RUN make install

#Fix library path since for some reason /usr/local/lib isn't included in the path
RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

RUN mkdir /ros_workspace
WORKDIR /ros_workspace

#Next build stage which includes the ros underwater sim
FROM ros-underwater-sim-dependencies AS ros-underwater-sim

#Copy the source for ros underwater sim into the ros workspace in the image
RUN mkdir -p /ros_workspace/src/ros-underwater-sim
COPY src /ros_workspace/src/ros-underwater-sim/src

WORKDIR /ros_workspace

#Automatically source the workspace.
RUN echo "source /ros_workspace/devel/setup.bash" >> ~/.bashrc

RUN bash -c "source /opt/ros/noetic/setup.bash && catkin_make"