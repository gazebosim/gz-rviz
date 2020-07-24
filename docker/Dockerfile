FROM osrf/ros:foxy-desktop

MAINTAINER Sarathkrishnan Ramesh <sarathkrishnan99@gmail.com>

RUN apt-get update && apt-get install -y qtbase5-dev \
		       libtinyxml2-dev \
		       libprotoc-dev \
		       libprotobuf-dev \
		       qtdeclarative5-dev \
		       qtquickcontrols2-5-dev \
		       qml-module-qtquick2 \
		       qml-module-qtquick-controls \
		       qml-module-qtquick-controls2 \
		       qml-module-qtquick-dialogs \
		       qml-module-qtquick-layouts \
		       qml-module-qt-labs-folderlistmodel \
		       qml-module-qt-labs-settings \
		       qml-module-qtgraphicaleffects \
		       libignition-cmake2-dev \
		       libignition-common3-dev \
		       libignition-math6-dev \
		       libignition-msgs5-dev \
		       libignition-plugin-dev \
		       libignition-tools-dev \
		       libignition-transport8-dev \
		       libogre-1.9-dev \
		       libogre-2.1-dev \
		       libglew-dev \
		       libfreeimage-dev \
		       freeglut3-dev \
		       libxmu-dev \
		       libxi-dev

RUN git clone https://github.com/ignitionrobotics/ign-rendering && \
	cd ign-rendering/ && \
	mkdir build && \
	cd build && \
	cmake .. -DBUILD_TESTING=OFF && \
	make -j4 && \
	make install && \
	cd /

RUN git clone https://github.com/ignitionrobotics/ign-gui && \
	cd ign-gui/ && \
	mkdir build && \
	cd build && \
	cmake .. -DBUILD_TESTING=OFF && \
	make -j4 && \
	make install && \
	cd /
