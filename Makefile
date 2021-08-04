#acceptable build_types: Release/Debug/Profile
build_type=Release
# build_type=Debug

.SILENT:

all: build build/CMakeLists.txt.copy graph_navigation/bin/navigation
	$(info Build_type is [${build_type}])
	$(MAKE) --no-print-directory -C build

clean:
	rm -rf build bin lib
	cd graph_navigation && rm -rf build bin lib

build/CMakeLists.txt.copy: build CMakeLists.txt Makefile manifest.xml
	cd build && cmake -DCMAKE_BUILD_TYPE=$(build_type) ..
	cp CMakeLists.txt build/CMakeLists.txt.copy

build:
	mkdir -p build

graph_navigation/bin/navigation:
	cd graph_navigation && $(MAKE)