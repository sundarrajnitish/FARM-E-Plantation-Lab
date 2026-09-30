# Host unit tests for the controller cores (protocol, drive, link).
CXX ?= g++
CXXFLAGS = -std=c++17 -O1 -g -Wall -Wextra -Werror -Wconversion -Wno-sign-conversion -fsanitize=address,undefined -fno-sanitize-recover=all
SRC = test_core.cpp ../FarmeDrive/src/farme_proto.cpp ../FarmeDrive/src/drive_core.cpp ../FarmeLink/src/link_core.cpp

run: build/test_core
	./build/test_core
	@cmp -s ../FarmeDrive/src/farme_proto.h ../FarmeLink/src/farme_proto.h && cmp -s ../FarmeDrive/src/farme_proto.cpp ../FarmeLink/src/farme_proto.cpp || (echo "farme_proto differs between the two sketches" && false)

build/test_core: $(SRC) $(wildcard ../FarmeDrive/src/*.h ../FarmeLink/src/*.h)
	@mkdir -p build
	$(CXX) $(CXXFLAGS) -o $@ $(SRC)

clean:
	rm -rf build
.PHONY: run clean
