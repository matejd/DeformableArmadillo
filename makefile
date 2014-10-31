INCLUDE = -I./framework -I./framework/external -I./framework/external/AntTweakBar/include `pkg-config --cflags glew` `pkg-config --cflags libglfw`
LIBS = -lstdc++ -lm -lGLEW -lpthread `pkg-config --libs libglfw` -lGL -L./framework/external/AntTweakBar/lib -lAntTweakBar

TETRA_INCLUDE = -I./framework -I./framework/external -I./framework/external/AntTweakBar/include `pkg-config --cflags glew` `pkg-config --cflags libglfw`
TETRA_LIBS = -lstdc++ -lm -lGLEW -lpthread `pkg-config --libs libglfw` -L./bin/linux -ltetra -lGL -lassimp -L./framework/external/AntTweakBar/lib -lAntTweakBar

# TODO: cereal library needs cxxabi.h, which is included in emscripten, but not visible for some reason. As a temporary workaround I've added the full path to libcxxabi, which seems to work.
EM_INCLUDE = -I./framework -I./framework/external -I/home/matej/Downloads/emsdk_portable/emscripten/master/system/lib/libcxxabi/include/
EM_LIBS = -lGL

linux:
	mkdir -p bin/linux/
	clang -g3 -Wall -Wno-missing-prototypes -std=c++11 -o bin/linux/DeformableArmadillo DeformableArmadillo.cpp SimulationConstraint.cpp Forsyth.cpp framework/*.cpp $(INCLUDE) $(LIBS)

# Tetrahedralization was moved into a shared library due to gigantic compile times (it's based on CGAL).
tetra:
	mkdir -p bin/linux/
	clang -g3 -Wall -c -fPIC -fvisibility=hidden -std=c++11 -o bin/linux/tetra.o Tetrahedralization.cpp -I./framework -I./framework/external
	clang -shared -s -o bin/linux/libtetra.so bin/linux/tetra.o -lCGAL -lboost_thread -lboost_system -lgmp -lmpfr
	rm bin/linux/tetra.o

# If you want tetrahedralization, you need to link with the tetra library above.
linux_with_tetra:
	mkdir -p bin/linux/
	clang -g3 -Wall -Wno-missing-prototypes -std=c++11 -o bin/linux/DeformableArmadillo DeformableArmadillo.cpp SimulationConstraint.cpp Forsyth.cpp framework/*.cpp $(TETRA_INCLUDE) $(TETRA_LIBS)

emscripten:
	mkdir -p bin/emscripten/
	em++ -o bin/emscripten/DeformableArmadillo.html -O2 -std=c++11 -Wall DeformableArmadillo.cpp SimulationConstraint.cpp framework/*.cpp $(EM_INCLUDE) $(EM_LIBS) --preload-file assets --exclude-file assets/armadillo_decimated.obj
