ROOT := ../
SRC := $(ROOT)/dwave/preprocessing/
CATCH2 := $(ROOT)/testscpp/Catch2/single_include/
DIMOD := $(shell python -c 'import dimod; print(dimod.get_include())')
SPDLOG := $(ROOT)/extern/spdlog/include/
INCLUDES := -I $(SRC)/include/ -I $(DIMOD) -I $(CATCH2) -I $(TASKFLOW) -I $(SPDLOG)
FLAGS := -std=c++17 -Wall -Wno-unknown-pragmas -Wno-sign-compare -Wno-deprecated-declarations -fcompare-debug-second -O3 

all: update test_main test_main_parallel tests tests_parallel

tests: test_main.out
	./test_main 

tests_parallel: test_main_parallel.out
	./test_main_parallel

test_main: test_main.cpp
	g++ -std=c++11 -Wall -c test_main.cpp
	g++ -std=c++11 -Wall test_main.o tests/*.cpp -o test_main -I $(SRC)/include/ -I $(DIMOD) -I $(CATCH2)

test_main_parallel: test_main.cpp
	g++ -std=c++11 -fopenmp -Wall -c test_main.cpp -o test_main_parallel.o
	g++ -std=c++11 -fopenmp -Wall test_main_parallel.o tests/*.cpp -o test_main_parallel -I $(SRC)/include/ -I $(DIMOD) -I $(CATCH2)

update:
	git submodule init
	git submodule update
