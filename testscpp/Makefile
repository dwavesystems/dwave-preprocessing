ROOT := ..
SRC := $(ROOT)/dwave/preprocessing/
CATCH2 := $(ROOT)/testscpp/Catch2/single_include/
TASKFLOW := $(ROOT)/extern/taskflow/
DIMOD := $(shell python -c 'import dimod; print(dimod.get_include())')
FLAGS=-std=c++17 -Wall -Wno-unknown-pragmas -Wno-deprecated-declarations -Wno-sign-compare -Wno-reorder -g -O0
INCLUDES=-I$(SRC)/include/ -I$(DIMOD) -I$(CATCH2) -I$(TASKFLOW)

all: catch2 test_main test_main_parallel tests tests_parallel

tfprof: test_main.out
	TF_ENABLE_PROFILER=test_main.tfp ./test_main 

tests: test_main.out
	./test_main

tests_parallel: test_main_parallel.out
	./test_main_parallel

test_main: test_main.cpp
	g++ ${FLAGS} -pthread -c test_main.cpp
	g++ ${FLAGS} -pthread test_main.o tests/*.cpp -o test_main ${INCLUDES}

test_main_parallel: test_main.cpp
	g++ ${FLAGS} -fopenmp -c test_main.cpp -o test_main_parallel.o
	g++ ${FLAGS} -fopenmp test_main_parallel.o tests/*.cpp -o test_main_parallel ${INCLUDES}

catch2:
	git submodule init
	git submodule update

clean:
	rm -f *.o *.out test_main test_main_parallel *.json *.tfp
