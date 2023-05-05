ROOT := ..
INCLUDE := $(ROOT)/dwave/preprocessing/include
SRC := $(ROOT)/dwave/preprocessing/src
DIMOD := $(shell python -c 'import dimod; print(dimod.get_include())')
CATCH2 := $(ROOT)/testscpp/Catch2/single_include/
SPDLOG := $(ROOT)/extern/spdlog/include/
FLAGS := -std=c++17 -Wall -Wno-unknown-pragmas -fcompare-debug-second -O3

all: update tests

test_main.o:
	$(CXX) test_main.cpp -c

# Developer note: we could make this more generic, but because the package structure
# is not at all consistent and because we have so few test files I think it's clearer to
# just enumerate them for now.

exceptions.o: $(INCLUDE)/dwave/exceptions.hpp $(SRC)/exceptions.cpp
	$(CXX) $(FLAGS) $(SRC)/exceptions.cpp -c -I$(INCLUDE)

presolve.o: $(INCLUDE)/dwave/presolve.hpp $(SRC)/presolve.cpp $(SRC)/presolveimpl.hpp
	$(CXX) $(FLAGS) $(SRC)/presolve.cpp -c -I$(INCLUDE) -I$(SRC) -I$(DIMOD)

test_presolve.o: tests/test_presolve.cpp $(INCLUDE)/dwave/presolve.hpp 
	$(CXX) $(FLAGS) tests/test_presolve.cpp -c -I$(CATCH2) -I$(INCLUDE) -I$(DIMOD) -I$(SPDLOG)

test_roof_duality.o: tests/test_roof_duality.cpp $(wildcard $(INCLUDE)/dwave-preprocessing/*hpp)
	$(CXX) $(FLAGS) tests/test_roof_duality.cpp -c -I$(DIMOD) -I$(CATCH2) -I$(INCLUDE)

tests.out: test_main.o exceptions.o presolve.o test_presolve.o test_roof_duality.o
	$(CXX) $(FLAGS) test_main.o exceptions.o presolve.o test_presolve.o test_roof_duality.o -o tests.out

tests: tests.out
	./tests.out

update:
	git submodule init
	git submodule update

clean:
	rm -f *.o *.out
