.PHONY: test clean

INCLUDE = -I/opt/homebrew/include -I../include
LDFLAGS = -L/opt/homebrew/lib -lcriterion -L../lib -lpmic
SRC = main.c emulator.c
TARGET = test
CC := clang

# run in verbose mode to see log information, run with a single job to ensure
# logs are in expected order, we can reduce verbosity and increase job count
# if test time becomes a concern
# VERBOSITY = --verbose -j1

test:
	@$(CC) $(INCLUDE) $(LDFLAGS) $(SRC) -o $(TARGET)
	@DYLD_LIBRARY_PATH=../lib ./$(TARGET) $(VERBOSITY)

clean:
	@rm test
