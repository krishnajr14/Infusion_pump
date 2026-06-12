.PHONY: coverage

coverage:
	cmake -B build -DENABLE_COVERAGE=ON
	cmake --build build -j$(shell nproc)
	cmake --build build --target coverage
