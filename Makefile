# Dev convenience wrappers around the pip-based build (install = build).
# Uses the venv's pip directly, so no prior activation is needed.
PIP := .venv/bin/pip

.PHONY: build debug clean docs

# Incremental release build of the extension module (default target)
build:
	$(PIP) install -e . --no-build-isolation

# Debug build
debug:
	$(PIP) install -e . --no-build-isolation --config-settings=cmake.build-type=Debug

# Drop the CMake build tree; next `make build` rebuilds from scratch
clean:
	rm -rf build

# Generate the Doxygen API reference into developer-docs/doxygen/html/
docs:
	cd developer-docs/doxygen && doxygen Doxyfile
