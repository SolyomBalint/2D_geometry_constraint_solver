# Project: [2D Geometric Constraint Solver]

## Project Description

This is a research proof-of-concept project that aims to provide a working
implementation of 2D geometric constraint solving algorithms.

## Tech Stack

- Main Language: C++ std20
- GUI Language: 3.+ Python
- Build System: CMake 3.31.+
- Dependency Management: Conan2
- Declarative Development Environment: devenv (devenv.sh)

## Project Structure

- `/src` - Main source code
- `/src/utils` - Mathematical library dependencies
- `/src/commons` - General code that is used project wide
- `/src/python_bindings` - Python bindings
- `/src/constraint_solver` - The implementation of the constraint solver algorithms
- `/scripts` - python GUI scripts
- `/include` - API headers that should be installed on targets

## Commands

- `build_linux_gcc_debug` - build debug project with dependencies with gcc
- `build_linux_gcc_release` - build release project with dependencies with gcc

## Code Style

- Use `clang-format` with the `.clang-format` config file in the project root
- Use variable naming defined in the `.clangd` file in the project root (look for the `readability-identifier-naming`
  fields)

## Instructions for Claude

- Use the C++ 20 standard
- Use the latest good practices allowed by the current standard
- Startup in vim mode by default

## Environment Setup

- devenv shell
- Environment variables: CONAN_HOME
- The ogdf-wrapper uses the OGDF graph library