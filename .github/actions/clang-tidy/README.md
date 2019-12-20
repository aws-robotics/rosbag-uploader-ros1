# Clang Tidy Github Action

This action runs clang-tidy against all packages. It runs using a custom python script 
that has sane defaults and logging of what it's working on.

If it discovers any issues it will fail the build. 

## Example Workflow

```
jobs:
  build:
    runs-on: ubuntu-latest
    name: 'Build + Test'
    steps:
    - name: Checkout
      uses: actions/checkout@v1
    - name: Clang Tidy
      uses: ./.github/actions/clang-tidy/
```

## Requirements

You must compile your code with the cmake flag `CMAKE_EXPORT_COMPILE_COMMANDS=ON`. 
You can do this when running `colcon build` by adding the argument `--cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`.

## Inputs

### `workspace-dir`

Path to the workspace folder of your package. **Default: ./**
