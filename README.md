<h1 align="center">🤖 Micras 🤖</h1>
<p align="center">NTF Classic Micromouse project with a STM32 microcontroller</p>

<div align="center">
    <img src="https://forthebadge.com/images/badges/made-with-c-plus-plus.svg"  height="30" href="https://cplusplus.com/"/>
    <img src="data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIxODkuMzkwNjQ0MDczNDg2MzMiIGhlaWdodD0iMzUiIHZpZXdCb3g9IjAgMCAxODkuMzkwNjQ0MDczNDg2MzMgMzUiPjxyZWN0IHdpZHRoPSI1Ni44MjgxMjg4MTQ2OTcyNjYiIGhlaWdodD0iMzUiIGZpbGw9IiMwMGM2ZDEiLz48cmVjdCB4PSI1Ni44MjgxMjg4MTQ2OTcyNjYiIHdpZHRoPSIxMzIuNTYyNTE1MjU4Nzg5MDYiIGhlaWdodD0iMzUiIGZpbGw9IiMwMGE2YjkiLz48dGV4dCB4PSIyOC40MTQwNjQ0MDczNDg2MzMiIHk9IjIxLjUiIGZvbnQtc2l6ZT0iMTIiIGZvbnQtZmFtaWx5PSInUm9ib3RvJywgc2Fucy1zZXJpZiIgZmlsbD0iI0ZGRkZGRiIgdGV4dC1hbmNob3I9Im1pZGRsZSIgbGV0dGVyLXNwYWNpbmc9IjIiPlVTRVM8L3RleHQ+PHRleHQgeD0iMTIzLjEwOTM4NjQ0NDA5MTgiIHk9IjIxLjUiIGZvbnQtc2l6ZT0iMTIiIGZvbnQtZmFtaWx5PSInTW9udHNlcnJhdCcsIHNhbnMtc2VyaWYiIGZpbGw9IiNGRkZGRkYiIHRleHQtYW5jaG9yPSJtaWRkbGUiIGZvbnQtd2VpZ2h0PSI5MDAiIGxldHRlci1zcGFjaW5nPSIyIj5TVE0zMkNVQkVNWDwvdGV4dD48L3N2Zz4="  height="30" href="https://www.st.com/en/development-tools/stm32cubemx.html"/>
     <img src="data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIyNzAuMDMxMjcyODg4MTgzNiIgaGVpZ2h0PSIzNSIgdmlld0JveD0iMCAwIDI3MC4wMzEyNzI4ODgxODM2IDM1Ij48cmVjdCB3aWR0aD0iOTMuNTAwMDA3NjI5Mzk0NTMiIGhlaWdodD0iMzUiIGZpbGw9IiNhZGVjMzciLz48cmVjdCB4PSI5My41MDAwMDc2MjkzOTQ1MyIgd2lkdGg9IjE3Ni41MzEyNjUyNTg3ODkwNiIgaGVpZ2h0PSIzNSIgZmlsbD0iIzI3YTc0NCIvPjx0ZXh0IHg9IjQ2Ljc1MDAwMzgxNDY5NzI2NiIgeT0iMjEuNSIgZm9udC1zaXplPSIxMiIgZm9udC1mYW1pbHk9IidSb2JvdG8nLCBzYW5zLXNlcmlmIiBmaWxsPSIjRkZGRkZGIiB0ZXh0LWFuY2hvcj0ibWlkZGxlIiBsZXR0ZXItc3BhY2luZz0iMiI+QlVJTFQgRk9SPC90ZXh0Pjx0ZXh0IHg9IjE4MS43NjU2NDAyNTg3ODkwNiIgeT0iMjEuNSIgZm9udC1zaXplPSIxMiIgZm9udC1mYW1pbHk9IidNb250c2VycmF0Jywgc2Fucy1zZXJpZiIgZmlsbD0iI0ZGRkZGRiIgdGV4dC1hbmNob3I9Im1pZGRsZSIgZm9udC13ZWlnaHQ9IjkwMCIgbGV0dGVyLXNwYWNpbmc9IjIiPkVNQkVEREVEIERFVklDRVM8L3RleHQ+PC9zdmc+"  height="30" href="https://en.wikipedia.org/wiki/Embedded_system"/>
</div>

## 📑 Summary

- [📁 Folder structure](#📁-folder-structure)
- [📦️ Packages](#📦️-packages)
- [🔨 Building](#🔨-building)
- [🚀 Running](#🚀-running)
- [🐛 Debugging](#🐛-debugging)
- [💄 Code style](#💄-code-style)
  - [🎨 Format](#🎨-format)
  - [🚨 Linter](#🚨-linter)
- [📝 Documentation](#📝-documentation)
- [💬 Git commit messages](#💬-git-commit-messages)
- [🔀 Git workflow](#🔀-git-workflow)
- [👥 Contributors](#👥-contributors)

## 📁 Folder structure

- **.vscode** - Visual Studio Code configuration files
- **cmake/** - Functions to include in the main CMake
- **config/** - Target and constants configuration values
- **cube/** - STM32CubeMX configuration and build files
- **include/** - Header files for class definitions
- **src/** - Source file for class implementations and executables
- **tests/** - Executable test files

## 📦️ Packages

- [micras_hal](./micras_hal/) - Wrapper to the STM32 HAL, implementing the needed functionalities in C++ classes.
- [micras_proxy](./micras_proxy/) - Intermediate abstraction layer for the hardware components.

## 🔨 Building

To build the project, it is first necessary to install some dependencies:

```bash
sudo apt install cmake build-essential gcc-arm-none-eabi
```

The [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) program is also needed. After the instalation is competed, the building process can be started by creating a build folder with:

```bash
mkdir build && cd build
```

And then generating the build commands:

```bash
cmake ..
```

The project can then be compiled by running:

```bash
make -j
```

It is possible to get all possible make commands by running:

```bash
make helpme
```

## 🚀 Running

The binaries can be flashed into the microcontroller using the [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html), whitch needs to be installed, and can then be run using the command:

```bash
make -j flash
```

If the project hasn't already been compiled, this command also automatically compiles the desired target.

## 🧪 Testing

Any test can be compiled and run using the same commands as the ones used for the main executable, but adding the test name without the file extension at the end:

```bash
make -j [test_name]
```

```bash
make -j flash [test_name]
```

## 🐛 Debugging

It is possible to debug the project using [`gdb`](https://www.gnu.org/software/gdb/). To do that, first install `gdb-multiarch`, on Ubuntu, just run:

```bash
sudo apt install gdb-multiarch
```

To be able to debug the project, it is necessary run the `cmake` command with the `BUILD_TYPE` set to `Debug` or `RelWithDebInfo`, for example:

```bash
cmake .. -DBUILD_TYPE=Debug
```

It is also possible to debug test executables, by first running the command:

```bash
make debug [test_name]
```

Finally, to debug the project, the [Cortex Debug extension](https://marketplace.visualstudio.com/items?marus25.Cortex-Debug) for VSCode must be installed. There are three configurations for debugging present at the [`.vscode/launch.json`](./.vscode/launch.json) file that uses differents programs:

- [OpenOCD](https://openocd.org/)
- [J-Link]()
- [ST-Util]()

For each debug type, it is necessary to install the respective gdb server.

## 💄 Code style

### 🎨 Format

The project uses `clang-format` to format files, there is a `.clang-format` with the formatting rules for the project. To install it, on Ubuntu, run the following command on the terminal:

```bash
sudo apt install clang-format
```

In order to format the project, run the following command in the `build` folder:

```bash
make format
```

### 🚨 Linter

The project uses a linter in order to follow the best code practices. The linter used is `clang-tidy`, there is a `.clang-tidy` with the linting rules for the project. To install it on Ubuntu, run the following command on the terminal:

```bash
sudo apt install clang-tidy
```

The linting process is done when compiling the project using a special config variable, the `ENABLE_LINTER` cmake variable. So to lint the code, do as follows using catkin tools:

```bash
cmake .. -DENABLE_LINTER=ON
```

To disable the linter while compiling, do as follows:

```bash
cmake .. -DENABLE_LINTER=OFF
```

It is also possible to lint the project and let the linter fix it using its suggestions:

```bash
cmake .. -DENABLE_LINTER=FIX
```

## 📝 Documentation

The project is documented using Doxygen. In Ubuntu, it is possible to install it with the following command:

```bash
sudo apt install doxygen pdflatex
```

For other operating systems, you can see download options on the [official Doxygen page](https://www.doxygen.nl/download.html).

To generate the pdf documentation at the **docs/** folder, run the following command inside the [**build**](./build/) folder:

```bash
make docs
```

The configuration is in the file [Doxyfile](./Doxyfile).

### 💬 Git commit messages

- Use the present tense ("Add feature" not "Added feature")
- Use the imperative mood ("Move cursor to..." not "Moves cursor to...")
- It is strongly recommended to start a commit message with a related emoji
  - 📝 `:memo:` for documentation
  - 🐛 `:bug:` for bug issues
  - 🚑 `:ambulance:` for critical fixes
  - 🎨 `:art:` for formatting code
  - ✨ `:sparkles:` for new features

  For more examples, see [this reference](https://gitmoji.carloscuesta.me/).

### 🔀 Git workflow

The project workflow is based on [Git Flow](https://nvie.com/posts/a-successful-git-branching-model/).

## 👥 Contributors

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tr>
    <td align="center"><a href="https://github.com/GabrielCosme"><img src="https://avatars.githubusercontent.com/u/62270066?v=4?s=100" width="100px;" alt="Gabriel Cosme Barbosa"/><br/><sub><b>Gabriel Cosme Barbosa</b></sub></a><br/><a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=GabrielCosme" title="Code">💻</a> <a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=GabrielCosme" title="Documentation">📖</a> <a href="#research-GabrielCosme" title="Research">🔬</a> <a href="https://github.com/Team-Micras/MicrasFirmware/pulls?q=is%3Apr+reviewed-by%3AGabrielCosme" title="Reviewed Pull Requests">👀</a></td>
    <td align="center"><a href="https://github.com/PedroDeSanti"><img src="https://avatars.githubusercontent.com/u/62271285?v=4" width="100px;" alt="Pedro de Santi"/><br/><sub><b>Pedro de Santi</b></sub></a><br/><a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=PedroDeSanti" title="Code">💻</a> <a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=PedroDeSanti" title="Documentation">📖</a> <a href="#research-PedroDeSanti" title="Research">🔬</a> <a href="https://github.com/Team-Micras/MicrasFirmware/pulls?q=is%3Apr+reviewed-by%3APedroDeSanti" title="Reviewed Pull Requests">👀</a></td>
    <td align="center"><a href="https://github.com/Matheus3007"><img src="https://avatars.githubusercontent.com/u/53058455?v=4" width="100px;" alt="Matheus Rezende Pereira"/><br/><sub><b>Matheus Rezende Pereira</b></sub></a><br/><a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=Matheus3007" title="Code">💻</a> <a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=Matheus3007" title="Documentation">📖</a> <a href="#research-Matheus3007" title="Research">🔬</a> <a href="https://github.com/Team-Micras/MicrasFirmware/pulls?q=is%3Apr+reviewed-by%3AMatheus3007" title="Reviewed Pull Requests">👀</a></td>
    <td align="center"><a href="https://github.com/Eduardo-Barreto"><img src="https://avatars.githubusercontent.com/u/34964398?v=4" width="100px;" alt="Eduardo Barreto"/><br/><sub><b>Eduardo Barreto</b></sub></a><br/><a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=Eduardo-Barreto" title="Code">💻</a> <a href="https://github.com/Team-Micras/MicrasFirmware/pulls?q=is%3Apr+reviewed-by%3AEduardo-Barreto" title="Reviewed Pull Requests">👀</a></td>
  </tr>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!
