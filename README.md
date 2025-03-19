<!-- markdownlint-disable -->
<div align="center">

![micras_firmware_logo](https://github.com/Team-Micras/MicrasFirmware/assets/62271285/1fa07e90-a279-4210-bb58-5e8931c87f64)

NTF Classic Micromouse project with a STM32 microcontroller

</div>

<div align="center">
  <a href="https://cplusplus.com/"><img alt="Made with C++" src="https://img.shields.io/badge/made_with-c%2B%2B-blue?style=for-the-badge&labelColor=ef4041&color=c1282d" height="30"></a>
  <a href="https://www.st.com/en/development-tools/stm32cubemx.html"><img alt="Uses STM32CubeMX" src="https://img.shields.io/badge/uses-stm32cubemx-blue?style=for-the-badge&labelColor=38c1d0&color=45a4b8&link=https%3A%2F%2Fwww.st.com%2Fen%2Fdevelopment-tools%2Fstm32cubemx.html" height="30"></a>
  <a href="https://en.wikipedia.org/wiki/Embedded_system"><img alt="Built for Embedded Devices" src="https://img.shields.io/badge/built_for-embedded_devices-blue?style=for-the-badge&labelColor=adec37&color=27a744&link=https%3A%2F%2Fen.wikipedia.org%2Fwiki%2FEmbedded_system" height="30"></a>
</div>
<!-- markdownlint-restore -->

## 📑 Summary

- [📑 Summary](#-summary)
- [📁 Folder structure](#-folder-structure)
- [📦️ Packages](#️-packages)
- [🔨 Building](#-building)
- [🚀 Running](#-running)
- [🧪 Testing](#-testing)
- [🐛 Debugging](#-debugging)
- [💄 Code style](#-code-style)
  - [🎨 Format](#-format)
  - [🚨 Linter](#-linter)
- [🐋 Docker](#-docker)
- [📝 Documentation](#-documentation)
- [👥 Contributing](#-contributing)
  - [💬 Git commit messages](#-git-commit-messages)
  - [🔀 Git workflow](#-git-workflow)
- [✨ Contributors](#-contributors)

## 📁 Folder structure

- **.docker/** - Docker build and format scripts for CI/CD.
- **.github/** - GitHub Actions workflow files.
- **.vscode/** - Visual Studio Code configuration files.
- **cmake/** - Functions to include in the main CMake.
- **config/** - Target and constants configuration values.
- **cube/** - STM32CubeMX configuration and build files.
- **include/** - Header files for class definitions.
- **src/** - Source file for class implementations and executables.
- **tests/** - Executable test files.

## 📦️ Packages

- [micras_hal](./micras_hal/) - Wrapper to the STM32 HAL, implementing the needed functionalities in C++ classes.
- [micras_proxy](./micras_proxy/) - Intermediate abstraction layer for the hardware components.
- [micras_nav](./micras_nav/) - Mapping, planning and control algorithms to navigate inside a maze.

## 🔨 Building

To build the project, it is first necessary to install some dependencies:

```bash
sudo apt install cmake make gcc-arm-none-eabi
```

And initialize the libraries submodules:

```bash
git submodule update --init --recursive
```

The [STM32CubeMX](https://www.st.com/en/development-tools/stm32cubemx.html) program is also needed. After the installation is completed, it is necessary to set the environment variable `CUBE_PATH` to the installation directory path of STM32CubeMX.

The building process can be started by creating a build folder with:

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

The binaries can be flashed into the microcontroller using the [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html), which needs to be installed. For flashing the main program, run the following command:

```bash
make flash -j
```

If the project hasn't already been compiled, this command also automatically compiles the desired target.

## 🧪 Testing

Any test can be compiled and run using the same commands as the ones used for the main executable, but adding the test name without the file extension at the end:

```bash
make [test_name] -j
```

```bash
make flash_[test_name] -j
```

It also possible to build all tests at once, using the command:

```bash
make test_all -j
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

And then generate the launch files for debugging with the command:

```bash
make debug
```

It is also possible to debug test executables, by first running the command:

```bash
make debug_[test_name]
```

Finally, to debug the project, the [Cortex Debug extension](https://marketplace.visualstudio.com/items?marus25.Cortex-Debug) for VSCode must be installed. There are three configurations for debugging present at the [`.vscode/launch.json`](./.vscode/launch.json) file that uses different programs:

- [J-Link](https://www.segger.com/downloads/jlink/)
- [OpenOCD](https://openocd.org/) (`sudo apt install openocd`)
- [ST-Util](https://github.com/stlink-org/stlink) (`sudo apt install stlink-tools`)

For each debug type, it is necessary to install the respective gdb server.

If using J-Link, the flashing process can be done by running the following command:

```bash
make jflash -j
```

or if you want to flash a test:

```bash
make jflash_[test_name] -j
```

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

The linting process is done when compiling the project using a special config variable, the `LINTER_MODE` cmake variable. You can enable the linter by running:

```bash
cmake .. -DLINTER_MODE=ON
```

To disable the linter while compiling, do as follows:

```bash
cmake .. -DLINTER_MODE=OFF
```

It is also possible to lint the project and let the linter fix it using its suggestions:

```bash
cmake .. -DLINTER_MODE=FIX
```

## 🐋 Docker

Docker can be used to build the project inside a container, which makes it possible to implement CI/CD pipelines and develop in any environment, for this, it is necessary to have [Docker](https://docs.docker.com/get-started/introduction/get-docker-desktop/) installed on your system.

### 🐳 Building

The project can be built without entering a container by running the following command:

```bash
docker compose run build
```

This also works for formatting (`docker compose run format`) and linting (`docker compose run lint`).

### 🧑‍💻 Development

To enter a container and mount the project folder, run the following command:


```bash
docker compose run dev
```

From inside the container, most of the previous sections can be executed, such as [building](#-building) and [code style](#-code-style).

If Visual Studio Code is being used, it is possible to use the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension to develop inside the container with the other recommended extensions.

## 📝 Documentation

The project is documented using Doxygen. In Ubuntu, it is possible to install it with the following command:

```bash
sudo apt install doxygen texlive-latex-extra texlive-fonts-extra
```

For other operating systems, you can see download options on the [official Doxygen page](https://www.doxygen.nl/download.html).

To generate the pdf documentation at the **docs/** folder, run the following command inside the [**build**](./build/) folder:

```bash
make docs
```

The configuration is in the file [Doxyfile](./Doxyfile).

## 👥 Contributing

To learn how to contribute to the project, see the following contribution guidelines.

### 💬 Git commit messages

- Use the present tense ("Add feature" not "Added feature").
- Use the imperative mood ("Move cursor to..." not "Moves cursor to...").
- It is strongly recommended to start a commit message with a related emoji:
  - 📝 `:memo:` for documentation.
  - 🐛 `:bug:` for bug issues.
  - 🚑 `:ambulance:` for critical fixes.
  - 🎨 `:art:` for formatting code.
  - ✨ `:sparkles:` for new features.

  For more examples, see [this reference](https://gitmoji.dev/).

### 🔀 Git workflow

The project workflow is based on [GitHub Flow](https://docs.github.com/en/get-started/using-github/github-flow).

## ✨ Contributors

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tr>
    <td align="center"><a href="https://github.com/GabrielCosme"><img src="https://avatars.githubusercontent.com/u/62270066?v=4?s=100" width="100px;" alt="Gabriel Cosme Barbosa"/><br/><sub><b>Gabriel Cosme Barbosa</b></sub></a><br/><a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=GabrielCosme" title="Code">💻</a> <a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=GabrielCosme" title="Documentation">📖</a> <a href="#research-GabrielCosme" title="Research">🔬</a> <a href="https://github.com/Team-Micras/MicrasFirmware/pulls?q=is%3Apr+reviewed-by%3AGabrielCosme" title="Reviewed Pull Requests">👀</a></td>
    <td align="center"><a href="https://github.com/PedroDeSanti"><img src="https://avatars.githubusercontent.com/u/62271285?v=4" width="100px;" alt="Pedro de Santi"/><br/><sub><b>Pedro de Santi</b></sub></a><br/><a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=PedroDeSanti" title="Code">💻</a> <a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=PedroDeSanti" title="Documentation">📖</a> <a href="#research-PedroDeSanti" title="Research">🔬</a> <a href="https://github.com/Team-Micras/MicrasFirmware/pulls?q=is%3Apr+reviewed-by%3APedroDeSanti" title="Reviewed Pull Requests">👀</a></td>
    <td align="center"><a href="https://github.com/Matheus3007"><img src="https://avatars.githubusercontent.com/u/53058455?v=4" width="100px;" alt="Matheus Rezende Pereira"/><br/><sub><b>Matheus Rezende Pereira</b></sub></a><br/><a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=Matheus3007" title="Code">💻</a> <a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=Matheus3007" title="Documentation">📖</a> <a href="#research-Matheus3007" title="Research">🔬</a> <a href="https://github.com/Team-Micras/MicrasFirmware/pulls?q=is%3Apr+reviewed-by%3AMatheus3007" title="Reviewed Pull Requests">👀</a></td>
    <td align="center"><a href="https://github.com/Eduardo-Barreto"><img src="https://avatars.githubusercontent.com/u/34964398?v=4" width="100px;" alt="Eduardo Barreto"/><br/><sub><b>Eduardo Barreto</b></sub></a><br/><a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=Eduardo-Barreto" title="Code">💻</a> <a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=Eduardo-Barreto" title="Documentation">📖</a> <a href="#research-Eduardo-Barreto" title="Research">🔬</a> <a href="https://github.com/Team-Micras/MicrasFirmware/pulls?q=is%3Apr+reviewed-by%3AEduardo-Barreto" title="Reviewed Pull Requests">👀</a></td>
    <td align="center"><a href="https://github.com/dardengo"><img src="https://avatars.githubusercontent.com/u/102487809?v=4?s=100" width="100px;" alt="Claudio Dardengo"/><br/><sub><b>Claudio Dardengo</b></sub></a><br/><a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=dardengo" title="Code">💻</a> <a href="https://github.com/Team-Micras/MicrasFirmware/commits?author=dardengo" title="Documentation">📖</a> <a href="#research-dardengo" title="Research">🔬</a> <a href="https://github.com/Team-Micras/MicrasFirmware/pulls?q=is%3Apr+reviewed-by%3Adardengo" title="Reviewed Pull Requests">👀</a></td>
  </tr>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!
