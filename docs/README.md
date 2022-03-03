# Environment Sensor Bluetooth

This is the documentation of Environment Sensor Bluetooth documentation.
The official website is [here](https://hktkzyx.github.io/environment-sensor-bluetooth/).
You can find the correponding hardware repository @hktkzyx/environment-sensor-bluetooth-hardware.

## Description

This firmware firstly setup the device pending bluetooth connection.
If no connection after a certain limit, i.e., `IDLE_LIMIT` in the `config.h` file,
the device will start deep sleep and wake up after `WAKEUP_INTERVAL` seconds.
If connected, the device start measure temperature, humidity, and illustration repeatly
at the `UPDATE_INTERVAL` interval seconds, until the remote client disconnect.
Onice the remote client is disconnected, deep sleep starts to reduce power assumption.

## Installation

This project is based on [PlatformIO](https://platformio.org/).
Please install [PlatformIO IDE](https://platformio.org/platformio-ide)
or [PlatformIO Core](https://platformio.org/install/cli) firstly.
Then, clone the repository @hktkzyx/environment-sensor-bluetooth

```bash
git clone https://github.com/hktkzyx/environment-sensor-bluetooth.git
cd environment-sensor-bluetooth
```

## Usage

Create your configuation files.

```bash
cp ./src/config.h.example ./src/config.h
```

If you use PlatformIO IDE, open this project by VSCode and then upload to the hardware.
For PlatformIO Core, upload the firmware by the following command:

```bash
pio run --target upload
```

## Contributing

Welcome fork this project!
I am not professional on the embeded development.
I appreciate if you can fix bugs or develop new features.
Before your development, please follow the below rules to ensure the code quality.

1. Install [pre-commit](https://pre-commit.com/)
   and follow the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/).

    When `pre-commit` is installed, run

    ```bash
    pre-commit install -t pre-commit -t commit-msg
    ```

    And you can also install [commitizen](https://github.com/commitizen-tools/commitizen) to submit your commits.

2. Use [clang-format](https://clang.llvm.org/docs/ClangFormat.html) to format your source code.
3. Follow gitflow branch manage strategies.
    You can install [git-flow](https://github.com/petervanderdoes/gitflow-avh) to manage branches. Then,

    ```bash
    git config gitflow.branch.master main
    git config gitflow.prefix.versiontag v
    git flow init -d
    ```

## License

Copyright (c) 2022 hktkzyx.

Environment Sensor Bluetooth firmware is licensed under Mulan PSL v2.

You can use this software according to the terms and conditions of the Mulan PSL v2. You may obtain a copy of Mulan PSL v2 at: http://license.coscl.org.cn/MulanPSL2.

THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT, MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.

See the Mulan PSL v2 for more details.
