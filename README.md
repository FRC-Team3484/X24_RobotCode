# X24_RobotCode
---
<img src="logo.png" style="width: 70%; margin-left: 15%; margin-right: 15%;">

## Installation

First, clone the repository
```
git clone https://github.com/FRC-Team3484/X24_RobotCode
```

Then, open the `2024Robot_Team3484` folder in Visual Studio Code.
Open the command pallate with `Ctrl-Shift-P` and run the `WPILib: Build Robot Code` command.

You can use `Git: Open Repository` and select the `X24_RobotCode` parent folder to enable the Git tools inside of VSCode, while still being able to build and run the code.

## NeoVim
If you wanted to use NeoVim to write code instead of Visual Studio Code, follow these steps:

Make a `CMakeLists.txt` file in `2024Robot_Team3484`

*TODO: What are the contents of CMakeLists.txt?*

Inside the `2024Robot_Team3484` directory, create a directory called `out`, and `cd out`

Run `cmake ..`

Move `compile_commands.json` from `out` to `2024Robot_Team3484/build`

Build the code by running the command `./gradlew build`