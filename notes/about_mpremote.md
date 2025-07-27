very cool little app

mpremote is a command-line tool that allows you to manage a micropython device over a serial connection. It provides a simple interface to interact with the device, upload files, and execute commands.

link: https://github.com/micropython/micropython/tree/master/tools/mpremote

## Installation

on Ubuntu, mpremote is available in the package repositories, so you can install it using the following command:

```bash
sudo apt install micropython-mpremote
```

## supported commands:

```bash
mpremote connect <device>         -- connect to given device
                                     device may be: list, auto, id:x, port:x
                                     or any valid device name/path
mpremote disconnect               -- disconnect current device
mpremote mount <local-dir>        -- mount local directory on device
mpremote eval <string>            -- evaluate and print the string
mpremote exec <string>            -- execute the string
mpremote run <file>               -- run the given local script
mpremote fs <command> <args...>   -- execute filesystem commands on the device
                                     command may be: cat, ls, cp, rm, mkdir, rmdir, sha256sum
                                     use ":" as a prefix to specify a file on the device
mpremote repl                     -- enter REPL
                                     options:
                                         --capture <file>
                                         --inject-code <string>
                                         --inject-file <file>
mpremote mip install <package...> -- Install packages (from micropython-lib or third-party sources)
                                     options:
                                         --target <path>
                                         --index <url>
                                         --no-mpy
mpremote help                     -- print list of commands and exit
```

#### Example Usage

```bash
mpremote
mpremote a1
mpremote connect /dev/ttyUSB0 repl
mpremote ls
mpremote a1 ls
mpremote exec "import micropython; micropython.mem_info()"
mpremote eval 1/2 eval 3/4
mpremote mount .
mpremote mount . exec "import local_script"
mpremote ls
mpremote cat boot.py
mpremote cp :main.py .
mpremote cp main.py :
mpremote cp -r dir/ :
mpremote sha256sum :main.py
mpremote mip install aioble
mpremote mip install github:org/repo@branch
mpremote mip install gitlab:org/repo@branch
```

### Listing Devices

To list serial devices, use the following command:

```bash
$ mpremote connect list

/dev/ttyACM0 e66164084361382d 2e8a:0005 MicroPython Board in FS mode
/dev/ttyS0 None 0000:0000 None None
/dev/ttyUSB0 f4ff8904fb63ef118309e1a9c169b110 10c4:ea60 Silicon Labs CP2102N USB to UART Bridge Controller
```

### Connecting to a Device

To connect to a specific device, use the `connect` command followed by the device path:


```bash
mpremote connect a0
```

### Copy Files to the Device

To copy files to the device, use the `fs cp` command. For example, to copy a file named `main.py` to the root directory of the device:


```bash
mpremote cp path/to/main.py :
```

example:
```bash
mpremote cp main.py :
```

### Copy files from the Device

To copy files from the device to your local machine, use the `fs cp` command with the device path prefixed by a colon. For example, to copy a file named `main.py` from the device to the current directory:

```bash
mpremote cp :main.py .
```

### reset the device

To reset the device, you can use the `exec` command to run a reset command. For example, to reset a MicroPython device, you can use:

```bash
mpremote exec "import machine; machine.reset()"
```

or 

```bash
mpremote a0 reset
```