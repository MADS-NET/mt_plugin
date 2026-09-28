# machinetool plugin for MADS

[![Build](https://github.com/MADS-NET/mt_plugin/actions/workflows/build-release.yml/badge.svg)](https://github.com/MADS-NET/mt_plugin/actions/workflows/build-release.yml)  


This is a Sink plugin for [MADS](https://github.com/MADS-NET/MADS). 

This plugin visualizes on [Rerun](https://rerun.io) a 3D model of a machine tool, updating axes positions according to the received commands. It is just a viewer and does not provide any dinamics simulation. Foor that, head to <https://gitbub.com/mads-net/FMU_agent>.

> *Required MADS version: 2.4.3.* or later
> This version won't work with earlier versions of MADS.

## Supported platforms

Currently, the supported platforms are:

* **Linux** 
* **MacOS**
* **Windows**


## Requirement: Rerun


### Build from source

The easiest way to install the Rerun viewer is via pip:

```sh
python3 -mvenv .venv
source .venv/bin/activate # or, on Windows: .venv\script\activate.ps1
pip install rerun-sdk==0.31.4
```

**Note that on Windows, you shall use a PowerShell terminal**

Type `rerun` and be sure that a Rerun windows opens. You might then close it.


### Linux and MacOS:

```bash
cmake -Bbuild -DCMAKE_INSTALL_PREFIX="$(mads -p)"
cmake --build build -j4
```

### Windows:

```powershell
cmake -Bbuild -DCMAKE_INSTALL_PREFIX="$(mads -p)"
cmake --build build --config Release
```

Once compiled, provided that you have a running broker and the settings contain a `[machinetool]` section, you may run the plugin from the same termninal in which you have activated the Python venv:

```sh
mads sink build/machinetool.plugin
```

## Install from release

Download a release from GitHub, unzip it, then open a shell in the unzipped folder and create a Python venv as above described, including installing the Rerun-SDK.

Provided that you have a running broker and the settings contain a `[machinetool]` section, proceed as follows:

### Windows

```ps1
cd <unzipped folder>
python -mvenv .venv
.\.venv\script\activate.ps1
pip install rerun-sdk
mads sink lib/machinetool.plugin
```

### MacOS

```sh
cd <unzipped folder>
xattr -dr com.apple.quarantine .
python -mvenv .venv
source .venv/bin/activate
pip install rerun-sdk
mads sink lib/machinetool.plugin
```



Then launch the agent from same shell.

> **NOTE**: the plugin expects to find the solid models for  machine tool components in the `models` folder right in the current working directory, so you must launch the plugin from the project root (if compiled) of from the root of the downloaded folder (if downloades a zipped release).

## Commands

The plugin moves the three axes and adjusts the tool size according to the following JSON structure:

```json
{
  "timecode": <elapsed seconds from last midnight>,
  "input": {"position": [<3 elements>]},
  "output": {
    "position": [<3 elements>],
    "speed": <value>
  },
  "tool": {
    "length": <scalar>,
    "diameter": <scalar>,
    "z_offset": <scalar>
  },
  "metrics": {
    <map of custom key-values>
  }
}
```

The `timecode` field is required and is used as Rerun's time abscissa. It must be
the elapsed seconds from the last midnight, rounded to the producer's 5 ms
resolution.

Only `["output"]["position"]` is used to move the axes. The remainig data are simply logged (and possibly plotted) by Rerun.



## INI settings

The plugin supports the following settings in the INI file:

```ini
[machinetool]
sub_topic = ["fmu_machine_tool"]
initial_position = [0, 0, 200]
tool = {length = 100, diameter = 20, z_offset = 50}
```

All settings are optional; if omitted, the default values are used.


---
