## Python SDK setup

These instructions are for Fedora Linux.Should work with Debian with correct modifications

### 1. Install the system dependencies

```bash
sudo dnf install python3 python3-tkinter
```

### 2. Create and activate a virtual environment

Run these commands from the `sdk_python` directory:

```bash
python3 -m venv foxglove_env
source foxglove_env/bin/activate
```

The environment must be activated whenever you work with or run the Python SDK.

### 3. Install the Foxglove WebSocket library

```bash
python -m pip install --upgrade pip
python -m pip install foxglove-websocket
```

### 4. Run the application

With the virtual environment active, run the project's Python entry point, for example:

```bash
python <entry_point>.py
```

Replace `<entry_point>.py` with the Python file that starts the application.

To leave the virtual environment when finished:

```bash
deactivate
```
