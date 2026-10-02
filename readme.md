## Python SDK setup

These instructions are for Fedora Linux.Should work with Debian with correct modifications

### 1. Install the system dependencies

```bash
sudo dnf install python3
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
python web_ui_stream.py
```

### 5. Connecting
Once running, the script hosts several services locally:

- Web UI Dashboard: Open http://localhost:8000 in your browser.

- Foxglove Studio: Open Foxglove, click "Open Connection", select Foxglove WebSocket, and connect to ws://localhost:8765.

- C++ Engine UDP Ports: The Python server listens for C++ telemetry on port 9001 and sends UI commands to C++ on port 9000.


To leave the virtual environment when finished:

```bash
deactivate
```
