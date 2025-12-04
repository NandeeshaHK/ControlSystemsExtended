# ControlSystemsExtended

Link to Basic Control systems (From *Lolbouy*): https://github.com/lolbuoy/controlsystems

## Installation

You can install this package via pip using the git link:

```bash
pip install git+https://github.com/NandeeshaHK/ControlSystemsExtended.git
```

Or install locally:

```bash
pip install .
```

## SDK Usage

The package is organized into modular components for easier use.

### Connecting
```python
from control_systems_extended.core import connect_mavlink
connection = connect_mavlink('udp:127.0.0.1:14550')
```

### Telemetry
```python
from control_systems_extended.telemetry import status
mode = status.get_mode(connection)
print(f"Current Mode: {mode}")
```

### Mission Management
```python
from control_systems_extended.mission import MissionManager
manager = MissionManager(connection)
manager.clear_mission()
```

## Examples

Check the `examples/` directory for complete scripts:

- `examples/basic_telemetry.py`: Connects and prints status.
- `examples/mission_execution.py`: Loads and manages missions.
- `examples/guided_control.py`: Demonstrates guided mode control.
- `examples/vision_processing.py`: Utilities for vision-based navigation.

To run an example:
```bash
python examples/basic_telemetry.py
```
