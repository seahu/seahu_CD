# seahu_CD
Project for easy create arduino 1-Wire slave devices as mulstisensors/actors

Features:
- 1-wire communication run in background thank interrupt.
- support save data beafore lost power (need additional basic el. circuit).

Contain:
- arduino libraries and examples for 1-Wire slave devices.
    - OneWireSlave.h contain generic 1-Wire slave functions
    - OneWireSlave0xCD.h contain function for easy create multi sensors/actors 1-wire slave devices
- arduino examples howto use the 1-Wire slave devices from master side.
- OWFS (One wire file system) with supprot with this 1-Wire slave devices.
- Linux kerne with supprot with this 1-Wire slave devices. (coming soon)
- Documnetation
