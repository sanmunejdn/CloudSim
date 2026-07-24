# RobotCommBridge

.NET Framework 4.8 console host for HslCommunication robot adapters.

## Run

```bat
set HSL_AUTH_CODE=your_code
bin\x64d\RobotComm\RobotCommBridge.exe --port 19610
```

Or place `hsl.auth` next to the exe (do not commit).

## Protocol

Newline-delimited JSON on TCP. Commands: `ping`, `connect`, `disconnect`, `get_state`, `get_feedback`.

See `CloudSim/docs/机器人通讯/DESIGN_机器人通讯.md`.
