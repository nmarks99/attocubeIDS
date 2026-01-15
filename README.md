# Attocube IDS
EPICS support for [Attocube IDS3010](https://www.attocube.com/en/products/laser-displacement-sensor/displacement-measuring-interferometer)
displacement measuring interferometer based on `asynPortDriver`.

The following records are available in the `attocubeIDS.db` database:
```cpp
int64in   $(P)$(R):AbsPos1
int64in   $(P)$(R):AbsPos2
int64in   $(P)$(R):AbsPos3
int64in   $(P)$(R):Disp1
int64in   $(P)$(R):Disp2
int64in   $(P)$(R):Disp3
int64in   $(P)$(R):RefPos1
int64in   $(P)$(R):RefPos2
int64in   $(P)$(R):RefPos3
bo        $(P)$(R):StartMeasurement
bo        $(P)$(R):StopMeasurement
longin    $(P)$(R):MeasEnabled
stringin  $(P)$(R):Mode
stringin  $(P)$(R):DeviceType
stringin  $(P)$(R):FPGAVersion
ao        $(P)$(R):PollPeriodSec
mbbo      $(P)$(R):PollPeriodMenu
bo        $(P)$(R):SuspendPoller
bo        $(P)$(R):ResumePoller
bi        $(P)$(R):Polling
```
