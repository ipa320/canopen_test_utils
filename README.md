# canopen_test_utils
Various helper scripts and test setups for testing and maintaining CANopen chains.


## Travis - Continuous Integration

Status: [![Build Status](https://travis-ci.com/ipa320/canopen_test_utils.svg?branch=kinetic-devel)](https://travis-ci.com/ipa320/canopen_test_utils)

## Referencing elmos with canopen_elmo_console
Stop bringup

Start elmo console
```
rosrun canopen_test_utils canopen_elmo_console can1 73
```

read current encoder value
```
CA[92]
```

... TO BE CONTINUED ...

set new encoder offset

save new settings

stop elmo console

switch off or reboot elmo???

restart bringup
