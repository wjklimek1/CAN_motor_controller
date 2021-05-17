# CAN motor controller
This project is a device that allows control of DC motor by commands on CAN bus. It is based on STM32F103 microcontroller that interprets CAN messages and drives motor using two BTN8982 half-bridges configured as a single H-bridge. 

## Motor driving capabilities
DC motor is controlled with 21kHz PWM signal with 0-100% duty. PWM can be regulated in 0.1% steps - from 0 to 1000. Theoretically max current that MOSFETs can handle is 55A, but it can be futher limited by traces width and connectors. It was tested up to 10A, but should handle much more without any problems. Power supply voltage range is 5-24V.
| Parameter | Min | Max |
|--|--|--|
| voltage | 5V | 24V |
| motor current |  | 55A (10A tested) |

Motor is accelerated slowly by changing PWM duty. If received command orders to immediately set PWM to 100% from 0%, duty is increased slowly in default 10ms intervals. Therefore spinning motor to top speed will take 10s. This property can be disabled by setting "motor inertia" parameter to 0 (see CAN commands table).

## CAN configuration
CAN bus in STM32F103 runs on 250kb/s by default. It can be set to other, hard coded value in the initialization function. Other supported baudrates are 500kb/s and 1Mb/s. 

## Heartbeat
Device emits "heartbeat" frame every 1s. It is an empty frame with ID = 0x444. Heartbeat can be disabled or it's period can be changed in 1s steps up to 255s.

## CAN message structure
Device utilizes two identifiers - 0x444 for commands and 0x445 for requesting data. This values are hard coded too, but can be changed in *canbus.h* file. DLC of CAN messages is not really important, but it cannot be shorter than expected.
In both commands and data requests first byte of message is code of command or data request. In commands next bytes represent values of command parameters. When data request is received by device, it will respond with frame of the same ID and data request code. Next bytes will contain requested data.

## Commands
Following table shows available commands:
|Command | frame ID | DLC | byte 0 (command code) | byte 1 | byte 2 | byte 3 |
|--|--|--|--|--|--|--|
|Set speed | 0x444 | 4 | 1 | rotation direction (0 or 1) | MSB of speed | LSB of speed|
|Set max temperature internal | 0x444 | 2 | 2 | max internal temperature (uint8_t) in Celcius | - | - |
|Set max temperature external | 0x444 | 2 | 3 | max external temperature (uint8_t) in Celcius | - | - |
|Set max current | 0x444 | 4 | 4 | MSB of max current (mA) | LSB of max current (mA) | - |
|Emergency stop | 0x444 | 1 | 5 | - | - | - |
|Set inertia | 0x444 | 2 | 6 | Inertia (ms delay between changing PWM duty) | - | - |
|Set heartbeat | 0x444 | 2 | 7 | heartbeat interval (0 for disabled) | - | - |

## Data requests
To request data, frame of ID = 0x445 and first byte containing data request code can be sent. Device will respond with similar frame, but containing data in next bytes.
 
Following table shows available data requests:
|Data request | frame ID | DLC | byte 0 (data request code) | byte 1 | byte 2 | byte 3 | byte 4 |byte 5 |
|--|--|--|--|--|--|--|--|--|
|Internal temperature | 0x445 | 2 | 1 | Internal temperature (uint8_t) in |Celcius | - | - | - | - |
|External temperature | 0x445 | 2 | 2 | External temperature (uint8_t) in |Celcius | - | - | - | - |
|Power supply voltage | 0x445 | 3 | 3 | MSB of voltage in mV | LSB of voltage in mV | - | - | - |
|Motor current | 0x445 | 3 | 4 | MSB of current in mA | LSB of current in mA | - | - | - |
|Speed | 0x445 | 4 | 5 | rotation direction (0 or 1) | MSB of speed | LSB of speed| - | - |
|Target speed | 0x445 | 4 | 6 | rotation direction (0 or 1) | MSB of speed | LSB of speed| - | - |
|Errors | 0x445 | 5 | 7 | MOSFET fault | overcurrent | overtemperature internal | overtemperature external | emergency stop |