### Multi-role Digital Module
This is a part of the implementation for the Ultracold Control System provided by ParkerLab at Georgia Tech.
A Digital Module can generate a sequence of digital signals over 16 channels. The TTL signals are directly generated by the GPIO pins from the EK-TM4C123GH6PM evaluation board. Other than a signal generator, the module can also act as the "Leader" module that generates the "Start" signal, and/or the "Clock" module that generates a 80MHz clock, or the "Repeater" that repeats messages from the PC terminal to other modules.
### Hardware
The corresponding hardware design can be find in https://github.com/parkerlab-external/UltracoldPCB-Dig.
### Protocol
To update the sequence the module accepts command lists in the following format.
```
 × 1    #bytes 1         2             
        value  'E'       chkpt count 
 × n    #bytes 1         2            4               2
        value  'U'       chkpt pos    duration        value 
 × 1    #bytes 1
        value  'T'
```
An implementation of the backend can be found here: https://github.com/parkerlab-external/UltracoldSequenceRunner.
### Configurables
The device specific configurations including modes and UART address can be set in `config_device.h` or `config_device_leadrepeat.h`. You can choose to include one of these two templates in `data_io_dig.h`. 