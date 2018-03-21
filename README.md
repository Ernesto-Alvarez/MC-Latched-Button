# MC-Latched-Button
A button module for activating other high-voltage devices for a specific time

The device uses a microcontroller to control a relay that can be connected to the power supply of mains powered devices.
The button has to be pressed for a preconfigured amount of time (default 5 seconds) and keeps the relay closed for another configurable time (default 12 minutes). Device can be shutdown by pressing the button for a configurable amount of time (default: short press, 0.2 seconds).

Module fits a 2 unit DIN circuit breaker box (but does not attach to rails).

Rationale:
The device was designed for manually controlling part of the basic infrastructure of a house (a water pump). The initial hold time is used to prevent accidental activation of the pump, as a mean of idiot-proofing. The button does not react immediately to prevent accidental or improper activation and the activation can be aborted easily with another button press. The timer value is related to the function of the device being controlled.

All three parameters (the necessary hold time to start the device, the amount of time it operates and the necessary hold time to abort operation) can be configured using the variables hold_tenths, timer_seconds and abort_tenths respectively. The hold times are expressed in 1/10th of a second and the timer in seconds.

Notes:
1. Timing loop is apparently 107 uS, so the corresponding timing variable should be close to 9345, but it is actually 11111. Realtime code seems to be taking 90uS to run, for reasons unknonwn.

2. Requires either a 5 volt connection (use the ICSP header for that mode) or an external mains power to 5V supply (connect using a screw in terminal on the lower left part of the PCB, and hook the inputs and outputs of the modules to the 4 holes immediately above the terminal.

3. Has no structural elements to support the module. Use thick copper wire to connect to the button and screw button to case. It may be possible to improvise a device that grips to the DIN rails, but the PSU will interfere if mounted in the back (and will interfere with the connection to the button if mounted in the front).

4. I normally use a dielectric coating to protect the module and its PSU from the elements and for added isolation.
