# MC-Latched-Button
A button module for activating other high-voltage devices for a specific time

The device uses a microcontroller to control a relay that can be connected to the power supply of mains powered devices.
The button has to be pressed for a preconfigured amount of time (default 5 seconds) and keeps the relay closed for another configurable time (default 12 minutes). Device can be shutdown by pressing the button for a configurable amount of time (default: short press, 0.2 seconds).

Module fits a 2 unit DIN circuit breaker box (but does not attach to rails).

Notes:
1. Timing loop is apparently 107 uS, so the corresponding timing variable should be close to 9345, but it is actually 11111. Realtime code seems to be taking 90uS to run, for reasons unknonwn.

2. Requires either a 5 volt connection (use the ICSP header for that mode) or an external mains power to 5V supply (connect using a screw in terminal on the lower left part of the PCB, and hook the inputs and outputs of the modules to the 4 holes immediately above the terminal.

3. Has no structural elements to support the module. Use thick copper wire to connect to the button and screw button to case. It may be possible to improvise a device that grips to the DIN rails, but the PSU will interfere if mounted in the back (and will interfere with the connection to the button if mounted in the front).

4. I normally use a dielectric coating to protect the module and its PSU from the elements and for added isolation.
