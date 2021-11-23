
# Pin Connections

### DC Motor Control to Launchpad 
#### Right
 . Pin 1.6 for spinning direction control
 . Pin 2.6 for for power control (PWM)
 . Pin 3.6 for sleep mode control
#### Left
 . Pin 1.7 for spinning direction control
 . Pin 2.7 for for power control (PWM)
 . Pin 3.7 for sleep mode control

### Power distribution to Launchpad 
 . GND   - GND
 . VREG  - 5V
 . VCCMD - 3V3
 . VPU   - 3V3

### Tachometer to Lauchpad 
 . ELA - P5.0
 . ERA - P6.1

### IR sensors to Launchpad 
 . Left   - P9.1
 . Center - P9.0
 . Right  - P5.2

 #### Debugging 
 . Pin 1.0 is used for debugging the clock. Check `clock_init_48MHz` in clock.c