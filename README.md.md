# DPS3005 Based Bench PSU

This is where all the design files for the **Ginger Wash Plant** is saved. 
![Render](https://raw.githubusercontent.com/nadeeofthings/bench-psu/main/Render.png)

# Folders

 - **Schematic** - Design schematic
 - **Gerber files** - All the files needed for manufacturing
 - **BOM**- Bill of material for parts order
 - **Firmware** - Embedded software for the microcontroller

# Program 
- Bin file is located in Debug. Use STM32CubeProgrammer and STLink-V2 to program the board through SWD

- UART link is provided to tune the PID parameters and enable outputs.

# ToDo

 - [ ] Finalize firmware
 - [ ] Define UART protocol
 - [ ] Prepare an easy way to calculate inductance and capacitance for the output filter

