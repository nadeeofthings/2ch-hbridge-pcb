# H-Bridge Driver

This is where all the design files for the **H-Bridge Driver** is saved. 
![Render](https://raw.githubusercontent.com/nadeeofthings/2ch-hbridge-pcb/main/Render.PNG)

# Specifications

 - Channels - 2
 - Input voltage - 12V - 48V
 - Output current - 100A
 - Control input voltage - 12V
 - IO - 2x Analog in, 2x Digital in, 2x Digital out (control) 
 - Output filter type - LC

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

