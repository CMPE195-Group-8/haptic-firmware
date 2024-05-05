# Haptic Gloves Firmware
<p>This is the firmware that belongs inside the Sources file of a STM32CubeIDE project of the haptic gloves</br>
This is not particularily locked into a specific MicroController, but we used the STM32F411RET6. As long as the Microcontroller has 5 SPI lines, 1 UART/Serial Communication, 5 PWM pins, and 5 GPIO pins.<p>
<p> We are using the STM LIS3DH SPI IMUs to determine the finger position and MG90S for force feedback.<p> 