UPCOMING Task:

Software:
1. Settings page in GUI
2. Compilation of Nextion 5 inch display graphics and code.
3. Dark/Light theme selector.

Hardware:
1. Designing the PCB with SMD component to make it within 100x100mm (Most PCB fabricator has a good package rate for that size)
2. Using line driver IC to release some GPIOs, Put a audio buzzer for warning tone,
3. Implimenting ALC features to feed to radio.

************************************
V 1.5
Bug fixed. Touch disabled when TX.

V 1.4
Arduino-LDMOS-SSPA-Controller-v1.4.ino
Pin assignment corrected for PCB V1.1

V 1.3
Arduino-LDMOS-SSPA-Controller-v1.3.ino
Arduino LDMOS SSPA controller with Nextion HMI display

![Display variables](./nextion%20variables.PNG)
![complete board](./pcb%20v1.1.png)

![Display variables](./display_mock.PNG)

Features:
* Arduino Nano [easy code to understand]
* Max watt, max SWR, Max Temp for Bar graph settings can be changed in Arduino code.
* Display Power output in bar graph and numeric
* Display SWR in bar graph and numeric
* Display Temp in bar graph and numeric
* Display Band in Numeric, also band button shows current display. Touch button for band selection.
* Antenna 2 selection (optional - currently no GPIO left)
* Shows FAN speed (LOW/MID/HIGH/HOT/ALRM)
* Band change by 8 pole rotary band switch (position 1 will enable touch switch).
* No Band change when TX on, if you change band switch or select different band from touch - it will change when TX goes of.
* PWM FAN control depends on temperature.
* Temp sensor DS18B20 3 pin IC
* Volt monitor.
* Current monitor from DXWORLD-E protection board (BTS.....)
* Shows SWR/High Po/High current LED status on LCD
* Shows PTT status (on air)
* Shutdown bias V at 54.9 Degree (you can set it)
* Shows operator callsign on front display (change it inside Arduino code)
* For touch band selection, band change memorize in eeprom, comes back same band after power on or reset.

If you need any clarification or help look for me in qrz.com

Enjoy, 73, S21RC
