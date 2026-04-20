Tools
Soldering Iron
Tweezer Soldering Iron
(If using v4 and not v4.1) Hot Air Reflow Station
XGecu T48
Either Spring Loaded POGO Pin SOIC 8 Flash Adapter OR 8x Grabby Hooky things
Atmel ICE

Software
VSCode / VSCodium with PlatformIO
David Griffith's MiniPro
Atmel Studio

Steps
Solder rework
Flash 2513.bin to chip
Apply 24V
Make sure the board is connected to a USB C device (random laptop or something)
minipro -k
to verify the programmer is connected
minipro -p AT24C02C -w 2513.bin

Connect USB C cable to laptop
Ensure you see the device COM port (VS Code little port button shows you drop downs)
If you want to use VS Code for this you need to open into the ATSAMD51J19A folder, not the BSU_v4_Test folder, since PlatformIO needs to see the platformio.ini file in the top level directory for the COM checker to appear

Power off
Must use Windows for this next part
Connect the Atmel ICE to the Main MCU
Ensure orientation correct
Power on
Check green light on ATMEL ICE

Open Microchip Studio
Tools
Device Programming
ENSURE ATSAMD51J19
Apply on SWD Interface
Read Device Signature
Memories
3 Dots next to FLASH
Switch to BINARY
Select bootloader-metro_m4_revb-v3.16.0.bin
Open
Program
Look for Verifying Flash OK
Power off
Disconnect ATMEL ICE
Connect USB C Cable to Laptop
Power On
Open ATSAMD51J19A Folder in VSCode
Select port Metro M4
Press the little upload button
Success
Open your favorite serial monitor to the M4 @115200, VS Code or Arduino

Send the following commands
help
Ensure you get a response

Next, run io_init so the expanders come up (motor enables live on the
EXTRA_IO_EXPANDER now, not direct MCU GPIO):
io_init

We will test the 3 motors first. BSU has three motor axes: Z1, Z2, M3.
Z1 and Z2 have independent STEP / DIR / UART but SHARE a common EN line on
GPA6 of the EXTRA expander (they drive the same physical gantry, like the
two Z motors on a Prusa). M3 has its own EN on GPA7. The enable CLI
therefore exposes only 'z' and 'm3' as canonical axes; 'z1'/'z2' are
accepted as aliases for 'z' so the rest of the CLI stays symmetric.
Connect stepper motor to Z1 port following wiring harness schematic
tmc_init z1
you may have to send the init twice, still figuring that timing out
tmc_microstep z1 8
tmc_current z1 50
tmc_enable z1
enable z
step z1 1600 2000
Motor should make a full rotation.
Move motor to Z2 (leave 'z' enabled — same pin drives both).
tmc_init z2
tmc_microstep z2 8
tmc_current z2 50
tmc_enable z2
step z2 1600 2000
Motor should make a full rotation.
disable z
Move motor to M3.
tmc_init m3
tmc_microstep m3 8
tmc_current m3 50
tmc_enable m3
enable m3
step m3 1600 2000
Motor should make a full rotation.
disable m3

Power off, next we swap the STEP / DIR lines to the TMC429 for scheduled testing
io_init
mc_init
Connect motor to z1
tmc_init z1
tmc_microstep z1 8
tmc_current z1 50
tmc_enable z1
mc_limits z1 100 1000 1000
enable z1
mc_velocity z1
mc_vtarget z1 1000
motor should move
disable z1
Connect motor to z2
tmc_init z2
tmc_microstep z2 8
tmc_current z2 50
tmc_enable z2
mc_limits z2 100 1000 1000
enable z2
mc_velocity z2
mc_vtarget z2 1000
motor should move
disable z2
Connect motor to m3 port
tmc_init m3
tmc_microstep m3 8
tmc_current m3 50
tmc_enable m3
mc_limits m3 100 1000 1000
enable m3
mc_velocity m3
mc_vtarget m3 1000
motor should move
disable m3

Next we will check the limit switch status for both MCU and TMC429
io_init
mc_swpol high
mc_rightsw on
io_read
See all switches open and motor EN lines HIGH (disabled, active-LOW signaling)
mc_switches
See all inactive
Take a jumper and bridge the Z_TOP sensor connector high (connect the middle pin to the pin furthest from the top of the board)
io_read
See Z_TOP TRIGGERED and rest open
mc_switches
See z1 and/or z2 ACTIVE (Z_TOP is shared between z1 and z2), rest inactive
Remove jumper on Z_TOP, and repeat the process for Z_BOT, SPARE_TOP, and SPARE_BOT.
SPARE_TOP and SPARE_BOT feed only the m3 TMC429 motor.

Next we test the drawer handle. Connect the drawer-handle harness (button +
bi-color LED) and verify the button and both LED colors:
drawer_button
Press the drawer button while repeatedly running this command — it should
report PRESSED while you hold it and released when you let go.
drawer_led red
LED on the drawer handle should light RED
drawer_led green
LED on the drawer handle should light GREEN
drawer_led off
LED should turn off

Next we test the 12 solenoid outputs on TRAY_IO_EXPANDER @ 0x20. The CLI
index walks in tray/valve order:
  sol 0..3  -> T1V1..T1V4 (Tray 1, Valves 1-4)
  sol 4..7  -> T2V1..T2V4 (Tray 2, Valves 1-4)
  sol 8..11 -> T3V1..T3V4 (Tray 3, Valves 1-4)
Test them one at a time — move a single test solenoid (or listen for the
click / LED on a test harness) between the 12 ports:
sol 0 on
Verify T1V1 fired
sol 0 off
Move the test solenoid to T1V2.
sol 1 on
...repeat through sol 11 / T3V4. You can also drive all at once with:
sol_all on
sol_all off
(Careful with total current draw if you do drive all 12 simultaneously.)
power off

Next we will flash the bootloader to the LED MCU
Connect ATMEL ICE to LED MCU watching orientation
Go through the exact same steps as before, except select the correct LED MCU.
The BSU design calls for the ATSAMD21E18A — flash bootloader-trinket_m0-v3.16.0.bin.
Power off, connect USB C to board, then power on.
Open VSCode to the ATSAMD21E18A folder.
You should see a new COM port for Trinket M0; select it.
Double tap the reset button at the bottom of the board.
You should see a flash drive called TRINKETBOOT on your PC. You may have to
try the double tap reset a few times to get this to work.
Either press the upload button to flash via bossa, or drag and drop the
firmware.uf2 file (in ATSAMD21E18A/.pio/build/trinket_m0/) onto the
TRINKETBOOT flashdrive. The flashdrive should immediately disconnect.
Press the reset button on the bottom of the board just one time.
Connect via your favorite serial monitor to this trinket COM port via 115200
baud. If you don't see it you can try single press reset, just don't double
click it.

make sure LEDs are connected to the LED port
disconnect and switch your serial to the main MCU port
led_set all spin 255 0 0 255
the LEDs should turn on (red spin animation)
led_set all off 0 0 0 0
LEDs off

Automated alternative: the Python test script `bsu_test.py` drives the whole
procedure interactively, writing a JSON log. Requires `pip install pyserial`:
python bsu_test.py

We are all done! Write a serial number under the Resolve logo.
