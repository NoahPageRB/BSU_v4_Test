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
Ensure you see the RS232 on the COM port (VS Code little port button shows you drop downs)
If you want to use VS Code for this you need to open into the ATSAMD51J19A folder, not the PSU_v4_Test folder, since PlatformIO needs to see the platform.ini file in the top level directory for the COM checker to appear

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
We will test the 4 motors first.
Connect stepper motor to x axis following wiring harness schematic
tmc_init x
you may have to send the init twice, still figuring that timing out
tmc_microstep x 8
tmc_current x 50
tmc_enable x
enable x
step x 1600 2000
Motor should make a full rotation.
disable x
Move motor to Y.
tmc_init y
tmc_microstep y 8
tmc_current y 50
tmc_enable y
enable y
step y 1600 2000
Motor should make a full rotation.
disable y
Move motor to Z1.
tmc_init z1
tmc_microstep z1 8
tmc_current z1 50
tmc_enable z1
enable z
step z 1600 2000
Motor should make a full rotation.
disable z
Move motor to Z2
tmc_init z2
tmc_microstep z2 8
tmc_current z2 50
tmc_enable z2
enable z
step z 1600 2000
Motor should make a full rotation.
disable z

Power off, next we swap the STEP / DIR lines to the TMC429 for scheduled testing
mc_init
Connect motor to x
tmc_init x
tmc_microstep x 8
tmc_current x 50
tmc_enable x
mc_limits x 100 1000 1000
enable x
mc_velocity x
mc_vtarget x 1000
motor should move
disable x
Connect motor to y
tmc_init y
tmc_microstep y 8
tmc_current y 50
tmc_enable y
mc_limits y 100 1000 1000
enable y
mc_velocity y
mc_vtarget y 1000
motor should move
disable y
Connect motor to z1 port
tmc_init z1
tmc_microstep z1 8
tmc_current z1 50
tmc_enable z1
mc_limits z 100 1000 1000
enable z
mc_velocity z
mc_vtarget z 1000
motor should move
disable z
Connect motor to z2 port
tmc_init z2
tmc_microstep z2 8
tmc_current z2 50
tmc_enable z2
mc_limits z 100 1000 1000
enable z
mc_velocity z
mc_vtarget z 1000
motor should move
disable z

Next we will check the limit switch status for both MCU and TMC429
io_init
mc_swpol high
mc_rightsw on
io_read
See all LOW and all OPEN
mc_switches
See all inactive
Take a jumper and bridge the ZEND2 sensor connector high (connect the middle pin to the pin furthest from the top of the board)
io_read
See ZEND2 TRIGGERED and rest open
mc_switches
See ZEND2 ACTIVE and rest inactive
Remove jumper on ZEND2, and repeat the process for ZEND1, YEND2, YEND1, XEND2, and XEND1
power off

Next we will flash the bootloader to the LED MCU
Connect ATMEL ICE to LED MCU watching orientation
Go through the exact same steps as before, except select the correct LED MCU. The design for v4 calls for the ATSAMD21E18A,
but due to inventory shortages some boards may have an ATSAMD21E17A. Visually confirm on the MCU the right one, and proceed.
For ATSAMD21E18A, flash bootloader-trinket_m0-v3.16.0.bin. For ATSAMD21E17A, flash bootloader-trinket_m9_e17a.bin.
Power off, connect USB C to board, then power on.
Open VSCode to ATSAMD21E17A (Regardless of if you are running the 17 or 18 version).
You should see a new com port, for Trinket M0, select it.
Due to the reduced memory of the 17A, we can't use the normal upload button via bossa like we do on the main MCU. If you are on the 18A, you can just upload like we did on the Main MCU and skip the below steps. But the below approach works for 17 or 18, so you if you aren't feeling adventurous you can just follow the instructions below. 
Double tap the reset button at the bottom of the board.
You should see a flash drive called TRINKETBOOT on your PC. You may have to try the double tap reset a few times to get this to work.
Press the trash can icon in the bottom left to clean the build artifacts
Press the checkmark icon in the bottom left to build without uploading
In the ATSAMD21E17A folder, go to .pio/build/trinket_m0.
Drag and drop the firmware.uf2 file onto the TRINKETBOOT flashdrive. The flashdrive should immediately disconnect
Press the reset button on the bottom of the board just one time.
Connect via your favorite serial monitor to this trinket COM port via 115200 baud. If you don't see it you can try single press reset, just don't double click it.

make sure LEDs are connected to the LED port
set all spin 0 255 0 255
disconnect and switch your serial to the main MCU port
led_set all spin 255 0 0 255
the LEDs should turn on

Lastly we test the RS232 port. Connect a USB to RS232 adapter to the RS232 port. Open a serial 115200 for the usbserial port on the PCB, and another serial for your USB to RS232 adapter. Send a message on one side, ensure it shows up on the other.

We are all done! Write a serial number under the Resolve logo.