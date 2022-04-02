Sample rate = 100 Hz
#Decimal places for each IMU value = 4

Power: side switch - 1x for on, 2x for off

screen display:
Always on @ low brightness
Charging and battery - same as before

SCREEN DISPLAYS

Default Screen:

Configured Body Part
#Recordings on SD card
Not Recording 

Recording Screen

Configured Body Part
Recording in Progress
#Seconds Recorded
#Lines Written

BUTTON BEHAVIOURS

BTN A
Toggle Body Part (only if not recording)
Trunk, Thigh L, Thigh R, Shank L, Shank R

BTN B
if not recording
	<set recording flag>
	<create file>
	<BEEP>

if recording
	<reset recording flag>
	<close file>

BTN C
Toggle Body Position (only if not recording)
Front Right Back Left

FUNCTIONALITY

Remove all serial functionality

GLOBAL VARIABLES

ChipSelect
BodyLocations
BodyLocIdx
SensorPlacements
SensorPlaceIdx
isRecording
isRecording_Z1
RecordTimeCounter
BeepFrequencies
BeepDurations
<all timing variables except screen timeout related ones>

SETUP

Setup LCD, IMU, Brightness, Textsize, Cursor
If no SD, print message SD CARD MISSING
Get initial number of txt files present

LOOP

Update Time, Check if elapsed time = sampling interval multiple
In Callback
	<Update battery info>
	<Handle Button Presses>
	<Handle If Recording>
	<Update and Shuffle>