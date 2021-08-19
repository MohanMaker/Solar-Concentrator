# *** Laboratory-scale 2-axis tracking heliostat apparatus/solar concentrator for conceptual demonstrations of novel geoengineering innovations - sun tracking code *** #

# Overall concept:
#    - Take the current time and location and calculate the elevation and azimuth angles of the sun
#    - From the angle of the sun, calculate the desired mirror angle on both axes
#    - Translate the desired mirror angle to the angle of the stepper motor on each axis, and then to steps/microsteps
#    - Repeat many times to adjust the mirror angle and maintain a constant focal point as the sun moves across the sky
# Resources:
#    - https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/circuitpython
#    - https://learn.adafruit.com/adafruit-ds3231-precision-rtc-breakout/circuitpython
#    - https://learn.adafruit.com/adafruit-feather-m4-express-atsamd51
#    - https://www.esrl.noaa.gov/gmd/grad/solcalc/
#    - https://www.suncalc.org
#    - https://www.cerebralmeltdown.com/2011/05/30/open-source-arduino-sun-trackingheliostat-program/#more-2866
#    - https://www.celestis.com/resources/faq/what-are-the-azimuth-and-elevation-of-a-satellite/
#    - https://www.e-education.psu.edu/eme810/node/486
#    - https://en.wikipedia.org/wiki/Spherical_coordinate_system
# Explanation:
#   - https://youtu.be/XiAtvd54MEk
#   - https://youtu.be/hKjgcwxgWZw
#   - https://youtu.be/bQlk8gwO2BI
#   - https://drive.google.com/file/d/1UTaJ4zspJ_HL3aO3oPqd_n4NMm-HvsP-/view?usp=sharing
# Setup/remember:
#   - Make sure to roughly center the mirrors before homing starts (so that endstops hit in the correct area)
#   - Always orient the mirror array with the mirrors and focal point facing north
#   - The 3D printed mirror mount blocks should already be angled so that when the sun is directly above and in columns and wheen the mirrors rods are pointing straight up, the mirrors all reflect the light to one desired focal point (in this case 1 ft in front and 1355mm high)
#   - The sun doesn't move straight across the sky!! It creates an arc shape/pattern. Have to understand azimuth and elevation


# Import general libraries
import board
import time

# Import libraries for calculations
import math

# Import libraries for RTC
import busio as io  # For hardware I2C (M0 boards)
import adafruit_ds3231

# Import libraries for stepper motor
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

# Import libraries for IO (endstops)
from digitalio import DigitalInOut, Direction, Pull

# Setup use of multiple I2C devices
i2c = board.I2C()
kit = MotorKit(i2c=i2c)
rtc = adafruit_ds3231.DS3231(i2c)

# Set up both endstops
switch1 = DigitalInOut(board.D5)
switch1.direction = Direction.INPUT
switch1.pull = Pull.UP

switch2 = DigitalInOut(board.D6)
switch2.direction = Direction.INPUT
switch2.pull = Pull.UP

# Inputs/variables
my_lat = 42.38251  # Latitude (+ to N)
my_lng = -71.12081  # Longitude (+ to E)
my_time_zone = -4  # Time Zone (+ to E)
# my_currentdatetime = rtc.datetime  # date and time from rtc in struct_time format
my_currentdatetime = (2020, 9, 12, 11, 36, 0, 0, -1, -1)  # Enable for testing

# Optional: set the date and time on the RTC if you have not done so yet
if False:  # change to True to set the time and date on the RTC (should only be done once)
    rtc.datetime = time.struct_time((2020, 8, 31, 0, 6, 0, 0, -1, -1))
    # Format is: (year, mon, date, hour, min, sec, wday, yday, isdst)
    # Change the year, mon, date, hour, min, sec and weekday (0 = Monday, 6 = Sunday) to current actual values -- leave yday and isdst as -1
    print("Set date/time successful. Current date and time are:", rtc.datetime)

# Function to calculate solar elevation and azimuth angle
def solar_calculations(lat, lng, time_zone, currentdatetime):
    # Julian Day
    timesince1970 = time.mktime(currentdatetime)  # current time in seconds since Jan 1, 1970
    datesince1900 = round(timesince1970/86400 + 25569)  # current date as number of days since December 30, 1899 (convert seconds since 1970 to days, add to number of days between 8/30/1899 and 1/1/1970)
    if (round(timesince1970/86400 + 25569) - (timesince1970/86400 + 25569)) > 0:  # if round function rounded up
        datesince1900 = round(timesince1970/86400 + 25569) - 1  # round down
    # timepastlocalmidnight = (currentdatetime.tm_hour + (currentdatetime.tm_min / 60)) / 24  # As a decimal out of 1 (sheets format)
    timepastlocalmidnight = (11 + (36 / 60)) / 24  # Enable for testing
    julianday = datesince1900 + 2415018.5 + timepastlocalmidnight - (time_zone / 24)
    # print(julianday)

    # Julian Century
    juliancentury = (julianday - 2451545) / 36525
    # print(juliancentury)

    # Geom Mean Long Sun (deg)
    geommeanlongsun = (280.46646 + juliancentury * (36000.76983 + juliancentury * 0.0003032)) % 360
    # print(geommeanlongsun)

    # Geom Mean Anom Sun (deg)
    geommeananomsun = 357.52911 + juliancentury * (35999.05029 - 0.0001537 * juliancentury)
    # print(geommeananomsun)

    # Eccent Earth Orbit
    eccentearthorbit = 0.016708634 - juliancentury * (0.000042037 + 0.0000001267 * juliancentury)
    # print(eccentearthorbit)

    # Sun Eq of Ctr
    suneqofctr = math.sin(math.radians(geommeananomsun)) * (1.914602 - juliancentury * (0.004817 + 0.000014 * juliancentury)) + math.sin(math.radians(2 * geommeananomsun)) * (0.019993 - 0.000101 * juliancentury) + math.sin(math.radians(3 * geommeananomsun)) * 0.000289
    # print(suneqofctr)

    # Sun True Long (deg)
    suntruelong = geommeanlongsun + suneqofctr
    # print(suntruelong)

    # Sun True Anom (deg)
    suntrueanom = geommeananomsun + suneqofctr
    # print(suntrueanom)

    # Sun Rad Vector (AUs)
    sunradvector = (1.000001018 * (1 - eccentearthorbit * eccentearthorbit)) / (1 + eccentearthorbit * math.cos(math.radians(suntrueanom)))
    # print(sunradvector)

    # Sun App Long (deg)
    sunapplong = suntruelong - 0.00569 - 0.00478 * math.sin(math.radians(125.04 - 1934.136 * juliancentury))
    # print(sunapplong)

    # Mean Obliq Ecliptic (deg)
    meanobliqecliptic = 23 + (26 + ((21.448 - juliancentury * (46.815 + juliancentury * (0.00059 - juliancentury * 0.001813)))) / 60) / 60
    # print(meanobliqecliptic)

    # Obliq Corr (deg)
    obliqcorr = meanobliqecliptic + 0.00256 * math.cos(math.radians(125.04 - 1934.136 * juliancentury))
    # print(obliqcorr)

    # Sun Rt Ascen (deg)
    sunrtascen = math.degrees(math.atan2(math.cos(math.radians(obliqcorr)) * math.sin(math.radians(sunapplong)), math.cos(math.radians(sunapplong))))
    # print(sunrtascen)

    # Sun Declin (deg)
    sundeclin = math.degrees(math.asin(math.sin(math.radians(obliqcorr)) * math.sin(math.radians(sunapplong))))
    # print(sundeclin)

    # var y
    vary = math.tan(math.radians(obliqcorr/2)) * math.tan(math.radians(obliqcorr/2))
    # print(vary)

    # Eq of Time (minutes)
    eqoftime = 4 * math.degrees(vary * math.sin(2 * math.radians(geommeanlongsun)) - 2 * eccentearthorbit * math.sin(math.radians(geommeananomsun)) + 4 * eccentearthorbit * vary * math.sin(math.radians(geommeananomsun)) * math.cos(2 * math.radians(geommeanlongsun)) - 0.5 * vary * vary * math.sin(4 * math.radians(geommeanlongsun)) - 1.25 * eccentearthorbit * eccentearthorbit * math.sin(2 * math.radians(geommeananomsun)))
    # print(eqoftime)

    # HA Sunrise (deg)
    hasunrise = math.degrees(math.acos(math.cos(math.radians(90.833)) / (math.cos(math.radians(lat)) * math.cos(math.radians(sundeclin))) - math.tan(math.radians(lat)) * math.tan(math.radians(sundeclin))))
    # print(hasunrise)

    # Solar Noon (LST)
    solarnoon = (720 - 4 * lng - eqoftime + time_zone * 60) / 1440
    # print(solarnoon)

    # Sunrise Time (LST)
    sunrisetime = solarnoon - hasunrise * 4 / 1440
    # print(sunrisetime)

    # Sunset Time (LST)
    sunsettime = solarnoon + hasunrise * 4 / 1440
    # print(sunsettime)

    # Sunlight Duration (minutes)
    sunlightduration = 8 * hasunrise
    # print(sunlightduration)

    # True Solar Time (min)
    truesolartime = (timepastlocalmidnight * 1440 + eqoftime + 4 * lng - 60 * time_zone) % 1440
    # print(truesolartime)

    # Hour Angle (deg)
    if truesolartime/4 < 0:
        hourangle = truesolartime/4 + 180
    else:
        hourangle = truesolartime/4 - 180
    # print(hourangle)

    # Solar Zenith Angle (deg)
    solarzenithangle = math.degrees(math.acos(math.sin(math.radians(lat)) * math.sin(math.radians(sundeclin)) + math.cos(math.radians(lat)) * math.cos(math.radians(sundeclin)) * math.cos(math.radians(hourangle))))
    # print(solarzenithangle)

    # Solar Elevation Angle (deg)
    solarelevationangle = 90 - solarzenithangle
    # print(solarelevationangle)

    # Approx Atmospheric Refraction (deg)
    if solarelevationangle > 85:
        approxatmosphericrefraction = 0
    else:
        if solarelevationangle > 5:
            approxatmosphericrefraction = 58.1 / math.tan(math.radians(solarelevationangle)) - 0.07 / (math.tan(math.radians(solarelevationangle)))**3 + 0.000086 / (math.tan(math.radians(solarelevationangle)))**5
        else:
            if solarelevationangle > -0.575:
                approxatmosphericrefraction = 1735 + solarelevationangle * (-518.2 + solarelevationangle * (103.4 + solarelevationangle * (-12.79 + solarelevationangle * 0.711)))
            else:
                approxatmosphericrefraction = -20.772 / math.tan(math.radians(solarelevationangle))
    approxatmosphericrefraction = approxatmosphericrefraction / 3600
    # print(approxatmosphericrefraction)

    # Solar Elevation corrected for atm refraction (deg)
    solarelevationcorrectedforatmrefraction = solarelevationangle + approxatmosphericrefraction
    # print(solarelevationcorrectedforatmrefraction)

    # Solar Azimuth Angle (deg cw from N)
    if hourangle > 0:
        solarazimuthangle = (math.degrees(math.acos(((math.sin(math.radians(lat)) * math.cos(math.radians(solarzenithangle)) - math.sin(math.radians(sundeclin))) / (math.cos(math.radians(lat)) * math.sin(math.radians(solarzenithangle)))))) + 180) % 360
    else:
        solarazimuthangle = (540 - math.degrees(math.acos(((math.sin(math.radians(lat)) * math.cos(math.radians(solarzenithangle))) - math.sin(math.radians(sundeclin))) / (math.cos(math.radians(lat)) * math.sin(math.radians(solarzenithangle)))))) % 360
    # print(solarazimuthangle)

    # solarelevationcorrectedforatmrefraction = 45  # Enable for testing
    # solarazimuthangle = 135  # Enable for testing

    print("Current date/time: " + str(currentdatetime))  # Enable for testing
    print("Solar elevation angle: " + str(solarelevationcorrectedforatmrefraction))  # Enable for testing
    print("Solar azimuth angle: " + str(solarazimuthangle))  # Enable for testing

    # Now that we know azimuth and elevation of the sun, calculate the desired angle of the mirrors
    # Convert azimuth and elevation angle of the sun (in spherical coordinate system) to cartesian coordinates. Use unit vectors, disregarding r (the distance from sun to earth)
    # Adjust azimuth angle to be counterclockwise from the positive x axis instead of clockwise from north
    x = math.sin(math.radians(90 - solarelevationcorrectedforatmrefraction)) * math.cos(math.radians(360 - (solarazimuthangle - 90)))
    y = math.sin(math.radians(90 - solarelevationcorrectedforatmrefraction)) * math.sin(math.radians(360 - (solarazimuthangle - 90)))
    z = math.cos(math.radians(90 - solarelevationcorrectedforatmrefraction))

    # p = x*x + y*y + z*z  # Enable for testing
    # print(p)  # Enable for testing
    print("X coordinate of sun: " + str(x))  # Enable for testing
    print("Y coordinate of sun: " + str(y))  # Enable for testing
    print("Z coordinate of sun: " + str(z))  # Enable for testing

    # Angle of sun on the right-left (x) axis (assuming mirrors and focal point face north)
    rightleftangle = math.degrees(math.atan(z / x))
    print("Right-left angle of the sun: " + str(rightleftangle))  # Enable for testing
    # If righleft angle is positive, the sun is to the right (east) of the mirror array (negative x), when righleftangle is negative, the sun is to the left (west) of the mirror array (positive x)
    # Convert angle coming from group to angle coming from vertical
    # Convert this angle to degrees right (positive) or left (negative) of vertical (0 degrees) where the mirrors start so that the mirrors are pointing at the sun.
    if rightleftangle >= 0:
        rightleftangle = 90 - rightleftangle
    else:
        rightleftangle = 90 - (rightleftangle + 180)
    # print(rightleftangle)  # Enable for testing

    # Angle of sun on the front-back (y) axis (assuming mirrors and focal point face north)
    frontbackangle = math.degrees(math.atan(z / y))
    print("Front-back angle of the sun: " + str(frontbackangle))
    # When frontback angle is negative the sun is in in back (south) of the mirror array, when it the frontback angle is positive, the sun is in front (north) of the mirror array
    # Convert angle coming from group to angle coming from vertical
    # Convert this angle to degrees front (positive) or back (negative) of vertical (0 degrees) where the mirrors start so that the mirrors are pointing at the sun
    if frontbackangle >= 0:
        frontbackangle = 90 - frontbackangle
    else:
        frontbackangle = 90 - (frontbackangle + 180)
    # print(frontbackangle)  # Enable for testing

    # Convert rightleftangle and frontbackangle into the angle the mirror should actually move to. Since angle of incidence = angle of reflection
    rightleftangle = rightleftangle / 2
    frontbackangle = frontbackangle / 2
    # IS THIS RIGHT? CHECK! WHAT HAPPENS IF SUN IS RIGHT ABOVE - ahh angle is 0 so 0/2 = 0
    print("Desired right-left mirror angle: " + str(rightleftangle))  # Enable for testing
    print("Desired front-back mirror angle: " + str(frontbackangle))  # Enable for testing

    # Adjust the rightleft and frontback angles by a factor of ~2 since for every degree that the mirror rod moves, the motor has to move around 2 degrees. y = (27/13)x, y = motor angle, x = rod angle - determined with a protractor
    rightleftangle = (27 / 13) * rightleftangle
    frontbackangle = (27 / 13) * frontbackangle
    print("Motor right-left movement angle: " + str(rightleftangle))  # Enable for testing
    print("Motor front-back movement angle: " + str(frontbackangle))  # Enable for testing

    #Return final elevation, azimuth, rightleft, and frontback angles from the calculation
    return rightleftangle, frontbackangle


# Homing
# Home stepper1 with switch1 (right-left direction)
print("Homing")  # Enable for testing
while switch1.value is False:
    kit.stepper1.onestep(direction=stepper.BACKWARD, style=stepper.INTERLEAVE)
    time.sleep(0.05)
for i in range(37):  # Return to upright/90 degrees
    kit.stepper1.onestep(style=stepper.INTERLEAVE)
    time.sleep(0.05)

# Home stepper2 with switch2 (front-back direction)
while switch2.value is False:
    kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.INTERLEAVE)
    time.sleep(0.05)
for i in range(38):  # Return to upright/90 degrees
    kit.stepper2.onestep(style=stepper.INTERLEAVE)
    time.sleep(0.05)
print("Homing done")  # Enable for testing

# define variables to keep track of position of steppers
rightleftposition = 0  # Current position of stepper1 (+ = right, - = left) in degrees
frontbackposition = 0  # Current position of stepper2 (+ = front, - = back) in degrees
# the code also uses these variables from the function above
#  - rightleftangle = desired/goal right-left motor position (positive  = right, negative = left) in degrees
#  - frontbackangle = desired/goal front-back motor position (positive = front, negative = back) in degrees

while True:
    rightleftangle, frontbackangle = solar_calculations(my_lat, my_lng, my_time_zone, my_currentdatetime)  # Call the solar_calculations function to get elevation, azimuth angles, rightleft and frontback angles
    print("called fxn")
    print(math.fabs(rightleftposition - rightleftangle))
    print(math.fabs(frontbackposition - frontbackangle))
    if math.fabs(rightleftposition - rightleftangle) > 0.9:  # If the angle is changed appreciably enough that movement (one step of more) is required, prevents jittering back and forth
        # Right - left movement
        if rightleftangle < 30 and rightleftangle > -30:  # Prevent motor from moving too far
            # Moving right
            while rightleftposition < rightleftangle:  # If current position is less than (more negative/left) than desired position
                kit.stepper1.onestep(direction=stepper.BACKWARD, style=stepper.INTERLEAVE)
                rightleftposition += 0.9 # Increment the position by the degrees of one interleaved step
                time.sleep(0.02)  # Delay for proper motor movement
                # print("moving right")
            # Moving left
            while rightleftposition > rightleftangle:  # If current position is greater than (more positive/right) than desired position
                kit.stepper1.onestep(direction=stepper.FORWARD, style=stepper.INTERLEAVE)
                rightleftposition -= 0.9  # Increment the position by the degrees of one interleaved step
                time.sleep(0.02)  # Delay for proper motor movement
                # print("moving left")
    if math.fabs(frontbackposition - frontbackangle) > 0.9:
        # Front - back movement
        if frontbackangle < 30 and frontbackangle > -30:  # Prevent motor from moving too far
            # Moving forward
            while frontbackposition < frontbackangle:  # If current position is less (more negative/backward) than desired position
                kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.INTERLEAVE)
                frontbackposition += 0.9  # Increment the position by the degrees of one interleaved step
                time.sleep(0.02)  # Delay for proper motor movement
            # Moving backward
            while frontbackposition > frontbackangle:  # If the current positon is greater (for positive/forward) than desired position
                kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.INTERLEAVE)
                frontbackposition -= 0.9 # Increment the position by the degrees of one interleaved step
                time.sleep(0.02)  # Delay for proper motor movement
    time.sleep(60)  # Code loops/checks again every 1 minute
# Going forward:
#   - What happens when it becomes night time?
#   - Set artifical azimuth and elevation and/or datetime's and check that motors move correctly
#   - Does the code work when the angle is changing? Is it repetedly calling the fxn?