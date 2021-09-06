# *** Laboratory-scale 2-axis tracking heliostat apparatus/solar concentrator for conceptual demonstrations of novel geoengineering innovations - sun tracking code *** #

# Overall concept:
#    - Take the current time and location and calculate the elevation and azimuth angles of the sun
#    - From the angle of the sun, calculate the desired rotation and elevation of the mirror array
#    - Take the current position of the tracker from the IMU and move the proper number of steps to achieve the desired angle
#    - Repeat throughout the day to adjust the mirror angle and maintain a constant focal point as the sun moves across the sky
# Resources:
#    - https://learn.adafruit.com/adafruit-feather-m4-express-atsamd51
#    - https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing/circuitpython
#    - https://learn.adafruit.com/adafruit-ds3231-precision-rtc-breakout/circuitpython
#    - https://learn.adafruit.com/adafruit-9-dof-orientation-imu-fusion-breakout-bno085
#    - https://gml.noaa.gov/grad/solcalc/
#    - https://www.suncalc.org
#    - https://greenpassivesolar.com/passive-solar/scientific-principles/movement-of-the-sun/
#    - https://www.celestis.com/resources/faq/what-are-the-azimuth-and-elevation-of-a-satellite/
#    - https://www.e-education.psu.edu/eme810/node/486
# Explanations:
#   - https://youtu.be/XiAtvd54MEk
#   - https://youtu.be/hKjgcwxgWZw
#   - https://youtu.be/bQlk8gwO2BI
#   - https://youtu.be/P4PevtijcwQ
#   - https://drive.google.com/file/d/1UTaJ4zspJ_HL3aO3oPqd_n4NMm-HvsP-/view?usp=sharing
# Setup/use:
#   - Make sure the mirror array is level before starting
#   - You can orient the mirror array in any way before powering on (the IMU will compensate), just make sure it is in a reasonable position, so the motors don't stall.
#   - The 3D printed mirror mount blocks are individually angled to determine the focal point (currently,  when the sun is directly above the mirrors reflect the light 1 ft in front and 1355mm high).
#   - The system adjusts the elevation and azimuth of the mirror array to maintain a stationary focal point as the sun moves in an arc across the sky.
#   - The focal point is will be due south from the mirror array (unless you set an offset).
#   - Modify parameters such as location and gear ratios in the "Inputs/varibles" section.
#   - Search for the "for testing" comments. You can set artifical azimuth and elevation and/or datetime's to troubleshoot.

# Import general libraries
import time
import board
import math
import busio

# Import libraries for RTC
import adafruit_ds3231

# Import libraries for stepper motor
from adafruit_motorkit import MotorKit
from adafruit_motor import stepper

# Import libraries for BNO085 IMU (UART mode)
import adafruit_bno08x
from adafruit_bno08x.uart import BNO08X_UART

# Setup use of I2C devices (motor, RTC) and UART devices (BNO085)
i2c = board.I2C()
kit = MotorKit(i2c=i2c, address=0x61) # need 0x61 instead of the default 0x60 since I soldered the pad on the back to use two motor driver boards later
rtc = adafruit_ds3231.DS3231(i2c)
uart = busio.UART(board.TX, board.RX, baudrate=3000000, receiver_buffer_size=2048)
bno = BNO08X_UART(uart)

# Enable quaternion (and more) on BNO085 IMU
bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)

# Inputs/variables
my_lat = 42  # Latitude (+ to N)
my_lng = -71  # Longitude (+ to E)
my_time_zone = -4  # Time Zone (+ to E)
# my_currentdatetime = (2020, 9, 12, 11, 36, 0, 0, -1, -1)  # Enable for testing if you want to set a custom date. Otherwise we call rtc.datetime which gets live date and time from rtc in struct_time format

rotationgearratio = 26.8512396694  # gearbox, 1:(26 103/121)
elevationgearratio = 20  # gears, 6:120
elevationmotordegreesperstep = 1.8
azimuthmotordegreesperstep = 1.8

# Optional: input offsets to focal point postion (how many degrees you want the FOCAL POINT to change)
elevationpositionoffset = 0 # in degrees (+ = more tilt lower focal point, - = less tilt higher focal point)
azimuthpositionoffset = 0 # in degrees (+ = right of south up to +180 moves focal point right, - = left of south up to -180 moves focal point left)

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
    timepastlocalmidnight = (currentdatetime.tm_hour + (currentdatetime.tm_min / 60)) / 24  # As a decimal out of 1 (sheets format)
    # timepastlocalmidnight = (11 + (36 / 60)) / 24  # Enable for testing if you set a custom time
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

    # Return solar elevation and azimuth angles
    return solarelevationcorrectedforatmrefraction, solarazimuthangle

def reflector_calculations(solarelevationcorrectedforatmrefraction, solarazimuthangle):
    # calculate desired elevation and azimuth angles for the reflector, knowing angle of incidence = angle of reflection, so need to go 1/2 way
    desiredreflectorelevation = (90 - solarelevationcorrectedforatmrefraction) / 2 # degrees from flat
    desiredreflectorazimuth = (solarazimuthangle - 180) / 2 # degrees from south
    print("Desired reflector elevation angle: " + str(desiredreflectorelevation))  # Enable for testing
    print("Desired reflector azimuth angle: " + str(desiredreflectorazimuth))  # Enable for testing

    return desiredreflectorelevation, desiredreflectorazimuth

def convert_angle_to_steps(elevation, azimuth):
    # calculate number of steps motor needs to move
    elevationsteps = round((elevation * elevationgearratio) / elevationmotordegreesperstep)
    azimuthsteps = round((azimuth * rotationgearratio) / azimuthmotordegreesperstep)

    return elevationsteps, azimuthsteps

def eulerpitchroll_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.degrees(math.asin(t2))

    return roll_x, pitch_y # in degrees

def find_heading(dqw, dqx, dqy, dqz):
    norm = math.sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz)
    dqw = dqw / norm
    dqx = dqx / norm
    dqy = dqy / norm
    dqz = dqz / norm

    ysqr = dqy * dqy

    t3 = +2.0 * (dqw * dqz + dqx * dqy)
    t4 = +1.0 - 2.0 * (ysqr + dqz * dqz)
    yaw_raw = math.atan2(t3, t4)
    yaw = yaw_raw * 180.0 / math.pi
    if yaw > 0:
        yaw = 360 - yaw
    else:
        yaw = abs(yaw)
    #adjust for degrees from south
    if 270 <= yaw <= 360:
        yaw = yaw - 450
    elif 0 <= yaw < 270:
        yaw = yaw - 90
    return yaw  # heading from south (+ = right of south up to +180, - = left of south up to -180)

time.sleep(1) # sleep for IMU
elevationposition_current = 0
azimuthposition_current = 0
while rtc.datetime.tm_hour in range(5, 21):
    # call the IMU and define variables to keep track of the current stepper position
    roll_x, pitch_y = eulerpitchroll_from_quaternion(*bno.quaternion)  # roll/pitch
    quat_i, quat_j, quat_k, quat_real = bno.quaternion  # heading/yaw
    heading = find_heading(quat_real, quat_i, quat_j, quat_k)
    elevationposition_current, azimuthposition_current = convert_angle_to_steps(pitch_y, heading)
    # elevationposition_current = Current position of elevation stepper (tilted frwd) in steps (positive = more tilted)
    # azimuthposition_current = current position of rotation stepper (+ = right of South, - = left of south) in steps

    elevationposition_desired, azimuthposition_desired = convert_angle_to_steps(*reflector_calculations(*solar_calculations(my_lat, my_lng, my_time_zone, rtc.datetime)))  # Call the solar_calculations, reflector_calculations, and convert_angle_to_steps function to get elevation, azimuth angles, and steps
    # elevationposition_desired, azimuthposition_desired = reflector_calculations(*solar_calculations(my_lat, my_lng, my_time_zone, my_currentdatetime))  # Enable for testing, if you have set a custom time above in my_currentdatetime

    # optional focal point offsets
    if elevationpositionoffset != 0 or azimuthpositionoffset != 0:
        print("Elevation position offset: " + str(elevationpositionoffset))  # Enable for testing
        print("Azimuth position offset: " + str(azimuthpositionoffset))  # Enable for testing
        elevationpositionoffset, azimuthpositionoffset = convert_angle_to_steps(elevationpositionoffset / 2, azimuthpositionoffset / 2)
        elevationposition_desired = elevationposition_desired + elevationpositionoffset
        azimuthposition_desired = azimuthposition_desired + azimuthpositionoffset

    print("Current reflector elevation angle: " + str(pitch_y))  # Enable for testing
    print("Current reflector azimuth angle: " + str(heading))  # Enable for testing

    if elevationposition_current != elevationposition_desired or azimuthposition_current != azimuthposition_desired:
        # Elevation movement
        if elevationposition_current < elevationposition_desired:  # need to angle more
            for x in range(math.fabs(elevationposition_desired - elevationposition_current)):
                kit.stepper1.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
                time.sleep(0.15)  # Delay for proper motor movement
        elif elevationposition_current > elevationposition_desired: # need to angle less
            for x in range(math.fabs(elevationposition_desired - elevationposition_current)):
                kit.stepper1.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
                time.sleep(0.15) # Delay for proper motor movement
        # Azimuth/rotation movement
        if azimuthposition_current < azimuthposition_desired: # need to rotate right
            for x in range(math.fabs(azimuthposition_desired - azimuthposition_current)):
                kit.stepper2.onestep(direction=stepper.FORWARD, style=stepper.DOUBLE)
                time.sleep(0.05) # Delay for proper motor movement
        elif azimuthposition_current > azimuthposition_desired: # need to rotate left
            for x in range(math.fabs(azimuthposition_desired - azimuthposition_current)):
                kit.stepper2.onestep(direction=stepper.BACKWARD, style=stepper.DOUBLE)
                time.sleep(0.05) # Delay for proper motor movement
    print("Done")
    time.sleep(10)  # Code loops/checks again every 10 seconds
    print("")
