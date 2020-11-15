# This is the v8 python code for the TinyBot_Legged controlled by the AKIFBOY via UDP
# Authored by Ahmet Akif Kaya | Aug/Sep 2020

#WELCOME MESSAGES
print(" ")
print("----------------------------")
print("Welcome")
print("This is the v8 code for the TinyBot_Legged controlled by the AKIFBOY via UDP")
print("----------------------------")
print(" ")

#UDP
import socket

UDP_IP = "192.168.1.15" #IP of the robot - same on the robot
UDP_PORT = 5005 # Port - also same on the robot

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

def helpcommands():
    print(" ")
    print("|-----Meanings of keyboard control commands-----|")
    print("----------------------------")
    print("'akifboy' = Enter the UDP Socket Control Mode, please use the AKIFBOY to control Legged!")
    print("----------------------------")
    print("'keyboardmanualcontrol' or 'k' = Enter the Keyboard Manual Control mode, and use the various keys to control the Legged. If you don't know the control layout, type 'help' after entering this mode.")
    print("----------------------------")
    print("'exit', 'quit', 'escape', 'suspense' = Quits the program")
    print("----------------------------")
    print(" ")

def keyboardmanualcontrolhelp():
    print(" ")
    print("|-----Meanings of keyboard manual control commands-----|")
    print("----------------------------")
    print("'exit', 'quit', 'escape', 'suspense' = Quits the program")
    print("----------------------------")
    print(" ")

#IMPORTMENTS
import serial
#import sys
import time
#import json
import RPi.GPIO as gpio
import random

#ARDUINO SERIAL
a = serial.Serial('/dev/ttyUSB0',9600,timeout=1)
a.flush()

#BALANCE VARIABLES
rollX0 = 0
pitchY1 = 0

#PID FOR BALANCE ROTATE
from simple_pid import PID
rpid = PID(0.1, 0.055, 0.005, setpoint=0)
ppid = PID(0.1, 0.055, 0.005, setpoint=0)
rpid.sample_time = 0
rpid.output_limits = (-10, 10)
ppid.sample_time = 0
ppid.output_limits = (-10, 10)
PIDnomovelimit = 3

#BALANCE TRANSLATE
translatelimit = 15
translatemovelimit = 3
Y0 = 0
X1 = 0

#PID FOR BALANCE TRANSLATE
ypid = PID(0.2, 0.005, 0.0003, setpoint=0)
xpid = PID(0.2, 0.005, 0.0003, setpoint=0)
ypid.sample_time = 0
ypid.output_limits = (-25, 25)
xpid.sample_time = 0
xpid.output_limits = (-25, 25)
PIDnomovelimit = 3

#GAIT PLANNER
#3D POSITIONS FOR EACH LEG
flx = 0
fly = 0
flz = 0
frx = 0
fry = 0
frz = 0
blx = 0
bly = 0
blz = 0
brx = 0
bry = 0
brz = 0
#BEZIER CURVE POINTS
x0 = -27
z0 = 0
x1 = -30
z1 = 20
x2 = 30
z2 = 20
x3 = 27
z3 = 0
x4 = 0
z4 = 0
#GAIT PLANNER POINTS
x5 = (x3-x0)/3
z5 = 0
x6 = (x0-x3)/3
z6 = 0
#BEZIER CURVE T
t = 0
#LOOP DELAY
loopdelay = 0.000001
#LEAN TO SIDES POSITIONS
y0 = 0
y1 = 10
y2 = -10
# Static Gait Planner
controlStaticGaitPlannerFirstTime = 1
controlStaticGaitPlannerGo = 0

def mapp(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)
    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

#ARDUINO COMMANDS
def fl1u():
    a.write("2")
    print(a.readline())
def fl1d():
    a.write("w")
    print(a.readline())
def fl2u():
    a.write("q")
    print(a.readline())
def fl2d():
    a.write("e")
    print(a.readline())
def fl3u():
    a.write("1")
    print(a.readline())
def fl3d():
    a.write("3")
    print(a.readline())
#---------------
def fr1u():
    a.write("5")
    print(a.readline())
def fr1d():
    a.write("t")
    print(a.readline())
def fr2u():
    a.write("r")
    print(a.readline())
def fr2d():
    a.write("y")
    print(a.readline())
def fr3u():
    a.write("4")
    print(a.readline())
def fr3d():
    a.write("6")
    print(a.readline())
#---------------
def bl1u():
    a.write("s")
    print(a.readline())
def bl1d():
    a.write("x")
    print(a.readline())
def bl2u():
    a.write("z")
    print(a.readline())
def bl2d():
    a.write("c")
    print(a.readline())
def bl3u():
    a.write("a")
    print(a.readline())
def bl3d():
    a.write("d")
    print(a.readline())
#---------------
def br1u():
    a.write("g")
    print(a.readline())
def br1d():
    a.write("b")
    print(a.readline())
def br2u():
    a.write("v")
    print(a.readline())
def br2d():
    a.write("n")
    print(a.readline())
def br3u():
    a.write("f")
    print(a.readline())
def br3d():
    a.write("h")
    print(a.readline())
#---------------
def resetdefpos():
    a.write("7")
    print(a.readline())
#---------------
def elevateu():
    a.write("+")
    print(a.readline())
def elevated():
    a.write("-")
    print(a.readline())
def backandforthu():
    a.write("*")
    print(a.readline())
def backandforthd():
    a.write("/")
    print(a.readline())
def rightandleftu():
    a.write("9")
    print(a.readline())
def rightandleftd():
    a.write("8")
    print(a.readline())
#---------------
def pitchu():
    a.write("j")
    print(a.readline())
def pitchd():
    a.write("k")
    print(a.readline())
def rollu():
    a.write("o")
    print(a.readline())
def rolld():
    a.write("p")
    print(a.readline())
def yawu():
    a.write(".")
    print(a.readline())
def yawd():
    a.write(",")
    print(a.readline())
#---------------
def elevate(leg,value): #FL:1, FR:2, BL:3, BR:4, ALL:5
    a.write("_" + str(leg) + str(value) + "\n")
    print(a.readline())
def backandforth(leg,value):
    a.write("=" + str(leg) + str(value) + "\n")
    print(a.readline())
def rightandleft(leg,value):
    a.write("&" + str(leg) + str(value) + "\n")
    print(a.readline())
#---------------
def pitch(value):
    a.write("?" + str(5) + str(value) + "\n")
    print(a.readline())
def roll(value):
    a.write("!" + str(5) + str(value) + "\n")
    print(a.readline())
def yaw(value):
    a.write(";" + str(5) + str(value) + "\n")
    print(a.readline())
#---------------
def imudatarequest():
    print(" ")
    print("Requesting the IMU data right now.")
    print(" ")
    a.write("<" + "\n")
    time.sleep(0.1)
    return a.readline()
def imudata():
    a.write(">" + "\n")
    incoming = a.readline()
    angle = incoming.split(":")
    print(angle)
    angle[0] = float(angle[0])
    angle[1] = float(angle[1][:-2])
    return angle
#---------------
#---------------
#---------------
def balancerotate():
    global rollX0
    global pitchY1
    anglen = imudata()
    if anglen[0] > 4:
        rollX0 = rollX0 - 1
    elif anglen[0] < -4:
        rollX0 = rollX0 + 1
    if anglen[1] > 4:
        pitchY1 = pitchY1 - 1
    elif anglen[1] < -4:
        pitchY1 = pitchY1 + 1
    roll(rollX0)
    pitch(pitchY1)
#---------------
def balancerotatePID():
    global rollX0
    global pitchY1
    anglen = imudata()
    if anglen[0] > PIDnomovelimit or anglen[0] < -PIDnomovelimit:
        #PID FOR ROLL
        routput = rpid(anglen[0])
        #rollX0 = routput
        rollX0 = rollX0 + routput
    if anglen[1] > PIDnomovelimit or anglen[1] < -PIDnomovelimit:
        #PID FOR PITCH
        poutput = ppid(anglen[1])
        #pitchY1 = poutput
        pitchY1 = pitchY1 + poutput
    #MOVE
    roll(rollX0)
    pitch(pitchY1)
#---------------
#---------------
def balancetranslate():
    global Y0 #Roll-rightandleft
    global X1 #Pitch-backandforth
    anglen = imudata()
    anglen[0] = anglen[0]/2
    anglen[1] = anglen[1]/2
    if anglen[0] >= translatemovelimit:
        if anglen[0] < translatelimit:
            Y0 = anglen[0]
        else:
            Y0 = translatelimit
    elif anglen[0] <= -translatemovelimit:
        if anglen[0] > -translatelimit:
            Y0 = anglen[0]
        else:
            Y0 = -translatelimit
    if anglen[1] >= translatemovelimit:
        if anglen[1] < translatelimit:
            X1 = -anglen[1]
        else:
            X1 = -translatelimit
    elif anglen[1] <= -translatemovelimit:
        if anglen[1] > -translatelimit:
            X1 = -anglen[1]
        else:
            X1 = translatelimit
    rightandleft(5,Y0)
    backandforth(5,X1)
#---------------
def balancetranslatePID():
    global Y0
    global X1
    anglen = imudata()
    if anglen[0] > PIDnomovelimit or anglen[0] < -PIDnomovelimit:
        #PID FOR Y - rightandleft
        routput = ypid(anglen[0])
        Y0 = -routput
        #Y0 = Y0 + routput
    if anglen[1] > PIDnomovelimit or anglen[1] < -PIDnomovelimit:
        #PID FOR X - backandforth
        poutput = xpid(anglen[1])
        X1 = poutput
        #X1 = X1 + poutput
    #MOVE
    rightandleft(5,Y0)
    backandforth(5,X1)
#---------------
#---------------
#---------------
def autobalancerotate():
    print(" ")
    print("Entering the Boolean Rotate Balance Mode, Type Ctrl+c to exit")
    print(" ")
    rollX0 = 0
    pitchY1 = 0
    while True:
        balancerotate()
        time.sleep(loopdelay)
def autobalancerotatePID():
    print(" ")
    print("Entering the PID Rotate Balance Mode, Type Ctrl+c to exit")
    print(" ")
    rollX0 = 0
    pitchY1 = 0
    while True:
        balancerotatePID()
        time.sleep(loopdelay)
#---------------
def autobalancetranslate():
    print(" ")
    print("Entering the Boolean Translate Balance Mode, Type Ctrl+c to exit")
    print(" ")
    Y0 = 0
    X1 = 0
    while True:
        balancetranslate()
        time.sleep(loopdelay)
def autobalancetranslatePID():
    print(" ")
    print("Entering the PID Translate Balance Mode, Type Ctrl+c to exit")
    print(" ")
    Y0 = 0
    X1 = 0
    while True:
        balancetranslatePID()
#---------------
#---------------
#---------------

#-----------------------------------------------------------------
#-----------------------------------------------------------------
#-----------------------------------------------------------------
#-----------------------------------------------------------------
def staticTakeStep(legg,speed,dir): #legg-(FL:1, FR:2, BL:3, BR:4),dir-(1:forward,-1:backward)
    #----------------------------------------------------------------- 5
    if legg == 1: #FL
        flx = x0
        flz = z0
        #-------
        frx = x5
        blx = x3
        brx = x6
        t = 0
        while True:
            flx1 = (1-t)*x0+t*x1
            flz1 = (1-t)*z0+t*z1
            flx2 = (1-t)*x1+t*x2
            flz2 = (1-t)*z1+t*z2
            flx3 = (1-t)*x2+t*x3
            flz3 = (1-t)*z2+t*z3
            flx44 = (1-t)*flx1+t*flx2
            flz44 = (1-t)*flz1+t*flz2
            flx55 = (1-t)*flx2+t*flx3
            flz55 = (1-t)*flz2+t*flz3
            flx = (1-t)*flx44+t*flx55
            flz = (1-t)*flz44+t*flz55
            #------------------------
            frx = (1-t)*x5+t*x6
            blx = (1-t)*x3+t*x5
            brx = (1-t)*x6+t*x0
            #------------------------
            t = t + speed
            backandforth(1,-flx*dir)
            elevate(1,flz)
            backandforth(2,-frx * dir)
            backandforth(3,-blx * dir)
            backandforth(4,-brx * dir)
            time.sleep(loopdelay)
            if t > 1:
                break
    #----------------------------------------------------------------- 7
    elif legg == 2: #FR
        frx = x0
        frz = z0
        #-------
        flx = x5
        blx = x6
        brx = x3
        t = 0
        while True:
            frx1 = (1-t)*x0+t*x1
            frz1 = (1-t)*z0+t*z1
            frx2 = (1-t)*x1+t*x2
            frz2 = (1-t)*z1+t*z2
            frx3 = (1-t)*x2+t*x3
            frz3 = (1-t)*z2+t*z3
            frx44 = (1-t)*frx1+t*frx2
            frz44 = (1-t)*frz1+t*frz2
            frx55 = (1-t)*frx2+t*frx3
            frz55 = (1-t)*frz2+t*frz3
            frx = (1-t)*frx44+t*frx55
            frz = (1-t)*frz44+t*frz55
            #------------------------
            flx = (1-t)*x5+t*x6
            blx = (1-t)*x6+t*x0
            brx = (1-t)*x3+t*x5
            #------------------------
            t = t + speed
            backandforth(2,-frx*dir)
            elevate(2,frz)
            backandforth(1,-flx * dir)
            backandforth(3,-blx * dir)
            backandforth(4,-brx * dir)
            time.sleep(loopdelay)
            if t > 1:
                break
    #----------------------------------------------------------------- 8
    elif legg == 3: #BL
        blx = x0
        blz = z0
        #-------
        flx = x6
        frx = x3
        brx = x5
        t = 0
        while True:
            blx1 = (1-t)*x0+t*x1
            blz1 = (1-t)*z0+t*z1
            blx2 = (1-t)*x1+t*x2
            blz2 = (1-t)*z1+t*z2
            blx3 = (1-t)*x2+t*x3
            blz3 = (1-t)*z2+t*z3
            blx44 = (1-t)*blx1+t*blx2
            blz44 = (1-t)*blz1+t*blz2
            blx55 = (1-t)*blx2+t*blx3
            blz55 = (1-t)*blz2+t*blz3
            blx = (1-t)*blx44+t*blx55
            blz = (1-t)*blz44+t*blz55
            #------------------------
            flx = (1-t)*x6+t*x0
            frx = (1-t)*x3+t*x5
            brx = (1-t)*x5+t*x6
            #------------------------
            t = t + speed
            backandforth(3,-blx*dir)
            elevate(3,blz)
            backandforth(1,-flx * dir)
            backandforth(2,-frx * dir)
            backandforth(4,-brx * dir)
            time.sleep(loopdelay)
            if t > 1:
                break
    #----------------------------------------------------------------- 6
    elif legg == 4: #BR
        brx = x0
        brz = z0
        #-------
        flx = x3
        frx = x6
        blx = x5
        t = 0
        while True:
            brx1 = (1-t)*x0+t*x1
            brz1 = (1-t)*z0+t*z1
            brx2 = (1-t)*x1+t*x2
            brz2 = (1-t)*z1+t*z2
            brx3 = (1-t)*x2+t*x3
            brz3 = (1-t)*z2+t*z3
            brx44 = (1-t)*brx1+t*brx2
            brz44 = (1-t)*brz1+t*brz2
            brx55 = (1-t)*brx2+t*brx3
            brz55 = (1-t)*brz2+t*brz3
            brx = (1-t)*brx44+t*brx55
            brz = (1-t)*brz44+t*brz55
            #------------------------
            flx = (1-t)*x3+t*x5
            frx = (1-t)*x6+t*x0
            blx = (1-t)*x5+t*x6
            #------------------------
            t = t + speed
            backandforth(4,-brx*dir)
            elevate(4,brz)
            backandforth(1,-flx * dir)
            backandforth(2,-frx * dir)
            backandforth(3,-blx * dir)
            time.sleep(loopdelay)
            if t > 1:
                break
#-----------------------------------------------------------------
#-----------------------------------------------------------------
def staticFirstSteps(speed,dir):
# -----------------------------------------------------------------1
    #FL - UP
    flx = x4
    flz = z4
    t = 0
    while True:
        flz = (1-t)*z4+t*z2
        t = t + speed
        elevate(1,flz)
        time.sleep(loopdelay)
        if t > 1:
            break
    #FL - MOVE
    flx = x4
    flz = z2
    t = 0
    while True:
        flx = (1-t)*x4+t*x3
        t = t + speed
        backandforth(1,-flx*dir)
        time.sleep(loopdelay)
        if t > 1:
            break
    #FL - DOWN
    flx = x3
    flz = z2
    t = 0
    while True:
        flz = (1-t)*z2+t*z3
        t = t + speed
        elevate(1,flz)
        time.sleep(loopdelay)
        if t > 1:
            break
# -----------------------------------------------------------------2
    #BR - UP
    brx = x4
    brz = z4
    t = 0
    while True:
        brz = (1-t)*z4+t*z2
        t = t + speed
        elevate(4,brz)
        time.sleep(loopdelay)
        if t > 1:
            break
    #BR - MOVE
    brx = x4
    brz = z2
    flx = x3
    t = 0
    while True:
        brx = (1-t)*x4+t*x3
        flx = (1-t)*x3+t*x5
        t = t + speed
        backandforth(4,-brx*dir)
        backandforth(1,-flx*dir)
        time.sleep(loopdelay)
        if t > 1:
            break
    #BR - DOWN
    brx = x3
    brz = z2
    t = 0
    while True:
        brz = (1-t)*z2+t*z3
        t = t + speed
        elevate(4,brz)
        time.sleep(loopdelay)
        if t > 1:
            break
# -----------------------------------------------------------------3
    #FR - UP
    frx = x4
    frz = z4
    t = 0
    while True:
        frz = (1-t)*z4+t*z2
        t = t + speed
        elevate(2,frz)
        time.sleep(loopdelay)
        if t > 1:
            break
    #FR - MOVE
    frx = x4
    frz = z2
    flx = x5
    brx = x3
    t = 0
    while True:
        frx = (1-t)*x4+t*x3
        flx = (1-t)*x5+t*x6
        brx = (1-t)*x3+t*x5
        t = t + speed
        backandforth(2,-frx*dir)
        backandforth(1,-flx*dir)
        backandforth(4,-brx*dir)
        time.sleep(loopdelay)
        if t > 1:
            break
    #FR - DOWN
    frx = x3
    frz = z2
    t = 0
    while True:
        frz = (1-t)*z2+t*z3
        t = t + speed
        elevate(2,frz)
        time.sleep(loopdelay)
        if t > 1:
            break
# -----------------------------------------------------------------4
    #BL - UP
    blx = x4
    blz = z4
    t = 0
    while True:
        blz = (1-t)*x4+t*z2
        t = t + speed
        elevate(3,blz)
        time.sleep(loopdelay)
        if t > 1:
            break
    #BL - MOVE
    blx = x4
    blz = z2
    flx = x6
    brx = x5
    frx = x3
    t = 0
    while True:
        blx = (1-t)*x4+t*x3
        flx = (1-t)*x6+t*x0
        brx = (1-t)*x5+t*x6
        frx = (1-t)*x3+t*x5
        t = t + speed
        backandforth(3,-blx*dir)
        backandforth(1,-flx*dir)
        backandforth(4,-brx*dir)
        backandforth(2,-frx*dir)
        time.sleep(loopdelay)
        if t > 1:
            break
    #BL - DOWN
    blx = x3
    blz = z2
    t = 0
    while True:
        blz = (1-t)*z2+t*z3
        t = t + speed
        elevate(3,blz)
        time.sleep(loopdelay)
        if t > 1:
            break
#-----------------------------------------------------------------
#-----------------------------------------------------------------
def staticLeanToSides(speed,dir): #0:right,1:left and 2:from start
    if dir == 2:
        fly = y0
        t = 0
        while True:
            fly = (1-t)*y0+t*y1
            t = t + speed
            rightandleft(5,fly)
            time.sleep(loopdelay)
            if t > 1:
                break
    elif dir == 0:
        fly = y2
        t = 0
        while True:
            fly = (1-t)*y2+t*y1
            t = t + speed
            rightandleft(5,fly)
            time.sleep(loopdelay)
            if t > 1:
                break
    elif dir == 1:
        fly = y1
        t = 0
        while True:
            fly = (1-t)*y1+t*y2
            t = t + speed
            rightandleft(5,fly)
            time.sleep(loopdelay)
            if t > 1:
                break
#-----------------------------------------------------------------
#-----------------------------------------------------------------
def staticGaitPlanner():
    staticFirstSteps(0.1,1)
    staticLeanToSides(0.35,2)
    while True:
        staticTakeStep(1,0.1,1)
        staticLeanToSides(0.35,1)
        staticTakeStep(4,0.1,1)
        staticTakeStep(2,0.1,1)
        staticLeanToSides(0.35,0)
        staticTakeStep(3,0.1,1)
def controlStaticGaitPlanner(speed,leanspeed,dir):
    if controlStaticGaitPlannerFirstTime == 1:
        staticFirstSteps(speed, dir)
        staticLeanToSides(leanspeed, 2)

    staticTakeStep(1, speed, dir)
    staticLeanToSides(leanspeed, 1)
    staticTakeStep(4, speed, dir)
    staticTakeStep(2, speed, dir)
    staticLeanToSides(leanspeed, 0)
    staticTakeStep(3, speed, dir)
#-----------------------------------------------------------------
#-----------------------------------------------------------------
#-----------------------------------------------------------------

#GETTING START MESSAGES IF ARDUINO RESETS ITSELF - TO BE ON THE SAFE SIDE
time.sleep(1)
print(a.readline())
time.sleep(0.1)
print(a.readline())
time.sleep(0.1)
print(a.readline())
time.sleep(0.1)
print(a.readline())
time.sleep(0.1)
print(a.readline())
time.sleep(0.1)
print(a.readline())
time.sleep(0.1)
print(a.readline())

#DEBUGGING AND CONTROLLING REMOTELY
while True:
    y = raw_input("Command: ")
    if y == "exit" or y == "quit" or y == "escape" or y == "suspense":
        print(" ")
        print("Quitting the main program, hope to see you soon -Legged")
        print(" ")
        break
    elif y == "help":
        helpcommands()
        continue
    elif y == "akifboy":
        print(" ")
        print("Entering the UDP Socket Remote Control Mode, please use the AKIFBOY to control me! Type CTRL+C to exit. -Legged")
        print(" ")
        controlmode = 0 # 0:Translation, 1:Rotation
        print("Controlmode: " + str(controlmode) + " (Translation)")
        print(" ")
        leg = 5 #FL:1, FR:2, BL:3, BR:4, ALL:5
        print("ControlLEG: " + str(leg) + " (FL:1, FR:2, BL:3, BR:4, ALL:5)")
        print("(Controlleg only matters for translational movements.)")
        print(" ")
        while True:
            data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
            if data[0] == "0" or data[0] == "1":
                command = data
                value = 0
            else:
                command = data[0:3]
                value = int(data[3:])
            if command == "1_four":
                controlmode = 1
                print("Controlmode: " + str(controlmode) + " (Rotation)")
            elif command == "1_three":
                controlmode = 0
                print("Controlmode: " + str(controlmode) + " (Translation)")
            elif command == "1_one":
                controlmode = 2
                print("Controlmode: " + str(controlmode) + " (Static Gait (Walking))")
            elif command == "1_right_bumper":
                print(" ")
                print("Resetting all the servos to their defaul positions.")
                print(" ")
                resetdefpos()
                if controlmode == 2:
                    controlStaticGaitPlannerGo = 0
            elif command == "1_left_bumper":
                if leg < 5:
                    leg = leg + 1
                else:
                    leg = 1
                print("ControlLEG: " + str(leg) + " (FL:1, FR:2, BL:3, BR:4, ALL:5)")
                print("(ControlLEG only matters for translational movements.)")
            elif command == "lup":
                if controlmode == 0:
                    print("backandforth:forward")
                    value = value - 128
                    value = mapp(value,0,128,0,20)
                    print("value: " + str(value))
                    backandforth(leg,value)
                elif controlmode == 1:
                    print("pitch:up")
                    value = value - 128
                    value = mapp(value,0,128,0,20)
                    print("value: " + str(value))
                    pitch(value)
                else:
                    print("Static Gait:forward")
                    value = value - 128
                    value = mapp(value, 0, 128, 0, 70)
                    value = value/100
                    print("value: " + str(value))
                    controlStaticGaitPlannerGo = 1
                    controlStaticGaitPlannerFirstTime = 0
            elif command == "ldo":
                if controlmode == 0:
                    print("backandforth:backward")
                    value = value - 128
                    value = mapp(value,0,128,0,20)
                    print("value: " + str(value))
                    backandforth(leg,value)
                elif controlmode == 1:
                    print("pitch:down")
                    value = value - 128
                    value = mapp(value,0,128,0,20)
                    print("value: " + str(value))
                    pitch(value)
                else:
                    print("Static Gait:backward")
                    value = value - 128
                    value = mapp(value, 0, 128, 0, 70)
                    value = value/100
                    print("value: " + str(value))
                    controlStaticGaitPlannerGo = -1
                    controlStaticGaitPlannerFirstTime = 0
            elif command == "lri":
                if controlmode == 0:
                    print("rightandleft:right")
                    value = value - 128
                    value = mapp(value,0,128,0,20)
                    print("value: " + str(value))
                    rightandleft(leg,value)
                elif controlmode == 1:
                    print("roll:right")
                    value = -(value - 128)
                    value = -mapp(-value,0,128,0,20)
                    print("value: " + str(value))
                    roll(value)
            elif command == "lle":
                if controlmode == 0:
                    print("rightandleft:left")
                    value = value - 128
                    value = mapp(value,0,128,0,20)
                    print("value: " + str(value))
                    rightandleft(leg,value)
                elif controlmode == 1:
                    print("roll:left")
                    value = -(value - 128)
                    value = -mapp(-value,0,128,0,20)
                    print("value: " + str(value))
                    roll(value)
            elif command == "rri":
                if controlmode == 0:
                    print("elevate:up")
                    value = value - 128
                    value = mapp(value,0,128,0,20)
                    print("value: " + str(value))
                    elevate(leg,value)
                elif controlmode == 1:
                    print("yaw:right")
                    value = -(value - 128)
                    value = -mapp(-value,0,128,0,20)
                    print("value: " + str(value))
                    yaw(value)
            elif command == "rle":
                if controlmode == 0:
                    print("elevate:down")
                    value = value - 128
                    value = mapp(value,0,128,0,20)
                    print("value: " + str(value))
                    elevate(leg,value)
                elif controlmode == 1:
                    print("yaw:left")
                    value = -(value - 128)
                    value = -mapp(-value,0,128,0,20)
                    print("value: " + str(value))
                    yaw(value)
            else:
                print("UDP Socket ERROR: Unknown Message")
                controlStaticGaitPlannerFirstTime = 1
            if controlStaticGaitPlannerGo == 1 or controlStaticGaitPlannerGo == -1:
                controlStaticGaitPlanner(value, 0.35, controlStaticGaitPlannerGo)
    elif y == "keyboardmanualcontrol" or y == "k":
        print(" ")
        print("You have entered the keyboard manual control mode. Exit commands are the same. -Legged")
        print(" ")
        while True:
            y = raw_input("Keyboard Manual Control Command: ")
            if y == "exit" or y == "quit" or y == "escape" or y == "suspense":
                print(" ")
                print("Quitting the Keyboard Manual Control Mode -Legged")
                print(" ")
                break
#-----------------------------------------
#-----------------------------------------
            elif y == "2":
                print("fl1u")
                fl1u() #Means to move the first servo on the front left leg 1 positive angle
            elif y == "w":
                print("fl1d")
                fl1d()
            elif y == "q":
                print("fl2u")
                fl2u()
            elif y == "e":
                print("fl2d")
                fl2d()
            elif y == "1":
                print("fl3u")
                fl3u()
            elif y == "3":
                print("fl3d")
                fl3d()
#------------------------------------------
            elif y == "5":
                print("fr1u")
                fr1u()
            elif y == "t":
                print("fr1d")
                fr1d()
            elif y == "r":
                print("fr2u")
                fr2u()
            elif y == "y":
                print("fr2d")
                fr2d()
            elif y == "4":
                print("fr3u")
                fr3u()
            elif y == "6":
                print("fr3d")
                fr3d()
#-----------------------------------------
            elif y == "s":
                print("bl1u")
                bl1u()
            elif y == "x":
                print("bl1d")
                bl1d()
            elif y == "z":
                print("bl2u")
                bl2u()
            elif y == "c":
                print("bl2d")
                bl2d()
            elif y == "a":
                print("bl3u")
                bl3u()
            elif y == "d":
                print("bl3d")
                bl3d()
#----------------------------------------
            elif y == "g":
                print("br1u")
                br1u()
            elif y == "b":
                print("br1d")
                br1d()
            elif y == "v":
                print("br2u")
                br2u()
            elif y == "n":
                print("br2d")
                br2d()
            elif y == "f":
                print("br3u")
                br3u()
            elif y == "h":
                print("br3d")
                br3d()
#--------------------------------------
            elif y == "+":
                print("elevateu")
                elevateu()
            elif y == "-":
                print("elevated")
                elevated()
            elif y == "*":
                print("backandforthu")
                backandforthu()
            elif y == "/":
                print("backandforthd")
                backandforthd()
            elif y == "9":
                print("rightandleftu")
                rightandleftu()
            elif y == "8":
                print("rightandleftd")
                rightandleftd()
#--------------------------------------
            elif y == "j":
                print("pitchu")
                pitchu()
            elif y == "k":
                print("pitchd")
                pitchd()
            elif y == "o":
                print("rollu")
                rollu()
            elif y == "p":
                print("rolld")
                rolld()
            elif y == ".":
                print("yawu")
                yawu()
            elif y == ",":
                print("yawd")
                yawd()
#---------------------------------------
#---------------------------------------
            elif y == "help":
                keyboardmanualcontrolhelp()
                continue
            elif y == "rdp" or y == "resetpos":
                print(" ")
                print("Resetting all the servos to their defaul positions.")
                print(" ")
                resetdefpos()
                continue
            elif y == "<":
                print(imudatarequest())
                continue
            elif y == ">":
                print(" ")
                print("Requesting an IMU short data right now.")
                print(" ")
                anglen = imudata()
                print(anglen)
                continue
            elif y == "|":
                autobalancerotate()
            elif y == "||":
                autobalancerotatePID()
            elif y == "|||":
                autobalancetranslate()
            elif y == "||||":
                autobalancetranslatePID()
            elif y == "aa":
                staticGaitPlanner()
            else:
                print(" ")
                print("This command does not exist please try another or check the guide above")
                print(" ")
                continue
    else:
        print("This command does not exist please try another or check the guide above")
        continue
    print(a.readline())
    print(" ")
