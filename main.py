import sys
import vex
import math

brain = vex.Brain()
competition = vex.Competition()

controller1 = vex.Controller(vex.ControllerType.PRIMARY)
controller2 = vex.Controller(vex.ControllerType.PARTNER)

leftFront = vex.Motor(vex.Ports.PORT17, vex.GearSetting.RATIO18_1, False)
leftBack = vex.Motor(vex.Ports.PORT2, vex.GearSetting.RATIO18_1, False)
rightBack = vex.Motor(vex.Ports.PORT1, vex.GearSetting.RATIO18_1, True)
rightFront = vex.Motor(vex.Ports.PORT18, vex.GearSetting.RATIO18_1, True)
swerve = vex.Motor(vex.Ports.PORT15, vex.GearSetting.RATIO18_1, True)

catapult = vex.Motor(vex.Ports.PORT13, vex.GearSetting.RATIO36_1, True)
intake = vex.Motor(vex.Ports.PORT14, vex.GearSetting.RATIO18_1, True)

flipper = vex.Motor(vex.Ports.PORT20, vex.GearSetting.RATIO18_1, True)

vision1 = vex.Vision(vex.Ports.PORT11)
vision1.set_brightness(50)
green = vex.VisionSignature(1, -1375, -1011, -1193, -5539, -4449, -4994, 3, 0)
vision1.set_signature(green)


class PID:
    kP = 0
    kI = 0
    kD = 0

    prevTime = 0
    integral = 0
    prevValue = 0

    prevError = 0

    target = 0

    def __init__(self, p, i, d):
        self.kP = p
        self.kI = i
        self.kD = d
        self.prevTime = brain.timer.time()

    def apply(self, measured):
        current_time = brain.timer.time()

        dt = current_time - self.prevTime

        error = self.target - measured

        proportional = self.kP * error
        self.integral += self.kI * ((error + self.prevError) / 2.) * dt
        derivative = self.kD * (error - self.prevError) / dt

        self.prevTime = current_time
        self.prevError = error

        return proportional + self.integral + derivative

    def set_target(self, t):
        self.target = t


swervePID = PID(200, 0, 0)
visionPID = PID(0.33, 0.01, 0)
basePID = PID(0.33, 0.01, 0)

# Do not adjust the lines below

# Robot Mesh Studio runtime continues to run until all threads and
# competition callbacks are finished.
runIntake = 0
braking = False
catapultTarget = 0


def base(lf, lb, rb, rf):
    leftFront.spin(vex.DirectionType.FWD, lf, vex.VelocityUnits.PCT)
    leftBack.spin(vex.DirectionType.FWD, lb, vex.VelocityUnits.PCT)
    rightFront.spin(vex.DirectionType.FWD, rf, vex.VelocityUnits.PCT)
    rightBack.spin(vex.DirectionType.FWD, rb, vex.VelocityUnits.PCT)


def cant_control(a):
    swervePID.target = 3 * a / (2 * math.pi)


def vision_control():
    cant_control(math.pi / 2)
    if abs(swerve.rotation(vex.RotationUnits.REV) - (3/4)) < 0.1:
        return False

    vision1.take_snapshot(green)

    if vision1.object_count == 0:
        return True

    brain.screen.draw_rectangle(vision1.largest_object.originX,
                                vision1.largest_object.originY,
                                vision1.largest_object.width,
                                vision1.largest_object.height)

    visionPID.set_target(160)
    v = -visionPID.apply(vision1.largest_object.centerX)
    base(v, v, v, v)


def base_control(distance, velocity):
    leftBack.rotate_for(vex.DirectionType.FWD, distance / (4 * math.pi),
                        vex.RotationUnits.REV, velocity, vex.VelocityUnits.PCT)
    rightBack.rotate_for(vex.DirectionType.FWD, distance / (4 * math.pi),
                         vex.RotationUnits.REV, velocity, vex.VelocityUnits.PCT)
    rightFront.rotate_for(vex.DirectionType.FWD, distance / (4 * math.pi),
                          vex.RotationUnits.REV, velocity, vex.VelocityUnits.PCT)
    leftFront.rotate_for(vex.DirectionType.FWD, distance / (4 * math.pi),
                         vex.RotationUnits.REV, velocity, vex.VelocityUnits.PCT)


def tare_base():
    leftFront.reset_rotation()
    leftBack.reset_rotation()
    rightFront.reset_rotation()
    rightBack.reset_rotation()


def brake_base():
    leftFront.stop(vex.BrakeType.HOLD)
    leftBack.stop(vex.BrakeType.HOLD)
    rightFront.stop(vex.BrakeType.HOLD)
    rightBack.stop(vex.BrakeType.HOLD)


def unbrake_base():
    leftFront.stop(vex.BrakeType.BRAKE)
    leftBack.stop(vex.BrakeType.BRAKE)
    rightFront.stop(vex.BrakeType.BRAKE)
    rightBack.stop(vex.BrakeType.BRAKE)

def toggle_brake():
    global braking
    braking = not braking

    if braking:
        brake_base()
    else:
        unbrake_base()

def catapult_control():
    # 3:5
    if catapult.rotation(vex.RotationUnits.REV) < catapultTarget:
        catapult.spin(vex.DirectionType.FWD, 100, vex.VelocityUnits.PCT)
        return False
    else:
        catapult.stop()
        return True


def pre_auton():
    leftFront.stop(vex.BrakeType.BRAKE)
    leftBack.stop(vex.BrakeType.BRAKE)
    rightFront.stop(vex.BrakeType.BRAKE)
    rightBack.stop(vex.BrakeType.BRAKE)

    catapult.stop(vex.BrakeType.HOLD)
    catapult.reset_rotation()

    tare_base()


def autonomous():
    """base(100, 100, 100, 100)
    sys.sleep(1.1)
    base(0,0,0,0)
    sys.sleep(1)
    base(-30,-30,-30,-30)
    sys.sleep(0.7)
    base(0,0,0,0)
    sys.sleep(1)
    shoot()
    sys.sleep(1)
    base(-100,-100,-100,-100)
    sys.sleep(1.55)
    base(0,0,0,0)
    sys.sleep(0.5)
    cant_control(math.pi/2 + 0.09)
    sys.sleep(1)
    base(100,100,100,100)
    sys.sleep(3)"""

    # base(100,100,100,100)
    # sys.sleep(0.8)
    # base(0,0,0,0)
    # cant_control(math.pi/2)
    # sys.sleep(2)
    # base(100,100,100,100)
    # sys.sleep(1.8)
    # while(torque() > 10):
    #     base(100,100,100,100)
    # base(0,0,0,0)

    cant_control(math.pi/2)
    sys.sleep(2)
    base(100, 100, 100, 100)
    sys.sleep(0.6)
    base(0, 0, 0, 0)
    sys.sleep(0.5)
    cant_control(0)
    sys.sleep(2)
    base(100, 100, 100, 100)
    sys.sleep(2.7)
    base(0, 0, 0, 0)

    pass


def drivercontrol():
    # User control code here, inside the loop
    # left cant
    # right vertical fwd/rev, right horizontal turn
    while True:
        controller1.screen.print_(torque())

        c1a1 = controller1.axis1.position()  # right horizontal
        c1a2 = controller1.axis2.position()  # right vertical
        # c1a3 = controller1.axis3.position()  # left vertical
        c1a4 = controller1.axis4.position()  # left horizontal

        c2a2 = controller2.axis2.position()

        flipper.spin(vex.DirectionType.FWD, c2a2, vex.VelocityUnits.PCT)

        # the left joystick controls the swerve. There is no rotation done via this joystick.

        swerve_pos = swerve.rotation(vex.RotationUnits.REV) * 2 * math.pi / 3.

        if controller1.buttonX.pressing():
            vision_control()
        else:
            base(c1a2 + 0.6 * c1a1 * math.sin(swerve_pos + (math.pi / 4)),
                 c1a2 + 0.6 * c1a1 * math.sin(swerve_pos + (3 * math.pi / 4)),
                 c1a2 + 0.6 * c1a1 * math.sin(swerve_pos + (5 * math.pi / 4)),
                 c1a2 + 0.6 * c1a1 * math.sin(swerve_pos + (7 * math.pi / 4)))
            cant_control(0.6*math.pi*(c1a4/100))

        sys.sleep(0.02)  # Sleep the task for a short amount of time to prevent wasted resources.


def torque():
    return (leftBack.torque(vex.TorqueUnits.NM) +
            rightBack.torque(vex.TorqueUnits.NM) +
            leftFront.torque(vex.TorqueUnits.NM) +
            rightFront.torque(vex.TorqueUnits.NM))/4


def c1l1():
    pass


def c1l2():
    pass


def c1r1():
    pass


def c1r2():
    pass


def c1a():
    shoot()


def c1b():
    toggle_brake()


def c1x():
    pass


def c1y():
    pass


def c2l1():
    toggle_intake(-100)


def c2l2():
    toggle_intake(-100)


def c2r1():
    toggle_intake(100)


def c2r2():
    toggle_intake(100)


def c2a():
    shoot()


def c2b():
    toggle_brake()

def c2x():
    pass


def c2y():
    pass


def shoot():
    global catapultTarget
    catapultTarget += 5


def toggle_intake(value):
    global runIntake
    runIntake = value if runIntake != value else 0

# Do not adjust the lines below
# Set up (but don't start) callbacks for autonomous and driver control periods.
competition.autonomous(autonomous)
competition.drivercontrol(drivercontrol)

# Run the pre-autonomous function.
pre_auton()

# Robot Mesh Studio runtime continues to run until all threads and
# competition callbacks are finished.

buttonState = {}

buttons = [(controller1.buttonL1, c1l1),
           (controller1.buttonL2, c1l2),
           (controller1.buttonR1, c1r1),
           (controller1.buttonR2, c1r2),
           (controller1.buttonA, c1a),
           (controller1.buttonB, c1b),
           (controller1.buttonX, c1x),
           (controller1.buttonY, c1y),
           (controller2.buttonL1, c2l1),
           (controller2.buttonL2, c2l2),
           (controller2.buttonR1, c2r1),
           (controller2.buttonR2, c2r2),
           (controller2.buttonA, c2a),
           (controller2.buttonB, c2b),
           (controller2.buttonX, c2x),
           (controller2.buttonY, c2y)]

for button, method in buttons:
    buttonState[button] = False

while True:
    swerve.spin(vex.DirectionType.FWD, swervePID.apply(swerve.rotation(vex.RotationUnits.REV)), vex.VelocityUnits.PCT)
    intake.spin(vex.DirectionType.FWD, runIntake, vex.VelocityUnits.PCT)
    catapult_control()
    for button, method in buttons:
        if button.pressing() and not buttonState[button]:
            method()
        buttonState[button] = button.pressing()
    sys.sleep(0.02)