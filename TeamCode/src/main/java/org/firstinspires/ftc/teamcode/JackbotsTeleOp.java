package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

/** Field-relative drive with intake, encoder-controlled goal aiming, and flywheels. */
@Configurable
@TeleOp(name = "JackbotsTeleOp")
public class JackbotsTeleOp extends LinearOpMode {
    // Negative power preserves the front flywheel's spin direction.
    private static final double FLY_FRONT_POWER = -1.0;
    // Live in Panels under JackbotsTeleOp; negative RPM preserves firing direction.
    public static volatile double FLY_BACK_TARGET_RPM = -1000.0;

    // Same Expansion Hub port and gearing as TurretAimTest. Do not reset the
    // shared motor channel: read its encoder directly and keep a software zero.
    private static final int TURRET_ENCODER_PORT = 0;

    // Live in Panels under JackbotsTeleOp. Error is degrees, time is seconds.
    // F is friction feedforward in the direction of error; speeds are servo power (0..1).
    public static volatile double TURRET_P = 0.005;
    public static volatile double TURRET_I = 0.0;
    public static volatile double TURRET_D = 0.0;
    public static volatile double TURRET_F = 0.11;
    public static volatile double TURRET_MIN_SPEED = 0.1;
    public static volatile double TURRET_MAX_SPEED = 1.0;
    // Automatic aiming stops within +/- this many degrees of IdealTheta.
    public static volatile double TURRET_TOLERANCE_DEGREES = 3.0;

    private static TurretAimController.Settings turretSettings() {
        // PIDF covers the entire +/-180 degree travel range (up to 360 degrees of error).
        double maxSpeed = TURRET_MAX_SPEED;
        return new TurretAimController.Settings(TURRET_P, TURRET_I, TURRET_D, TURRET_F,
                TURRET_MIN_SPEED, maxSpeed, 0.2, maxSpeed, 360.0, TURRET_TOLERANCE_DEGREES);
    }

    // Field origin is the bottom-left corner; the robot starts inset from that corner.
    private static final double START_X_MM = 198.0;
    private static final double START_Y_MM = -217.4;
    private static final double TURRET_OFFSET_X_MM = 48.0;
    private static final double GOAL_ONE_X_MM = 1397.6;
    private static final double GOAL_TWO_X_MM = 2260.0;
    private static final double GOAL_Y_MM = -1471.0;
    private static final double GOAL_SWITCH_X_MM = 1829.0;

    // TODO: replace all four placeholders with measured up/down positions.
    private static final double INTAKE_LEFT_UP_POSITION = 0.73;
    private static final double INTAKE_RIGHT_UP_POSITION = 0.00;
    private static final double INTAKE_LEFT_DOWN_POSITION = 0.64;
    private static final double INTAKE_RIGHT_DOWN_POSITION = 0.46;

    // Front: geared 1150 RPM goBILDA Yellow Jacket (nominally 5.2:1). Back: direct-drive 6000 RPM.
    private static final double FLY_FRONT_MOTOR_GEARING = 1.0 + 46.0 / 11.0;
    private static final double FLY_FRONT_MOTOR_MAX_RPM = 1150.0;
    private static final double FLY_BACK_MOTOR_GEARING = 1.0;
    private static final double FLY_BACK_MOTOR_MAX_RPM = 6000.0;
    private static final double FLYWHEEL_TICKS_PER_MOTOR_REV = 28.0;
    // REV Hub velocity gains for the back flywheel. I and D are held at zero.
    // Copy tuned P/F values from PIDTuning; F starts from the back motor's rated speed.
    private static final double FLY_BACK_P = 100;
    private static final double FLY_BACK_F = 32767.0 / (FLY_BACK_MOTOR_MAX_RPM
            * FLYWHEEL_TICKS_PER_MOTOR_REV * FLY_BACK_MOTOR_GEARING / 60.0);
    // Flywheel revolutions per gearbox output revolution.
    private static final double FLYWHEEL_RATIO = 1.0;
    private static final double HEADING_OFFSET_DEGREES = 0.0;
    // Matches the tested direction in RollerTest; the right motor is reversed.
    private static final double INTAKE_ROLLER_POWER = -1.0;
    // Triggers act as buttons: above this threshold runs the rollers at full power.
    private static final double ROLLER_TRIGGER_THRESHOLD = 0.10;
    // ColorSenseTest calibration, applied to every hue.
    private static final float COLOR_GAIN = 2.0f;
    private static final float MIN_COLOR_SIGNAL = 0.005f;
    private static final float MIN_COLOR_SATURATION = 0.20f;
    // RGBTest uses 500–2500 us PWM, with off at 500 us and green at 1500 us.
    private static final double RGB_OFF = 0.0;
    private static final double RGB_GREEN = 0.5;

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor rollLeft, rollRight;
    private DcMotorEx flyFront, flyBack;
    private Servo intakeLeft, intakeRight;
    private CRServo leftTurretServo, rightTurretServo;
    private GoBildaPinpointDriver odo;
    private List<LynxModule> hubs;
    private LynxModule turretEncoderHub;
    private final ElapsedTime pinpointStartupClock = new ElapsedTime();
    private final StartingPose startingPose = new StartingPose();
    // Preserve field coordinates when A resets the displayed local odometry X/Y.
    private double fieldOriginXMM, fieldOriginYMM;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontLeftName.get());
        frontRight = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontRightName.get());
        backLeft = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backLeftName.get());
        backRight = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backRightName.get());
        rollLeft = hardwareMap.get(DcMotor.class, "rollLeft");
        rollRight = hardwareMap.get(DcMotor.class, "rollRight");
        flyFront = hardwareMap.get(DcMotorEx.class, "flyFront");
        flyBack = hardwareMap.get(DcMotorEx.class, "flyBack");
        intakeLeft = hardwareMap.get(Servo.class, "intakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "intakeRight");
        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        ServoImplEx[] indicators = {
                hardwareMap.get(ServoImplEx.class, "LeftRGB"),
                hardwareMap.get(ServoImplEx.class, "RightRGB")
        };
        // Keep the configuration names (including spelling) from CRServosTest.
        leftTurretServo = hardwareMap.get(CRServo.class, "leftTurretServo");
        rightTurretServo = hardwareMap.get(CRServo.class, "rightTurretServo");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());

        DcMotor[] allMotors = {
                frontLeft, frontRight, backLeft, backRight,
                rollLeft, rollRight, flyFront, flyBack
        };

        try {
            colorSensor.setGain(COLOR_GAIN);
            for (ServoImplEx indicator : indicators) {
                indicator.setDirection(Servo.Direction.FORWARD);
                indicator.setPwmRange(new PwmControl.PwmRange(500, 2500));
                indicator.setPosition(RGB_OFF);
            }
            leftTurretServo.setPower(0.0);
            rightTurretServo.setPower(0.0);
            // Share the hardware configuration used by Pedro Auto Tune and autonomous.
            configurePowerMotor(frontLeft, Constants.drivetrainConfig.frontLeftDirection.get());
            configurePowerMotor(backLeft, Constants.drivetrainConfig.backLeftDirection.get());
            configurePowerMotor(frontRight, Constants.drivetrainConfig.frontRightDirection.get());
            configurePowerMotor(backRight, Constants.drivetrainConfig.backRightDirection.get());
            configurePowerMotor(rollLeft, DcMotor.Direction.FORWARD);
            configurePowerMotor(rollRight, DcMotor.Direction.REVERSE);
            configureFlywheel(flyFront, FLY_FRONT_MOTOR_MAX_RPM, FLY_FRONT_MOTOR_GEARING,
                    DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            configureFlywheel(flyBack, FLY_BACK_MOTOR_MAX_RPM, FLY_BACK_MOTOR_GEARING,
                    DcMotor.RunMode.RUN_USING_ENCODER);
            flyBack.setVelocityPIDFCoefficients(FLY_BACK_P, 0.0, 0.0, FLY_BACK_F);
            configurePinpoint();
            configureTurretEncoder();
            ElapsedTime clock = new ElapsedTime();
            TurretTracking turret = new TurretTracking(sampleSensors(), gamepad1.dpad_down);

            telemetry.setMsTransmissionInterval(100);
            telemetry.addLine("Left stick: field-relative drive | Right stick X: turn");
            telemetry.addLine("A: reset local X/Y (preserve field aim) | LT: intake | RT: outtake");
            telemetry.addLine("LB/RB: manual turret | Both: stop/clear fault | D-pad Up: toggle auto aim");
            telemetry.addData("Left turret connection", leftTurretServo.getConnectionInfo());
            telemetry.addData("Right turret connection", rightTurretServo.getConnectionInfo());
            telemetry.addLine("X: toggle flywheels on/off | Y: toggle intake up/down");
            telemetry.addLine("Intake starts DOWN. One full second of color raises intake and stops rollers.");
            telemetry.addLine("RGBs are green while color is detected, off when clear.");
            telemetry.addLine("Rollers work only with the intake DOWN.");
            telemetry.addLine("Keep robot still during INIT for odometry calibration.");
            telemetry.update();
            while (opModeInInit()) {
                turret.initSample(sampleSensors(), gamepad1.dpad_down);
                Pose2D pose = odo.getPosition();
                telemetry.addLine("INIT: keep robot still until odometry calibration finishes.");
                telemetry.addLine("Align turret straight-ahead mark, then press D-pad Down to set zero.");
                telemetry.addLine("Auto aim starts after START once zero is set and odometry is READY.");
                telemetry.addData("Odometry status", odo.getDeviceStatus());
                telemetry.addData("Starting field pose applied", startingPose.initialized);
                telemetry.addData("Distance (m) X", "%.3f", pose.getX(DistanceUnit.METER));
                telemetry.addData("Distance (m) Y", "%.3f", pose.getY(DistanceUnit.METER));
                telemetry.addData("Heading (Degrees)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
                calculateAndReportTurretTarget(pose);
                addTurretTelemetry(turret);
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            turret.onStart(gamepad1.dpad_down, gamepad1.dpad_up);

            intakeLeft.setPosition(INTAKE_LEFT_DOWN_POSITION);
            intakeRight.setPosition(INTAKE_RIGHT_DOWN_POSITION);

            // A held while pressing START must be released/re-pressed to reset X/Y.
            boolean previousA = gamepad1.a, previousX = false, previousY = false;
            boolean flywheelsEnabled = false;
            boolean intakeUp = false;
            ColorConfirmation colorConfirmation = new ColorConfirmation();
            double frontTicksPerSecondPerRPM = flyFront.getMotorType().getTicksPerRev()
                    / (60.0 * FLYWHEEL_RATIO);
            double backTicksPerSecondPerRPM = flyBack.getMotorType().getTicksPerRev()
                    / (60.0 * FLYWHEEL_RATIO);

            while (opModeIsActive()) {
                boolean a = gamepad1.a;
                boolean x = gamepad1.x;
                boolean y = gamepad1.y;
                boolean leftBumper = gamepad1.left_bumper;
                boolean rightBumper = gamepad1.right_bumper;
                boolean zeroTurret = gamepad1.dpad_down;
                boolean toggleTracking = gamepad1.dpad_up;
                boolean leftTrigger = gamepad1.left_trigger > ROLLER_TRIGGER_THRESHOLD;
                boolean rightTrigger = gamepad1.right_trigger > ROLLER_TRIGGER_THRESHOLD;

                int turretEncoderTicks = sampleSensors();
                if (!startingPose.initialized) {
                    // START can arrive before calibration finishes. Keep outputs at
                    // their initialized zero power until the field pose is read back.
                    turret.initSample(turretEncoderTicks, zeroTurret);
                    turret.onStart(zeroTurret, toggleTracking);
                    previousA = a;
                    previousX = x;
                    previousY = y;
                    telemetry.addLine("Keep robot still: waiting for starting field pose (198, -217.4 mm).");
                    telemetry.addData("Odometry status", odo.getDeviceStatus());
                    telemetry.update();
                    idle();
                    continue;
                }
                Pose2D pose = odo.getPosition();
                if (a && !previousA) {
                    // Reset distance once per press without changing field-drive alignment
                    // or moving the field goals / goal-selection boundary.
                    fieldOriginXMM += pose.getX(DistanceUnit.MM);
                    fieldOriginYMM += pose.getY(DistanceUnit.MM);
                    pose = new Pose2D(DistanceUnit.METER, 0.0, 0.0,
                            AngleUnit.RADIANS, pose.getHeading(AngleUnit.RADIANS));
                    odo.setPosition(pose);
                }

                double heading = pose.getHeading(AngleUnit.RADIANS)
                        + Math.toRadians(HEADING_OFFSET_DEGREES);
                double fieldForward = -gamepad1.left_stick_y;
                double fieldRight = gamepad1.left_stick_x;
                double robotForward = fieldForward * Math.cos(heading)
                        - fieldRight * Math.sin(heading);
                double robotRight = fieldForward * Math.sin(heading)
                        + fieldRight * Math.cos(heading);
                drive(robotForward, robotRight, gamepad1.right_stick_x);

                boolean colorDetected = detectsColor(colorSensor.getNormalizedColors());
                boolean colorConfirmed = colorConfirmation.update(colorDetected, clock.seconds());
                // Indicators follow the current reading, not the delayed/latching intake state.
                for (ServoImplEx indicator : indicators) {
                    indicator.setPosition(colorDetected ? RGB_GREEN : RGB_OFF);
                }

                if (colorConfirmed) {
                    intakeUp = true;
                } else if (y && !previousY) {
                    intakeUp = !intakeUp;
                }

                double rollerPower = 0.0;
                if (!intakeUp && leftTrigger != rightTrigger) {
                    rollerPower = leftTrigger ? INTAKE_ROLLER_POWER : -INTAKE_ROLLER_POWER;
                }
                rollLeft.setPower(rollerPower);
                rollRight.setPower(rollerPower);
                intakeLeft.setPosition(intakeUp ? INTAKE_LEFT_UP_POSITION : INTAKE_LEFT_DOWN_POSITION);
                intakeRight.setPosition(intakeUp ? INTAKE_RIGHT_UP_POSITION : INTAKE_RIGHT_DOWN_POSITION);

                double idealTheta = calculateAndReportTurretTarget(pose);
                double turretPower = turret.update(turretEncoderTicks, clock.seconds(),
                        odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY,
                        idealTheta, zeroTurret, toggleTracking, leftBumper, rightBumper, turretSettings());
                // Both CR servos drive one turret and share the same controller output.
                leftTurretServo.setPower(turretPower);
                rightTurretServo.setPower(turretPower);

                if (x && !previousX) {
                    flywheelsEnabled = !flywheelsEnabled;
                }
                double frontFlywheelPower = flywheelsEnabled ? FLY_FRONT_POWER : 0.0;
                flyFront.setPower(frontFlywheelPower);
                if (flywheelsEnabled) {
                    flyBack.setVelocity(FLY_BACK_TARGET_RPM * backTicksPerSecondPerRPM);
                } else {
                    flyBack.setPower(0.0);
                }

                telemetry.addData("Color detected", colorDetected);
                telemetry.addData("Color confirmed (1 second)", colorConfirmed);
                telemetry.addData("Intake", intakeUp ? "UP" : "DOWN");
                telemetry.addData("Distance (m) X", "%.3f", pose.getX(DistanceUnit.METER));
                telemetry.addData("Distance (m) Y", "%.3f", pose.getY(DistanceUnit.METER));
                telemetry.addData("Heading (Degrees)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
                addTurretTelemetry(turret);
                telemetry.addData("Odometry status", odo.getDeviceStatus());
                telemetry.addLine("D-pad Down: set straight zero | D-pad Up: toggle auto aim / retry");
                telemetry.addLine("LB/RB: manual (cancels auto) | Both: stop/clear fault");
                telemetry.addData("Turret LB / RB", "%b / %b", leftBumper, rightBumper);
                telemetry.addData("Turret commanded power (both servos)", "%.2f", turretPower);
                telemetry.addData("flyFront power", "%.2f", frontFlywheelPower);
                telemetry.addData("flyBack Target rpm", "%.0f", FLY_BACK_TARGET_RPM);
                telemetry.addData("flyFront rpm", "%.1f", flyFront.getVelocity() / frontTicksPerSecondPerRPM);
                telemetry.addData("flyBack rpm", "%.1f", flyBack.getVelocity() / backTicksPerSecondPerRPM);
                telemetry.update();

                previousA = a;
                previousX = x;
                previousY = y;
                idle();
            }
        } finally {
            leftTurretServo.setPower(0.0);
            rightTurretServo.setPower(0.0);
            for (DcMotor motor : allMotors) {
                motor.setPower(0.0);
            }
            for (ServoImplEx indicator : indicators) {
                indicator.setPosition(RGB_OFF);
            }
        }
    }

    private double calculateAndReportTurretTarget(Pose2D pose) {
        double robotX = pose.getX(DistanceUnit.MM) + fieldOriginXMM;
        double robotY = pose.getY(DistanceUnit.MM) + fieldOriginYMM;
        // Use the odometry heading in radians for trig; the drive offset is separate.
        double currentHeading = pose.getHeading(AngleUnit.RADIANS);
        TurretTarget target = new TurretTarget(robotX, robotY, currentHeading);

        telemetry.addData("Field robot X / Y (mm)", "%.1f / %.1f", robotX, robotY);
        telemetry.addData("Selected goal", target.useGoalTwo ? 2 : 1);
        telemetry.addData("Goal X / Y (mm)", "%.1f / %.1f", target.goalX, GOAL_Y_MM);
        telemetry.addData("DiffX / DiffY (mm)", "%.1f / %.1f", target.diffX, target.diffY);
        telemetry.addData("TurretGoalX / TurretGoalY (mm)", "%.1f / %.1f",
                target.turretGoalX, target.turretGoalY);
        telemetry.addData("IdealTheta (Degrees)", "%.2f", target.idealTheta);
        return target.idealTheta;
    }

    /** Field distances in mm; heading in radians; target in signed degrees. */
    static final class TurretTarget {
        final boolean useGoalTwo;
        final double goalX, diffX, diffY, turretGoalX, turretGoalY, idealTheta;

        TurretTarget(double robotX, double robotY, double currentHeading) {
            // Keep goal one at the boundary; switch to goal two only above 1829 mm.
            useGoalTwo = robotX > GOAL_SWITCH_X_MM;
            goalX = useGoalTwo ? GOAL_TWO_X_MM : GOAL_ONE_X_MM;
            diffX = goalX - robotX;
            diffY = GOAL_Y_MM - robotY;
            turretGoalX = diffX + TURRET_OFFSET_X_MM * Math.cos(-currentHeading);
            // The supplied (357, -1687, 42.9 deg) example defines this Y sign:
            // -(216 + 32.6746) = -248.6746 mm.
            turretGoalY = -(diffY - TURRET_OFFSET_X_MM * Math.sin(-currentHeading));
            // Java atan2 takes Y first: the example's bearing is -13.0159 deg.
            double degrees = Math.toDegrees(Math.atan2(turretGoalY, turretGoalX) + currentHeading);
            // Normalize to [-180, 180); a coincident goal has no defined bearing.
            idealTheta = Math.hypot(turretGoalX, turretGoalY) < 1e-6
                    ? Double.NaN : ((degrees + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
        }
    }

    private void configureTurretEncoder() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            if (!hub.isParent()) {
                if (turretEncoderHub != null) {
                    throw new IllegalStateException("Multiple Expansion Hubs: select turret encoder Hub explicitly.");
                }
                turretEncoderHub = hub;
            }
        }
        if (turretEncoderHub == null) {
            throw new IllegalStateException("Turret encoder Expansion Hub missing; check RS485 connection.");
        }
    }

    private int sampleSensors() {
        // Refresh even if a previous OpMode left MANUAL bulk caching enabled.
        for (LynxModule hub : hubs) hub.clearBulkCache();
        odo.update();
        if (startingPose.shouldApply(odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY,
                pinpointStartupClock.seconds())) {
            odo.setPosition(new Pose2D(DistanceUnit.MM, START_X_MM, START_Y_MM,
                    AngleUnit.DEGREES, 0.0));
            // setPosition writes the device; getPosition remains cached until update.
            odo.update();
            Pose2D readback = odo.getPosition();
            startingPose.confirm(odo.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY,
                    readback.getX(DistanceUnit.MM), readback.getY(DistanceUnit.MM),
                    readback.getHeading(AngleUnit.DEGREES));
        }
        LynxModule.BulkData encoderData = turretEncoderHub.getBulkData();
        if (encoderData.isFake()) {
            throw new IllegalStateException("Turret encoder read failed; stopping OpMode.");
        }
        return encoderData.getMotorCurrentPosition(TURRET_ENCODER_PORT);
    }

    /** Startup-only initialization; never reapplies the pose after driving or an A reset. */
    static final class StartingPose {
        private boolean initialized;

        boolean shouldApply(boolean ready, double secondsSinceReset) {
            // SDK documents ~0.25 s calibration. Wait at least 0.5 s AND READY,
            // so a stale READY immediately after the reset command cannot qualify.
            return !initialized && ready && secondsSinceReset >= 0.5;
        }

        void confirm(boolean ready, double xMM, double yMM, double headingDegrees) {
            initialized |= ready && Math.abs(xMM - START_X_MM) <= 1.0
                    && Math.abs(yMM - START_Y_MM) <= 1.0 && Math.abs(headingDegrees) <= 0.5;
        }
    }

    private void addTurretTelemetry(TurretTracking turret) {
        telemetry.addData("Turret auto requested", turret.trackingRequested);
        telemetry.addData("Turret zero", turret.zeroEstablished
                ? "Straight-ahead mark" : "Align straight-ahead mark and press D-pad Down");
        telemetry.addData("Turret state", turret.state());
        telemetry.addData("Turret angle (Degrees)", "%.2f", turret.angleDegrees());
        telemetry.addData("Turret encoder ticks (from zero)", turret.signedTicks);
        telemetry.addData("Turret controller target (Degrees)", "%.2f", turret.controller.getTargetDegrees());
        telemetry.addData("Turret P / I / D / F", "%.5f / %.5f / %.5f / %.3f",
                TURRET_P, TURRET_I, TURRET_D, TURRET_F);
        telemetry.addData("Turret min / max speed (power)", "%.3f / %.3f",
                TURRET_MIN_SPEED, TURRET_MAX_SPEED);
        telemetry.addData("Turret tolerance (+/- degrees)", "%.2f", TURRET_TOLERANCE_DEGREES);
    }

    /** Hardware-independent input handling around the existing turret controller. */
    static final class TurretTracking {
        private static final int ENCODER_SIGN = 1;
        private static final double TICKS_PER_DEGREE = 8192.0 * (90.0 / 24.0) / 360.0;
        final TurretAimController controller = new TurretAimController(false, false, false);
        private int zeroTicks;
        private long signedTicks;
        private boolean zeroEstablished;
        private boolean trackingRequested = true;
        private boolean trackingActive;
        private boolean previousZero, previousToggle;

        TurretTracking(int rawTicks, boolean zeroHeld) {
            zeroTicks = rawTicks;
            previousZero = zeroHeld;
        }

        void initSample(int rawTicks, boolean zero) {
            if (zero && !previousZero) calibrate(rawTicks);
            signedTicks = ENCODER_SIGN * ((long) rawTicks - zeroTicks);
            previousZero = zero;
        }

        void onStart(boolean zeroHeld, boolean toggleHeld) {
            previousZero = zeroHeld;
            previousToggle = toggleHeld;
        }

        double update(int rawTicks, double now, boolean odometryReady, double idealTheta,
                      boolean zero, boolean toggle, boolean lb, boolean rb,
                      TurretAimController.Settings settings) {
            boolean zeroPressed = zero && !previousZero;
            if (zeroPressed) calibrate(rawTicks);
            signedTicks = ENCODER_SIGN * ((long) rawTicks - zeroTicks);
            if (zeroPressed) controller.cancel(angleDegrees(), signedTicks, now);
            if (toggle && !previousToggle) trackingRequested = !trackingRequested;
            previousZero = zero;
            previousToggle = toggle;
            // Preserve JackbotsTeleOp's manual directions: LB negative, RB positive.
            // Any manual input leaves auto off until D-pad Up explicitly resumes it.
            if (lb || rb) trackingRequested = false;
            if (!trackingRequested || zero) {
                trackingActive = false;
            } else if (zeroEstablished && odometryReady) {
                trackingActive = true;
            }
            // Once tracking starts, loss of READY is sent as an invalid target so
            // the existing controller latches the fault until an explicit retry.
            double power = controller.update(angleDegrees(), signedTicks, now,
                    rb && !zero, lb && !zero, false, false, false, settings,
                    trackingActive, odometryReady ? idealTheta : Double.NaN);
            if (zero) {
                controller.cancel(angleDegrees(), signedTicks, now);
                return 0.0;
            }
            return power;
        }

        private void calibrate(int rawTicks) {
            zeroTicks = rawTicks;
            zeroEstablished = true;
        }

        double angleDegrees() {
            return signedTicks / TICKS_PER_DEGREE;
        }

        String state() {
            if (trackingRequested && !zeroEstablished) return "Waiting for straight-ahead zero";
            if (trackingRequested && !trackingActive && !previousZero) return "Waiting for odometry READY";
            return controller.getState();
        }
    }

    /** Require uninterrupted detection; every clear reading starts the wait over. */
    static final class ColorConfirmation {
        private static final double REQUIRED_SECONDS = 0.5;
        private double detectedSince = Double.NaN;

        boolean update(boolean detected, double nowSeconds) {
            if (!detected) {
                detectedSince = Double.NaN;
                return false;
            }
            if (Double.isNaN(detectedSince)) detectedSince = nowSeconds;
            return nowSeconds - detectedSince >= REQUIRED_SECONDS;
        }
    }

    private static boolean detectsColor(NormalizedRGBA colors) {
        float peak = Math.max(colors.red, Math.max(colors.green, colors.blue));
        float min = Math.min(colors.red, Math.min(colors.green, colors.blue));
        return peak >= MIN_COLOR_SIGNAL && (peak - min) / peak >= MIN_COLOR_SATURATION;
    }

    private static void configurePowerMotor(DcMotor motor, DcMotor.Direction direction) {
        motor.setPower(0.0);
        motor.setDirection(direction);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static void configureFlywheel(DcMotorEx motor, double maxRPM, double gearing,
                                          DcMotor.RunMode runMode) {
        motor.setPower(0.0);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        MotorConfigurationType type = motor.getMotorType().clone();
        type.setMaxRPM(maxRPM);
        type.setTicksPerRev(FLYWHEEL_TICKS_PER_MOTOR_REV * gearing);
        type.setGearing(gearing);
        motor.setMotorType(type);
        motor.setMode(runMode);
    }

    private void configurePinpoint() {
        // Read the tuned values directly so TeleOp stays aligned with autonomous.
        PinpointConfig config = Constants.localizerConfig;
        odo.setOffsets(config.xPodOffset.get(), config.yPodOffset.get(), config.offsetUnits.get());
        if (config.ticksPerUnit.get().isPresent()) {
            odo.setEncoderResolution(config.ticksPerUnit.get().getAsDouble(),
                    config.encoderResolutionUnit.get());
        } else {
            odo.setEncoderResolution(config.podType.get());
        }
        odo.setEncoderDirections(config.xPodDirection.get(), config.yPodDirection.get());
        // Reset/calibration completes asynchronously. sampleSensors applies the
        // starting pose only after READY, then verifies it from a fresh device read.
        odo.resetPosAndIMU();
        pinpointStartupClock.reset();
        odo.update();
    }

    private void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;
        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        frontLeft.setPower(frontLeftPower / maxPower);
        frontRight.setPower(frontRightPower / maxPower);
        backLeft.setPower(backLeftPower / maxPower);
        backRight.setPower(backRightPower / maxPower);
    }
}
