package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Driver 1 (gamepad1): drive base, intake toggle (cross = start, square = stop),
 *   slow-drive toggle (triangle), reverse-intake override (circle, hold).
 * Driver 2 (gamepad2): shooter trigger (left_bumper), angle servo toggle (square),
 *   lift motor (dpad_up / dpad_down).
 *
 * Shooter flow: pressing the trigger turns everything else off and spins the
 * shooter up to TARGET_RPM under PID control. Feeding (intake/mixer/feeder)
 * only runs while the flywheel is at speed: it turns on once RPM rises within
 * RPM_TOLERANCE_ENTER of target, and pauses again if a shot drags RPM down
 * past RPM_TOLERANCE_EXIT, so every ball is only fired once the flywheel has
 * recovered. Toggling the shooter off resets this state and the PID state.
 */
@TeleOp(name = "ShooterPIDTeleOp")
public class Teleop1 extends LinearOpMode {

    // Hardware map configuration names below are kept exactly as on the robot's
    // saved config, even though the Java field names are now in plain English.
    // Control Hub  : 0-intakeMotor, 1-shooterSag, 2-ballGiverMotor, 3-sagMotor
    // Expansion Hub: 0-liftMotor, 1-shooterSol, 2-mixerMotor, 3-cepMotor
    private DcMotorEx shooterRightMotor; // config name: "shooterSag" (PID reads this encoder)
    private DcMotorEx shooterLeftMotor;  // config name: "shooterSol" (mirrors shooterRightMotor)
    private DcMotor intakeMotor;
    private DcMotor ballFeederMotor;     // config name: "ballGiverMotor"
    private DcMotor rightDriveMotor;     // config name: "sagMotor"
    private DcMotor leftDriveMotor;      // config name: "cepMotor"
    private DcMotor liftMotor;
    private DcMotor mixerMotor;
    private Servo angleServo;            // config name: "aciServo"

    // ================= TUNING (ADJUST FOR YOUR ROBOT) =================
    // Shooter PID gains
    static final double kP = 0.001400;
    static final double kI = 0.000010;
    static final double kD = 0.000045;
    static final double kF = 0.000180; // feedforward, scales with target RPM

    static final double TICKS_PER_REV = 28.0;
    static final double TARGET_RPM    = 4000.0;

    // Feeding is gated on RPM with hysteresis so a shot's RPM dip doesn't cause
    // rapid on/off chatter: start feeding once within RPM_TOLERANCE_ENTER of
    // target, stop feeding only once RPM has dropped further, past
    // RPM_TOLERANCE_EXIT. This forces the flywheel to fully recover between
    // shots instead of firing balls back-to-back into a slowing wheel.
    static final double RPM_TOLERANCE_ENTER = 100.0;
    static final double RPM_TOLERANCE_EXIT  = 300.0;

    static final double INTEGRAL_MAX = 4000.0; // clamps integral windup
    static final double RPM_FILTER_A = 0.2;    // low-pass filter coefficient (0-1, lower = smoother)

    // Other motor powers
    static final double INTAKE_POWER      = 1.0;
    static final double MIXER_POWER       = -1.0;
    static final double BALL_FEEDER_POWER = 1.0;
    static final double LIFT_UP_POWER     = 1.0;
    static final double LIFT_DOWN_POWER   = -0.6;

    // Drive powers
    static final double DRIVE_POWER      = 1.0;
    static final double SLOW_DRIVE_POWER = 0.5;
    static final double TURN_POWER       = 1.0; // turning is always full power, even in slow mode

    // Angle servo positions. Servo.setPosition() takes 0.0-1.0, not degrees,
    // so these are expressed as a fraction of the servo's 180-degree range.
    static final double ANGLE_SERVO_RAISED_POSITION  = 150.0 / 180.0;
    static final double ANGLE_SERVO_LOWERED_POSITION = 100.0 / 180.0;
    // =====================================================================

    // PID state
    private double integralSum = 0;
    private double lastError = 0;
    private final ElapsedTime pidTimer = new ElapsedTime();

    // Filtered RPM state
    private double filteredRPM = 0;
    private boolean filterInitialized = false;

    // Mode state
    private boolean shooterMode = false;
    private boolean intakeMode = false;
    private boolean feeding = false; // true while flywheel is at speed and balls are being fed

    // Button edge-detection state
    private boolean lastLeftBumperG2 = false;
    private boolean lastCross = false;
    private boolean lastSquareG1 = false;
    private boolean lastTriangle = false;

    private boolean slowDriveMode = false;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Ready. Press PLAY to start.");
        telemetry.update();

        waitForStart();
        pidTimer.reset();

        while (opModeIsActive()) {
            updateSlowDriveToggle();
            driveRobot();
            updateLiftMotor();
            updateAngleServo();
            updateModeButtons();
            runActiveMode();
            applyIntakeReverseOverride();
            updateTelemetry();
        }
    }

    private void initHardware() {
        shooterRightMotor = hardwareMap.get(DcMotorEx.class, "shooterSag");
        shooterLeftMotor  = hardwareMap.get(DcMotorEx.class, "shooterSol");
        intakeMotor       = hardwareMap.get(DcMotor.class, "intakeMotor");
        mixerMotor        = hardwareMap.get(DcMotor.class, "mixerMotor");
        ballFeederMotor   = hardwareMap.get(DcMotor.class, "ballGiverMotor");
        rightDriveMotor   = hardwareMap.get(DcMotor.class, "sagMotor");
        leftDriveMotor    = hardwareMap.get(DcMotor.class, "cepMotor");
        liftMotor         = hardwareMap.get(DcMotor.class, "liftMotor");
        angleServo        = hardwareMap.get(Servo.class, "aciServo");

        shooterRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Drive motors are usually mirrored; flip these if "forward" drives backward.
        rightDriveMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // power is driven by our own PID
        shooterLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        angleServo.setPosition(ANGLE_SERVO_RAISED_POSITION);
    }

    private void updateSlowDriveToggle() {
        boolean triangle = gamepad1.triangle;
        if (triangle && !lastTriangle) {
            slowDriveMode = !slowDriveMode;
        }
        lastTriangle = triangle;
    }

    private void driveRobot() {
        double activeDrivePower = slowDriveMode ? SLOW_DRIVE_POWER : DRIVE_POWER;

        double driveLeft = 0;
        double driveRight = 0;

        if (gamepad1.right_bumper) {
            driveLeft = activeDrivePower;
            driveRight = activeDrivePower;
        } else if (gamepad1.left_bumper) {
            driveLeft = -activeDrivePower;
            driveRight = -activeDrivePower;
        }

        if (gamepad1.dpad_left) {
            driveLeft -= TURN_POWER;
            driveRight += TURN_POWER;
        } else if (gamepad1.dpad_right) {
            driveLeft += TURN_POWER;
            driveRight -= TURN_POWER;
        }

        leftDriveMotor.setPower(clamp(driveLeft));
        rightDriveMotor.setPower(clamp(driveRight));
    }

    private void updateLiftMotor() {
        if (gamepad2.dpad_up) {
            liftMotor.setPower(LIFT_UP_POWER);
        } else if (gamepad2.dpad_down) {
            liftMotor.setPower(LIFT_DOWN_POWER);
        } else {
            liftMotor.setPower(0.0);
        }
    }

    private void updateAngleServo() {
        angleServo.setPosition(gamepad2.square ? ANGLE_SERVO_LOWERED_POSITION : ANGLE_SERVO_RAISED_POSITION);
    }

    private void updateModeButtons() {
        boolean cross = gamepad1.cross;
        boolean squareG1 = gamepad1.square;
        boolean lbG2 = gamepad2.left_bumper;

        if (cross && !lastCross) {
            intakeMode = true;
            shooterMode = false; // the two modes are mutually exclusive
        }
        if (squareG1 && !lastSquareG1) {
            intakeMode = false;
        }
        lastCross = cross;
        lastSquareG1 = squareG1;

        if (lbG2 && !lastLeftBumperG2) {
            shooterMode = !shooterMode;
            if (shooterMode) {
                intakeMode = false;
            } else {
                resetShooterState();
            }
        }
        lastLeftBumperG2 = lbG2;
    }

    private void runActiveMode() {
        if (intakeMode) {
            runIntakeMode();
        } else if (shooterMode) {
            runShooterMode();
        } else {
            runIdleMode();
        }
    }

    private void runIntakeMode() {
        shooterRightMotor.setPower(0);
        shooterLeftMotor.setPower(0);
        intakeMotor.setPower(INTAKE_POWER);
        mixerMotor.setPower(0);
        ballFeederMotor.setPower(0);
    }

    private void runShooterMode() {
        double currentRPM = getFilteredShooterRPM();
        double power = calculatePID(TARGET_RPM, currentRPM);
        shooterRightMotor.setPower(power);
        shooterLeftMotor.setPower(power);

        if (feeding) {
            if (currentRPM < TARGET_RPM - RPM_TOLERANCE_EXIT) {
                feeding = false; // a shot just dropped RPM too far; wait for recovery
            }
        } else {
            if (currentRPM >= TARGET_RPM - RPM_TOLERANCE_ENTER) {
                feeding = true; // flywheel is back up to speed; resume feeding
            }
        }

        if (feeding) {
            intakeMotor.setPower(INTAKE_POWER);
            mixerMotor.setPower(MIXER_POWER);
            ballFeederMotor.setPower(BALL_FEEDER_POWER);
        } else {
            intakeMotor.setPower(0);
            mixerMotor.setPower(0);
            ballFeederMotor.setPower(0);
        }
    }

    private void runIdleMode() {
        shooterRightMotor.setPower(0);
        shooterLeftMotor.setPower(0);
        intakeMotor.setPower(0);
        mixerMotor.setPower(0);
        ballFeederMotor.setPower(0);
        resetPidState();
    }

    private void resetShooterState() {
        feeding = false;
        resetPidState();
    }

    private void resetPidState() {
        integralSum = 0;
        lastError = 0;
        filterInitialized = false;
    }

    private void applyIntakeReverseOverride() {
        // Held down, this overrides whatever the active mode set, to clear a jam
        // or spit a ball back out.
        if (gamepad1.circle) {
            intakeMotor.setPower(-INTAKE_POWER);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Mode", shooterMode ? "SHOOTER" : (intakeMode ? "INTAKE" : "IDLE"));
        telemetry.addData("Feeding (RPM gated)", feeding);
        telemetry.addData("Slow Drive Mode", slowDriveMode ? "ON (0.5)" : "OFF (1.0)");
        telemetry.addData("Shooter RPM (filtered)", "%.1f", filteredRPM);
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.update();
    }

    private static double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    /** Raw RPM from encoder velocity (ticks/sec). */
    private double getShooterRPM() {
        double ticksPerSecond = shooterRightMotor.getVelocity();
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    /** Low-pass filtered RPM, so encoder noise doesn't blow up the PID's D term. */
    private double getFilteredShooterRPM() {
        double raw = getShooterRPM();
        if (!filterInitialized) {
            filteredRPM = raw;
            filterInitialized = true;
        } else {
            filteredRPM = RPM_FILTER_A * raw + (1 - RPM_FILTER_A) * filteredRPM;
        }
        return filteredRPM;
    }

    /** PID + feedforward. Output is clamped to [-1.0, 1.0]. */
    private double calculatePID(double target, double current) {
        double error = target - current;

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0) dt = 0.001;

        integralSum += error * dt;
        integralSum = Math.max(-INTEGRAL_MAX, Math.min(INTEGRAL_MAX, integralSum));

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = (kP * error) + (kI * integralSum) + (kD * derivative) + (kF * target);
        return clamp(output);
    }
}
