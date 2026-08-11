package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FFGGCC")
public class Teleop2 extends LinearOpMode {

    private DcMotorEx shooterSag;
    private DcMotorEx shooterSol;
    private DcMotor   intakeMotor;
    private DcMotor   ballGiverMotor;
    private DcMotor   mixerMotor;
    private DcMotor   sagMotor;
    private DcMotor   liftMotor;
    private DcMotor   cepMotor;
    private Servo     aciServo;

    // PID gains
    static final double KP = 0.035;
    static final double KI = 0.0005;
    static final double KD = 0;

    static final double TARGET_VELOCITY   = 1750;   // ticks/sec
    static final double INTEGRAL_LIMIT    = 300;
    static final double BALLGIVER_ERROR_LIMIT = 500; // error bu değere ulaşınca ballGiverMotor durur

    static final double DRIVE_POWER       = 1.0;
    static final double TURN_POWER        = 1.0;
    static final double INTAKE_POWER      = 1.0;
    static final double BALLGIVER_POWER   = 1.0;
    static final double LIFT_POWER        = 1.0;

    static final double SERVO_POS_LOW     = 0.25;
    static final double SERVO_POS_HIGH    = 0.75;

    private double integral   = 0;
    private double lastError  = 0;

    private final ElapsedTime pidTimer  = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private boolean shooterOn = false;
    private boolean intakeOn  = false;
    private boolean feeding   = false;
    private boolean servoHigh = false;

    private boolean prevShooterBtn = false;
    private boolean prevIntakeOn   = false;
    private boolean prevIntakeOff  = false;
    private boolean prevServoBtn   = false;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addLine("Ready — press START.");
        telemetry.update();
        waitForStart();

        pidTimer.reset();
        loopTimer.reset();

        while (opModeIsActive()) {
            handleDrive();
            handleLift();
            handleServo();
            handleButtons();
            handleModes();
            handleIntakeReverse();
            sendTelemetry();
        }
    }

    private void initHardware() {
        shooterSag     = hardwareMap.get(DcMotorEx.class, "shooterSag");
        shooterSol     = hardwareMap.get(DcMotorEx.class, "shooterSol");
        intakeMotor    = hardwareMap.get(DcMotor.class,   "intakeMotor");
        mixerMotor    = hardwareMap.get(DcMotor.class,   "mixerMotor");
        ballGiverMotor = hardwareMap.get(DcMotor.class,   "ballGiverMotor");
        sagMotor       = hardwareMap.get(DcMotor.class,   "sagMotor");
        liftMotor      = hardwareMap.get(DcMotor.class,   "liftMotor");
        cepMotor       = hardwareMap.get(DcMotor.class,   "cepMotor");
        aciServo       = hardwareMap.get(Servo.class,     "aciServo");

        sagMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        cepMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sagMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cepMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterSag.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterSol.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterSag.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterSol.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterSag.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterSag.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterSol.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        aciServo.setPosition(SERVO_POS_LOW);
    }

    private void handleDrive() {
        double left  = 0;
        double right = 0;

        if      (gamepad1.right_bumper) { left =  DRIVE_POWER; right =  DRIVE_POWER; }
        else if (gamepad1.left_bumper)  { left = -DRIVE_POWER; right = -DRIVE_POWER; }

        if      (gamepad1.dpad_left)  { left -= TURN_POWER; right += TURN_POWER; }
        else if (gamepad1.dpad_right) { left += TURN_POWER; right -= TURN_POWER; }

        cepMotor.setPower(clamp(left));
        sagMotor.setPower(clamp(right));
    }

    private void handleLift() {
        if      (gamepad2.dpad_up)   liftMotor.setPower(-LIFT_POWER);
        else if (gamepad2.dpad_down) liftMotor.setPower( LIFT_POWER);
        else                         liftMotor.setPower(-0.05);
    }

    private void handleServo() {
        if (gamepad2.square && !prevServoBtn) {
            servoHigh = !servoHigh;
            aciServo.setPosition(servoHigh ? SERVO_POS_HIGH : SERVO_POS_LOW);
        }
        prevServoBtn = gamepad2.square;
    }

    private void handleButtons() {
        if (gamepad1.cross && !prevIntakeOn) {
            intakeOn  = true;
            shooterOn = false;
        }
        if (gamepad1.square && !prevIntakeOff) {
            intakeOn = false;
        }
        prevIntakeOn  = gamepad1.cross;
        prevIntakeOff = gamepad1.square;

        if (gamepad2.left_bumper && !prevShooterBtn) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                intakeOn = false;
                resetShooterState();
                pidTimer.reset();
            } else {
                resetShooterState();
            }
        }
        prevShooterBtn = gamepad2.left_bumper;
    }

    private void handleModes() {
        if (intakeOn) {
            runIntakeMode();
        } else if (shooterOn) {
            runShooterMode();
        } else {
            runIdleMode();
        }
    }

    private void handleIntakeReverse() {
        if (gamepad1.circle) {
            intakeMotor.setPower(-INTAKE_POWER);
        }
    }

    private void runIntakeMode() {
        shooterSag.setPower(0);
        shooterSol.setPower(0);
        intakeMotor.setPower(INTAKE_POWER);
        ballGiverMotor.setPower(0);
        mixerMotor.setPower(0);
    }

    private void runShooterMode() {
        double velocity = shooterSag.getVelocity();
        double power = calculatePID(TARGET_VELOCITY, velocity);
        shooterSag.setPower(power);
        shooterSol.setPower(power);

        if (!feeding && Math.abs(lastError) < 200) {
            feeding = true;
        }
        if (Math.abs(lastError) > 750) {
            feeding = false;
        }

        intakeMotor.setPower(feeding ? INTAKE_POWER : 0);
        mixerMotor.setPower(feeding ? BALLGIVER_POWER :0);

        // Error 500'e ulaşınca ball giver motoru dursun, error tekrar
        // 500'ün altına düşünce (feeding devam ediyorsa) yeniden çalışsın.
        boolean ballGiverActive = feeding && Math.abs(lastError) < BALLGIVER_ERROR_LIMIT;
        ballGiverMotor.setPower(ballGiverActive ? BALLGIVER_POWER : 0);

        telemetry.addData("Velocity", "%.0f", velocity);
        telemetry.addData("Feeding",  feeding);
        telemetry.addData("BallGiver", ballGiverActive);
    }

    private void runIdleMode() {
        shooterSag.setPower(0);
        shooterSol.setPower(0);
        intakeMotor.setPower(0);
        ballGiverMotor.setPower(0);
        mixerMotor.setPower(0);
        resetShooterState();
    }

    // Manual PID — target/current in ticks/sec, output clamped to [0, 1]
    private double calculatePID(double target, double current) {
        double error = target - current;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        double dTerm = 0;
        if (dt > 0 && dt < 1.0) {
            if (Math.abs(error) < 200) {
                integral = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integral + error * dt));
            }
            dTerm = KD * (error - lastError) / dt;
        }

        double pTerm = KP * error;
        double iTerm = KI * integral;

        lastError = error;
        return Math.max(0.0, Math.min(1.0, pTerm + iTerm + dTerm));
    }

    private void resetShooterState() {
        feeding   = false;
        integral  = 0;
        lastError = 0;
    }

    private double clamp(double value) {
        return Math.max(-1.0, Math.min(1.0, value));
    }

    private void sendTelemetry() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        String mode = shooterOn ? "SHOOTER" : (intakeOn ? "INTAKE" : "IDLE");
        telemetry.addData("Mode",      mode);
        telemetry.addData("Target",    "%.0f", TARGET_VELOCITY);
        telemetry.addData("Error",     "%.0f", lastError);
        telemetry.addData("Loop Time", "%.1f ms", loopMs);
        telemetry.update();
    }
}
