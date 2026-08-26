package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Shooter akışı: tetiğe basınca shooter PID+feedforward ile TARGET_VELOCITY'e
 * hızlanır. Hız ilk kez FEED_ENTER_TOLERANCE içine girdiğinde feeding
 * "kilitlenir" ve şutu tetiklendiği andan itibaren shooter modu kapatılana
 * kadar intake/mixer/ballGiver hiç durmadan (asla 0 güce inmeden) çalışır --
 * 35 topu art arda ateşlemek için ara vermeden beslemeyi garantiler.
 *
 * ballGiverMotor error'a bakılmaksızın hep BALLGIVER_POWER (1.0) ile
 * çalışır -- toparlanma yükü tamamen shooter PID + feedforward'a ait.
 * Ayrıca sürekli tam güç çekiminde batarya voltajı düştükçe motor gücü de
 * düşer; bunu telafi etmek için voltaj kompanzasyonu eklendi.
 */
@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {

    private DcMotorEx shooterSag;
    private DcMotorEx shooterSol;
    private DcMotor   intakeMotor;
    private DcMotor   ballGiverMotor;
    private DcMotor   mixerMotor;
    private DcMotor   sagMotor;
    private DcMotor   liftMotor;
    private DcMotor   cepMotor;
    private Servo     aciServo;
    private VoltageSensor batteryVoltageSensor;

    // ================= PID + Feedforward (ROBOTUNUZDA AYARLAYIN) =================
    // KF: sabit hızı tutmak için gereken temel gücü sağlar (kP/kI artık sadece
    // hatayı düzeltir, hızı sıfırdan üretmek zorunda kalmaz). Başlangıç tahmini:
    // kF ~= 1 / (motorun serbest -yüksüz- hızı, ticks/sec). Shooteri boşta tam
    // güçte (power=1.0) çalıştırıp telemetriden okuduğunuz "Velocity" değerini
    // FREE_SPEED_ESTIMATE yerine yazıp kF = 1.0 / o_deger yapın. Bu değer
    // yanlışsa PID her seferinde hızı baştan error ile üretmeye çalışır ve
    // arka arkaya atışlarda RPM kümülatif düşer -- en kritik ayar budur.
    static final double FREE_SPEED_ESTIMATE = 2800.0; // ticks/sec, gerçek değerle güncelleyin
    static final double KF = 1.0 / FREE_SPEED_ESTIMATE;

    static final double KP = 0.0035;
    static final double KI = 0.00006;
    static final double KD = 0.00022;

    static final double TARGET_VELOCITY = 1900;   // ticks/sec
    static final double INTEGRAL_LIMIT  = 4000;
    // D terimi encoder gürültüsünden etkilenmesin diye hız düşük geçirgen
    // filtreden geçirilir (0-1 arası, düşük değer = daha yumuşak/gecikmeli).
    static final double VELOCITY_FILTER_A = 0.2;

    // Batarya voltajı 12V'un altına düştükçe komut edilen güç, aynı torku
    // üretmek için orantılı olarak artırılır. Sürekli tam güçte 35 top atan
    // bir koşuda voltaj düşüşü, sonraki topların zayıf çıkmasının gerçek bir
    // sebebidir.
    static final double NOMINAL_VOLTAGE = 12.0;

    // Feeding, hız ilk kez hedefe bu kadar yaklaşınca kilitlenir ve bir daha
    // error'a bakılmaksızın shooter modu kapanana kadar açık kalır.
    static final double FEED_ENTER_TOLERANCE = 250;

    static final double DRIVE_POWER       = 1.0;
    static final double TURN_POWER        = 1.0;
    static final double INTAKE_POWER      = 1.0;
    static final double MIXER_POWER       = 1.0;
    static final double BALLGIVER_POWER   = 1.0;
    static final double LIFT_POWER        = 1.0;

    static final double SERVO_POS_LOW     = 0.99;
    static final double SERVO_POS_HIGH    = 0.70;

    private double integral   = 0;
    private double lastError  = 0;

    private double filteredVelocity = 0;
    private boolean velocityFilterInitialized = false;

    private final ElapsedTime pidTimer  = new ElapsedTime();
    private final ElapsedTime loopTimer = new ElapsedTime();

    private boolean shooterOn = false;
    private boolean intakeOn  = false;
    private boolean feeding   = false; // hedefe ilk ulaşınca kilitlenir, error'dan etkilenmez
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
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        sagMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        cepMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        mixerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        sagMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cepMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterSag.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterSol.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterSag.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterSol.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterSag.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // İkisi de RUN_WITHOUT_ENCODER: gücü tamamen aşağıdaki manuel PID
        // hesaplıyor, hub'ın kendi dahili hız PID'i devreye girip bizim
        // döngümüzle çakışmasın (iç içe iki PID salınıma/gecikmeye yol açar).
        shooterSag.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        if      (gamepad2.dpad_down)   liftMotor.setPower(-LIFT_POWER);
        else if (gamepad2.dpad_up) liftMotor.setPower( LIFT_POWER);
        else                         liftMotor.setPower(0.07);
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
        double rawVelocity = shooterSag.getVelocity();
        filteredVelocity = velocityFilterInitialized
                ? VELOCITY_FILTER_A * rawVelocity + (1 - VELOCITY_FILTER_A) * filteredVelocity
                : rawVelocity;
        velocityFilterInitialized = true;

        double power = calculatePID(TARGET_VELOCITY, filteredVelocity);
        shooterSag.setPower(power);
        shooterSol.setPower(power);

        // Hedefe ilk ulaşınca kilitlenir; bundan sonra shooter modu
        // kapanana kadar bir daha kapanmaz.
        if (!feeding && Math.abs(lastError) < FEED_ENTER_TOLERANCE) {
            feeding = true;
        }

        // Hedefe ulaşınca üçü de tam güçte, kesintisiz çalışır. Toparlanmayı
        // hızlı tutmak artık tamamen calculatePID'deki feedforward + PID'in işi.
        intakeMotor.setPower(feeding ? INTAKE_POWER : 0);
        mixerMotor.setPower(feeding ? MIXER_POWER : 0);
        ballGiverMotor.setPower(feeding ? BALLGIVER_POWER : 0);

        telemetry.addData("Velocity (raw)",      "%.0f", rawVelocity);
        telemetry.addData("Velocity (filtered)", "%.0f", filteredVelocity);
        telemetry.addData("Feeding (latched)",   feeding);
    }

    private void runIdleMode() {
        shooterSag.setPower(0);
        shooterSol.setPower(0);
        intakeMotor.setPower(0);
        ballGiverMotor.setPower(0);
        mixerMotor.setPower(0);
        resetShooterState();
    }

    // Manual PID + feedforward — target/current ticks/sec, çıkış [0, 1] aralığında
    private double calculatePID(double target, double current) {
        double error = target - current;

        double dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt <= 0) dt = 0.001;

        double pTerm = KP * error;
        double fTerm = KF * target;
        double dTerm = KD * (error - lastError) / dt;
        lastError = error;

        // Doğru anti-windup: KP+KF her gerçek hatada gücü zaten 1.0'a
        // saturasyona ulaştırıyor -- bu durumda integral eskisi gibi
        // biriktirilmeye devam ederse (eski kod |error|<400 iken hep
        // birikiyordu), flywheel toparlanınca elde fazladan bir "borç"
        // güç birikmiş olur; bu, RPM hedefi aşıp geri sönümlenmesine
        // (salınım/hunting) yol açar ve bu salınım sonraki topların
        // toparlanma süresini yer. Şimdi integral SADECE çıkış zaten
        // sınırda değilken, ya da hatanın çıkışı sınırdan geri çekeceği
        // yöndeyken birikir.
        double candidateIntegral = integral + error * dt;
        double candidateOutput = pTerm + fTerm + dTerm + KI * candidateIntegral;
        boolean pinnedHigh = candidateOutput > 1.0 && error > 0;
        boolean pinnedLow  = candidateOutput < 0.0 && error < 0;
        if (!pinnedHigh && !pinnedLow) {
            integral = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, candidateIntegral));
        }

        double iTerm = KI * integral;
        double rawPower = pTerm + iTerm + dTerm + fTerm;

        // Batarya voltajı düştükçe aynı güç fraksiyonu daha az tork üretir;
        // 12V'a göre ölçekleyerek uzun bir atış serisinde tutarlılığı korur.
        double voltage = batteryVoltageSensor.getVoltage();
        double voltageCompensation = voltage > 6.0 ? NOMINAL_VOLTAGE / voltage : 1.0;

        return Math.max(0.0, Math.min(1.0, rawPower * voltageCompensation));
    }

    private void resetShooterState() {
        feeding   = false;
        integral  = 0;
        lastError = 0;
        velocityFilterInitialized = false;
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
        telemetry.addData("Battery V", "%.2f", batteryVoltageSensor.getVoltage());
        telemetry.addData("Loop Time", "%.1f ms", loopMs);
        telemetry.update();
    }
}
