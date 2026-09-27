package org.firstinspires.ftc.teamcode;

// ═══════════════════════════════════════════════════════════════════════════════
//  TeleOpBasic.java  —  Plain hardware TeleOp. No Pedro Pathing, no odometry,
//                       no follower, no NextFTC. Just motors and one servo.
//
//  HARDWARE
//    4x drive motors   — fl, fr, bl, br   (mecanum)
//    2x shooter motors — ls, rs           (rs REVERSED so both spin the same way)
//    2x intake motors  — li, ri           (ri REVERSED so both pull inward)
//    1x linkage servo  — linkage          (toggles between UP and DOWN)
//
//  CONTROLS
//    Gamepad 1
//      left stick        — drive / strafe
//      right stick X     — rotate
//
//    Gamepad 2
//      dpad up           — toggle shooter on / off
//      dpad right        — shooter HIGH power
//      dpad left         — shooter LOW power
//      A                 — intake in  (hold)
//      B                 — intake out (hold)
//      X                 — toggle linkage up / down
//
//  MOTOR PAIR DIRECTIONS
//    Shooter and intake are each a pair of motors facing each other across the
//    robot. Mounted that way, giving both the same power makes them spin in
//    OPPOSITE directions in robot space and fight each other. So one of each
//    pair is reversed in init(), and from then on both take the same power
//    value — no sign juggling scattered through the code.
// ═══════════════════════════════════════════════════════════════════════════════

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpBasic", group = "TeleOp")
public class TeleOpBasic extends OpMode {

    // ── Drive ────────────────────────────────────────────────────────────────
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Stick values below this are treated as zero, so letting go of the sticks
    // gives true zero power rather than residual stick noise.
    private static final double DRIVE_DEADZONE = 0.05;

    // ── Shooter ──────────────────────────────────────────────────────────────
    private DcMotor leftShooter, rightShooter;

    private static final double SHOOTER_HIGH_POWER = 1.0;
    private static final double SHOOTER_LOW_POWER  = 0.75;

    private boolean shooterOn   = false;
    private boolean shooterHigh = false;

    // ── Intake ───────────────────────────────────────────────────────────────
    private DcMotor leftIntake, rightIntake;

    private static final double INTAKE_POWER = 1.0;

    // ── Linkage servo ────────────────────────────────────────────────────────
    private Servo linkage;

    private static final double LINKAGE_DOWN = 0.0;
    private static final double LINKAGE_UP   = 1.0;

    private boolean linkageUp = false;

    // ── Rising-edge flags ────────────────────────────────────────────────────
    // A held button should fire once, not every 20ms loop tick. These store
    // last tick's state so we can detect the frame a button first goes down.
    private boolean lastDpadUp    = false;
    private boolean lastDpadLeft  = false;
    private boolean lastDpadRight = false;
    private boolean lastX         = false;

    // ═════════════════════════════════════════════════════════════════════════
    //  init
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void init() {

        // ── Drive motors ─────────────────────────────────────────────────────
        frontLeft  = hardwareMap.get(DcMotor.class, "fl");
        frontRight = hardwareMap.get(DcMotor.class, "fr");
        backLeft   = hardwareMap.get(DcMotor.class, "bl");
        backRight  = hardwareMap.get(DcMotor.class, "br");

        // On most mecanum builds the left side is mounted mirrored and needs
        // reversing. Your previous code had all four FORWARD, so I've kept that
        // — if the robot spins instead of driving straight, reverse fl and bl.
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // ── Shooter motors ───────────────────────────────────────────────────
        leftShooter  = hardwareMap.get(DcMotor.class, "ls");
        rightShooter = hardwareMap.get(DcMotor.class, "rs");

        // One of the pair reversed — see header note.
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // FLOAT so the flywheels coast down instead of braking hard, which is
        // easier on the gearbox.
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ── Intake motors ────────────────────────────────────────────────────
        leftIntake  = hardwareMap.get(DcMotor.class, "li");
        rightIntake = hardwareMap.get(DcMotor.class, "ri");

        // One of the pair reversed — see header note.
        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        // BRAKE so game elements don't roll back out when the intake stops.
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ── Linkage servo ────────────────────────────────────────────────────
        linkage = hardwareMap.get(Servo.class, "linkage");
        linkage.setPosition(LINKAGE_DOWN);   // Park down on init
        linkageUp = false;

        telemetry.addLine("Initialized — press PLAY");
        telemetry.update();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  loop
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void loop() {

        // ── Drive (gamepad 1) ────────────────────────────────────────────────
        double stickY  = -gamepad1.left_stick_y;  // Controller up = negative Y
        double stickX  =  gamepad1.left_stick_x;  // Strafe
        double stickRx =  gamepad1.right_stick_x; // Rotate

        if (Math.abs(stickY)  < DRIVE_DEADZONE) stickY  = 0;
        if (Math.abs(stickX)  < DRIVE_DEADZONE) stickX  = 0;
        if (Math.abs(stickRx) < DRIVE_DEADZONE) stickRx = 0;

        // Standard mecanum mixing — each wheel gets a blend of the three inputs.
        double fl = stickY + stickX + stickRx;
        double fr = stickY - stickX - stickRx;
        double bl = stickY - stickX + stickRx;
        double br = stickY + stickX - stickRx;

        // If any wheel exceeds 1.0, scale all four down proportionally. This
        // keeps the direction of travel correct instead of clipping one wheel.
        double maxPow = Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br))));
        if (maxPow > 1.0) { fl /= maxPow; fr /= maxPow; bl /= maxPow; br /= maxPow; }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

        // ── Shooter mode (gamepad 2 dpad) ────────────────────────────────────
        boolean dUp    = gamepad2.dpad_up;
        boolean dLeft  = gamepad2.dpad_left;
        boolean dRight = gamepad2.dpad_right;

        if (dUp    && !lastDpadUp)    shooterOn   = !shooterOn; // Toggle on/off
        if (dRight && !lastDpadRight) shooterHigh = true;       // Select HIGH
        if (dLeft  && !lastDpadLeft)  shooterHigh = false;      // Select LOW

        lastDpadUp    = dUp;
        lastDpadRight = dRight;
        lastDpadLeft  = dLeft;

        // Both shooter motors take the SAME power — direction is already
        // handled by the REVERSE set on rs in init().
        double shooterPower;
        String shooterMode;
        if (!shooterOn) {
            shooterPower = 0;
            shooterMode  = "OFF";
        } else if (shooterHigh) {
            shooterPower = SHOOTER_HIGH_POWER;
            shooterMode  = "HIGH";
        } else {
            shooterPower = SHOOTER_LOW_POWER;
            shooterMode  = "LOW";
        }
        leftShooter.setPower(shooterPower);
        rightShooter.setPower(shooterPower);

        // ── Intake (gamepad 2 A / B, held) ───────────────────────────────────
        // Same power to both — direction handled by the REVERSE on ri.
        double intakePower;
        String intakeMode;
        if (gamepad2.a) {
            intakePower = INTAKE_POWER;
            intakeMode  = "IN";
        } else if (gamepad2.b) {
            intakePower = -INTAKE_POWER;
            intakeMode  = "OUT";
        } else {
            intakePower = 0;
            intakeMode  = "STOPPED";
        }
        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);

        // ── Linkage servo (gamepad 2 X, toggle) ──────────────────────────────
        boolean curX = gamepad2.x;
        if (curX && !lastX) {
            linkageUp = !linkageUp;
            linkage.setPosition(linkageUp ? LINKAGE_UP : LINKAGE_DOWN);
        }
        lastX = curX;

        // ── Telemetry ────────────────────────────────────────────────────────
        telemetry.addData("Shooter",  "%s (%.2f)", shooterMode, shooterPower);
        telemetry.addData("Intake",   "%s", intakeMode);
        telemetry.addData("Linkage",  linkageUp ? "UP" : "DOWN");
        telemetry.addLine("---");
        telemetry.addData("Drive fl/fr", "%.2f / %.2f", fl, fr);
        telemetry.addData("Drive bl/br", "%.2f / %.2f", bl, br);
        telemetry.update();
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  stop  —  park everything safely
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        leftShooter.setPower(0);
        rightShooter.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        linkage.setPosition(LINKAGE_DOWN);
    }
}