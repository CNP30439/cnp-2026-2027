package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Tester", group = "Test")
public class MotorTester extends OpMode {

    // ─── ADD YOUR MOTOR NAMES HERE ───────────────────────────────────────────
    // These must match your robot config exactly
    private static final String[] MOTOR_NAMES = {
            "fl",
            "fr",
            "bl",
            "br",
            // add more motors here, one per button (max 12 for full controller)
    };
    // ─────────────────────────────────────────────────────────────────────────

    private static final double TEST_POWER = 0.4;

    private DcMotor[] motors;
    private int motorCount;

    // button state tracking (edge detection)
    private boolean[] lastState;
    private boolean[] motorRunning;

    @Override
    public void init() {
        motorCount = MOTOR_NAMES.length;
        motors      = new DcMotor[motorCount];
        lastState   = new boolean[motorCount];
        motorRunning = new boolean[motorCount];

        for (int i = 0; i < motorCount; i++) {
            try {
                motors[i] = hardwareMap.get(DcMotor.class, MOTOR_NAMES[i]);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors[i].setPower(0);
            } catch (Exception e) {
                telemetry.addLine("ERROR: motor '" + MOTOR_NAMES[i] + "' not found in config!");
            }
        }

        telemetry.addLine("=== MOTOR TESTER READY ===");
        telemetry.addLine("Each button runs ONE motor at " + (TEST_POWER * 100) + "% power.");
        telemetry.addLine("Hold to run, release to stop.");
        telemetry.addLine("If motor spins BACKWARD, add REVERSE to its direction.");
        telemetry.addLine("");
        printMapping();
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- read all 12 possible buttons ---
        boolean[] pressed = new boolean[12];
        pressed[0]  = gamepad1.a;
        pressed[1]  = gamepad1.b;
        pressed[2]  = gamepad1.x;
        pressed[3]  = gamepad1.y;
        pressed[4]  = gamepad1.left_bumper;
        pressed[5]  = gamepad1.right_bumper;
        pressed[6]  = gamepad1.dpad_up;
        pressed[7]  = gamepad1.dpad_down;
        pressed[8]  = gamepad1.dpad_left;
        pressed[9]  = gamepad1.dpad_right;
        pressed[10] = gamepad1.left_stick_button;
        pressed[11] = gamepad1.right_stick_button;

        // --- drive each motor while its button is held ---
        for (int i = 0; i < motorCount; i++) {
            if (motors[i] == null) continue;

            boolean held = pressed[i];
            motorRunning[i] = held;

            if (held) {
                motors[i].setPower(TEST_POWER);
            } else {
                motors[i].setPower(0);
            }
        }

        // --- telemetry ---
        telemetry.addLine("=== MOTOR TESTER ===");
        telemetry.addLine("Power: " + (TEST_POWER * 100) + "%  |  Hold button = run motor");
        telemetry.addLine("");

        for (int i = 0; i < motorCount; i++) {
            String status;
            if (motors[i] == null) {
                status = "NOT FOUND IN CONFIG";
            } else if (motorRunning[i]) {
                status = ">>> RUNNING <<<";
            } else {
                status = "idle";
            }

            telemetry.addLine(
                    "[" + buttonName(i) + "] " + MOTOR_NAMES[i] + " -> " + status
            );
        }

        telemetry.addLine("");
        telemetry.addLine("--- BUTTON MAP ---");
        printMapping();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (motors == null) return;
        for (DcMotor m : motors) {
            if (m != null) m.setPower(0);
        }
    }

    private void printMapping() {
        String[] names = {"A", "B", "X", "Y", "LB", "RB",
                "D-Up", "D-Down", "D-Left", "D-Right", "L3", "R3"};
        for (int i = 0; i < motorCount && i < names.length; i++) {
            telemetry.addLine("  " + names[i] + " -> " + MOTOR_NAMES[i]);
        }
    }

    private String buttonName(int i) {
        String[] names = {"A", "B", "X", "Y", "LB", "RB",
                "D-Up", "D-Dn", "D-Lt", "D-Rt", "L3", "R3"};
        return (i < names.length) ? names[i] : "?";
    }
}