package org.firstinspires.ftc.teamcode.TurretCodes;
 
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Test: Shooter RPM Tuner", group = "Test")
public class ShooterRpmTuner extends LinearOpMode {
 
    // --- Configuration Constants ---
    private static final long RUN_DURATION_MS = 10000;  // 10 seconds to let heavy 600g+ wheels fully spool up
    private static final double TEST_POWER     = 0.85;  // 85% voltage to measure true loaded max speed
    private static final double NATIVE_FULL    = 32767.0; // REV Hub internal output ceiling
 
    private DcMotorEx ls;
    private DcMotorEx rs;
 
    @Override
    public void runOpMode() {
        ls = hardwareMap.get(DcMotorEx.class, "ls");
        rs = hardwareMap.get(DcMotorEx.class, "rs");
 
        // Stacking configuration: both spin the same physical direction to feed the shooter
        ls.setDirection(DcMotor.Direction.FORWARD);
        rs.setDirection(DcMotor.Direction.REVERSE);
 
        for (DcMotorEx m : new DcMotorEx[]{ls, rs}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // Let them coast safely
            m.setVelocity(0);
        }
 
        telemetry.addLine("=== Heavy Stacked Flywheel RPM Tuner Ready ===");
        telemetry.addLine("Press PLAY to spin up the motors and measure limits.");
        telemetry.update();
 
        waitForStart();
        if (isStopRequested()) return;
 
        telemetry.addLine("Spooling up... DO NOT TOUCH THE ROBOT!");
        telemetry.update();
 
        // 1. Run open-loop at 85% power to find maximum physical limits under load
        ls.setPower(TEST_POWER);
        rs.setPower(TEST_POWER);
 
        double maxTpsLeft = 0;
        double maxTpsRight = 0;
        long startTime = System.currentTimeMillis();
 
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < RUN_DURATION_MS)) {
            maxTpsLeft = Math.max(maxTpsLeft, Math.abs(ls.getVelocity()));
            maxTpsRight = Math.max(maxTpsRight, Math.abs(rs.getVelocity()));
 
            // Convert live encoder readings to real-time Wheel RPM for display
            double liveWheelRpmLeft = (Math.abs(ls.getVelocity()) * 60.0 / 28.0) * 1.5;
            double liveWheelRpmRight = (Math.abs(rs.getVelocity()) * 60.0 / 28.0) * 1.5;
 
            telemetry.addData("Left Live Wheel RPM", "%.1f", liveWheelRpmLeft);
            telemetry.addData("Right Live Wheel RPM", "%.1f", liveWheelRpmRight);
            telemetry.update();
        }
 
        // 2. Cut power safely and let the heavy mass coast down
        ls.setPower(0);
        rs.setPower(0);
 
        // 3. Synthesize the clean, mathematically rigorous Feedforward (F) gains per motor
        //    F maps a target velocity (ticks/sec) to the raw power command (-32767..32767)
        //    needed to hold it. We only drove the motor to maxTps using TEST_POWER (85%),
        //    not full power, so the power actually required per unit velocity is
        //    (TEST_POWER * NATIVE_FULL) / maxTps -- NOT NATIVE_FULL / maxTps, which would
        //    silently assume the motor was run at 100% power during the test.
        double tunedFLeft = (TEST_POWER * NATIVE_FULL) / maxTpsLeft;
        double tunedFRight = (TEST_POWER * NATIVE_FULL) / maxTpsRight;
 
        // Heavy flywheels have massive inertia, keep P low so it doesn't fight the weight
        double tunedP = 1.5;
 
        // Calculate absolute max physical Wheel RPM limit based on the test
        double maxPhysicalWheelRpmLeft = (maxTpsLeft * 60.0 / 28.0) * 1.5;
        double maxPhysicalWheelRpmRight = (maxTpsRight * 60.0 / 28.0) * 1.5;
        double absoluteMaxWheelRpm = Math.min(maxPhysicalWheelRpmLeft, maxPhysicalWheelRpmRight);
 
        // 4. Output final constants to screen
        while (opModeIsActive()) {
            telemetry.addLine("=== TUNING DATA GENERATED SUCCESSFULLY ===");
            telemetry.addLine("------------------------------------------------");
            telemetry.addData("RECOMMENDED P GAIN", "%.4f", tunedP);
            telemetry.addData("RECOMMENDED F LEFT", "%.4f", tunedFLeft);
            telemetry.addData("RECOMMENDED F RIGHT", "%.4f", tunedFRight);
            telemetry.addLine("------------------------------------------------");
            telemetry.addData("ABSOLUTE MAX WHEEL RPM", "%.0f RPM", absoluteMaxWheelRpm);
            telemetry.addLine("Choose a target safely below this max number.");
            telemetry.update();
        }
    }
}
 