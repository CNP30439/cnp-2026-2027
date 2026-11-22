package org.firstinspires.ftc.teamcode.TeleOps;

// ═══════════════════════════════════════════════════════════════════════════════
// TeleMainTurret.java — TeleOp for the turret robot.
// Pedro manual drive + turret that keeps pointing at the goal (on/off switch)
// + shooter, intake, flower, blocker, kicker, prism.
//
// THIS REVISION (shoot on the move REMOVED, anti-shake):
//   * Shoot-on-the-move aim lead removed: the turret aims at the real goal from where the robot is now.
//   * Kept: kD brakes the DIFFERENCE between turret speed and target speed (does not fight tracking).
//   * SHOOT_READY_DEG back to 1.0.
//   * Breakaway kick made gentler (KICK_ERR 1 -> 2 deg, KICK_RAMP_PER_S 2.5 -> 1.0) to stop the
//     turret bouncing side to side when it stops near the target.
//   * HOLD ZONE: settled within HOLD_ENTER (0.5 deg) and goal not moving -> power 0, BRAKE holds.
//     Corrects again only past HOLD_EXIT (= SHOOT_READY, 1.0 deg). Stops shooter vibration being amplified.
//     Telemetry "Goal rate / power / I" shows (HOLD) while it is holding.
//   * Turret speed is smoothed (SPEED_FILTER) so vibration does not feed kD.
//   * WHILE SHOOTING: hold lets the turret be knocked up to SHOOT_HOLD_EXIT_RAD (2.0 deg) before
//     correcting, and the breakaway kick is off.
//
// Homing on PLAY, in order:
//       1. SERVO_UP    : home servo goes UP, wait HOME_SERVO_MS.
//       2. SEEK_SWITCH : turret turns LEFT at HOME_POWER until the touch sensor is pressed.
//                        On press: turret stops and THAT SPOT BECOMES TURRET 0 (HOME_SWITCH_RAD).
//       3. SERVO_DOWN  : home servo goes DOWN, turret held still (no power) for HOME_SERVO_MS.
//       4. DONE        : tracking the goal starts.
//   * Sensor never hit within HOME_TIMEOUT_S -> INIT zero is kept, goes to SERVO_DOWN.
//   * L2 kill switch during homing -> homing aborts, servo DOWN, turret unpowered.
//   * The kicker never fires while homing.
//
// BASE TURRET TUNING ORDER (one change per test):
//   0. Checks: battery 12.5-13 V, 180 deg by hand reads 180.0 in INIT, homing shows DONE.
//   2. Breakaway: aim ON, robot still, release turret 1, 2, 3... deg off. Smallest angle that
//      returns on its own (worse direction) -> breakaway = kP * angle / 57.3.
//   3. MIN_POWER = breakaway + 0.01, kS = about half breakaway.
//   4. kP / kD on big swings.   5. kV while spinning.
//   6. SPIN_FILTER.   7. kI only if it always settles short.   8. SHOOT_READY_DEG.
//   9. Aim bias: PIVOT_X/Y_IN, then the AIM_ZONES tables.
//
//   * GAMEPAD 1 LEFT TRIGGER (L2) = TURRET KILL SWITCH. Press to cut ALL turret power, press again
//     to bring it back. (Motor is in BRAKE, so it resists turning by hand while killed.)
//   * GAMEPAD 1 RIGHT TRIGGER (R2, hold) = AUTO-DRIVE TO THE SHOOTING ZONE, only while the turret is
//     "off" (L2 kill switch ON, or right bumper aim OFF). Logic copied from AutoShootDodgeRange:
//       - drives to the nearest legal point in the shooting zone (SHOOT_DISTANCE from the goal),
//         turning so the BACK of the robot (turret at zero) faces the goal
//       - routes around the two structures (KEEP_OUT_ZONES) with one smooth path
//       - then HOLDS the pose while R2 stays held, re-aims, and once the aim is locked it
//         AUTO-SHOOTS exactly like gamepad 2 R2 (shooter must already be on: gamepad 2 dpad up)
//       - the turret is LOCKED at zero (powered hold) the whole time R2 is held
//       - release R2 (or turn the turret back on) = driver control again
//     NOT COPIED: the Limelight pose reset (this robot has no Limelight in its config).
//   * SHOOT GATE: the kicker only fires once the turret is within SHOOT_READY_DEG of its target
//     (checked while tracking or auto-driving). Set SHOOT_READY_DEG to 180 to turn the gate off.
//
// HARDWARE (names must match the Robot Configuration)
//   fl, fr, bl, br (Pedro Constants) | ls shooter | li, ri intake (ri REVERSED)
//   linkageleft, linkageright (flower) | blocker | kicker | prism | turret (ELC encoder on its encoder port)
//   homeservo | touch
//
// INIT (gamepad 1): dpad up = RED, dpad down = BLUE.
//
// GAMEPAD 1 (driver)
//   left stick = drive/strafe, right stick X = rotate
//   B (press)            = switch preset 0 <-> 1 (changes goal + reset pose)
//   X (press)            = reset pose to the preset's start pose (ignored while auto-driving)
//   right bumper (press) = TURRET AIM ON / OFF (OFF parks turret at zero)
//   L2 (press)           = TURRET KILL SWITCH (cuts all turret power, press again to restore)
//   R2 (hold)            = auto-drive to shooting zone + hold + auto-shoot (turret must be off)
//
// GAMEPAD 2 (operator)
//   dpad up      = shooter on / off
//   dpad left / right = shooter velocity -/+ 50 ticks/sec (starts at SHOOTER_START_VELOCITY)
//   A / B        = intake in / out (hold)
//   Y            = toggle flower up / down
//   R2 (hold)    = SHOOT: intake IN, blocker OPEN, kicker fires after KICK_DELAY_MS
//                  AND the turret is on target. Release: kicker back to BASE, blocker CLOSED.
//   NOTE: R2 does not spin the shooter up. Turn it on with dpad up first.
//
// GAME RAMP: from PLAY the commanded shooter velocity creeps up by GAME_RAMP_EXTRA over GAME_RAMP_SECS.
//
// SIDE AIM CORRECTION (turret tracking only, preset 0/1):
//   One table per alliance + preset: RED0/RED1/BLUE0/BLUE1_AIM_ZONES (in the "Turret tuning" section).
//   Each row = { fromIn, toIn, turnDeg }. Positive inches = robot is RIGHT of the goal,
//   negative = LEFT. First matching row wins.
// ═══════════════════════════════════════════════════════════════════════════════

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

@TeleOp(name = "TeleMain😎", group = "TeleOp")
public class TeleMainTurret extends OpMode {
    // ── Field data ───────────────────────────────────────────────────────────
    private enum Alliance { RED, BLUE }

    private static final PoseFactory p = PoseFactory.degrees();

    public static final double[][] RED_GOALS  = { {57, 53}, {57, 90} };
    public static final double[][] BLUE_GOALS = { {84.5, 53}, {84.5, 90} };

    public static final Pose[] RED_RESET_POSES = {
            p.of(7.25, 8, -90), p.of(7.25, 136, 90)
    };
    public static final Pose[] BLUE_RESET_POSES = {
            p.of(135.25, 8, -90), p.of(135.25, 136, 90)
    };

    // ── Drive ────────────────────────────────────────────────────────────────
    private Follower follower;
    private static final double DRIVE_DEADZONE = 0.05;

    // ── Auto-drive to the shooting zone (gamepad 1 R2, turret off) ──────────
    private enum AlignState { MANUAL, ALIGNING, ARRIVED }

    // The robot may shoot from anywhere inside these bounds (R2 drives to the nearest point inside them).
    private static final double PRESET0_MAX_Y = 25;   // preset 0: y must be under this
    private static final double PRESET1_MIN_Y = 116;  // preset 1: y must be over this
    private static final double RED_X_MIN = 25, RED_X_MAX = 85;
    private static final double BLUE_X_MIN = 50, BLUE_X_MAX = 119;
    private static final double ZONE_MARGIN = 1.0;    // aim this far INSIDE the bounds
    private static final double WALL_MARGIN = 8.0;    // robot center stays this far from field edge

    // Distance from the goal the robot tries to stop at (the distance the shot is tuned for).
    // Set to 0 to go back to "nearest point in zone".
    private static final double SHOOT_DISTANCE = 41.0;
    // Gap from robot center to where balls leave (shooter is on the back). 0 = measure from the center.
    private static final double SHOOTER_OFFSET_IN = 0.0;

    // Turn-in-place arrival tolerances (when already inside the zone).
    private static final double HEADING_TOL_RAD = Math.toRadians(3);
    private static final double POSITION_TOL_IN = 1.5;

    // Aim check before auto-shoot (measured from where the robot IS, not where it was told to stop).
    private static final double AIM_OK_RAD = Math.toRadians(1.0);   // start shooting when aim error is under this
    private static final double AIM_LOST_RAD = Math.toRadians(1.5); // pause shooting if it gets worse than this
    private static final double AIM_LATE_RAD = Math.toRadians(1.5); // after AIM_LATE_MS, accept this instead
    private static final double AIM_LATE_MS = 1500;
    private static final double REAIM_RAD = Math.toRadians(0.3);    // re-aim the hold heading when it is off by this
    private static final double REAIM_EVERY_MS = 50;

    private AlignState alignState = AlignState.MANUAL;
    private boolean alignWasHeld = false;
    private boolean alignStarted = false; // follower became busy after scheduling (stops a false ARRIVED)
    private boolean turnOnly = false;
    private boolean aimLocked = false;
    private final ElapsedTime arrivedTimer = new ElapsedTime();
    private final ElapsedTime reaimTimer = new ElapsedTime();
    private Pose alignTarget = p.of(0, 0, 0);

    // ── Structure dodging ────────────────────────────────────────────────────
    // Two structures to avoid, each { minX, minY, maxX, maxY } in field inches.
    // A rectangle left as {0,0,0,0} is ignored.
    public static final double[][] KEEP_OUT_ZONES = {
            {45, 50, 49, 91}, // STRUCTURE 1
            {91, 50, 96, 91}  // STRUCTURE 2
    };
    // The robot pose is its CENTER, so it must stay at least this far from a structure's edge.
    private static final double ROBOT_MARGIN = 7.0;
    // Blocked straight line -> one continuous rounded path (lines + curves) around the structures.
    private static final double CORNER_RADIUS = 10.0;
    private static final int CURVE_SAMPLES = 20; // per segment
    private static final double PAD_STEP = 1.5;
    private static final int PAD_TRIES = 10;
    private static final double FIELD_MIN = 0;
    private static final double FIELD_MAX = 144;
    private int routeCorners = 0; // detour guide points in the current route (0 = straight)

    // ── Turret tuning ────────────────────────────────────────────────────────
    private static final String TURRET_MOTOR = "turret";

    // ELC encoder: counts per ENCODER rev, and encoder revs per turret rev (25T encoder gear on 82T turret gear).
    // CHECK: in INIT turn the turret exactly 180 deg by hand -> "Turret angle" must read 180.0.
    // If it reads e.g. 175, multiply MOTOR_REV_PER_TURRET_REV by 175.0 / 180.0.
    private static final double TICKS_PER_MOTOR_REV      = 4000.0;
    private static final double MOTOR_REV_PER_TURRET_REV = 82.0 / 25.0;
    // +1 if turning the turret counter-clockwise (from above) makes raw ticks go UP, else -1.
    private static final double ENCODER_SIGN = 1.0;
    // +1 if positive motor power turns the turret toward + angle (CCW). Flip if it runs away when aim is ON.
    private static final double POWER_SIGN = 1.0;
    private static final double TICKS_PER_RAD =
            TICKS_PER_MOTOR_REV * MOTOR_REV_PER_TURRET_REV / (2 * Math.PI);

    private static final boolean TURRET_REVERSED = false;
    // Travel limits from the start (shooter facing back = 0). Total travel is MORE than 360 deg
    // (-95..275 = 370), so near the seam the goal can be reached two ways. The turret stays on the
    // way it is on and only swings back when it runs out of room. Widen using the INIT "Lowest / highest seen".
    private static final double TURRET_MIN_RAD = Math.toRadians(-95);
    private static final double TURRET_MAX_RAD = Math.toRadians(275);
    // Turret angle at INIT in the turret's own frame (0 = it starts facing back).
    // Only used until homing finds the touch sensor (or if homing fails).
    private static final double TURRET_START_RAD = Math.toRadians(0.0);
    // Direction the shooter points relative to robot forward when the turret is at zero. PI = backward.
    private static final double TURRET_ZERO_OFFSET_RAD = Math.PI;

    // Turret pivot relative to robot center, inches. x = forward, y = left.
    // Measured: robot 17 in long, pivot 10 in from front -> 1.5 in behind center.
    //           robot 14.75 in wide, pivot 7.5 in from left -> 0.125 in right of center.
    private static final double PIVOT_X_IN = -1.5;
    private static final double PIVOT_Y_IN = -0.125;

    // Swings side to side before settling = too little damping: lower kP / raise kD.
    private static final double kP = 3.0;    // power per rad of error
    // Power per rad/s of (target speed - turret speed). Brakes the swing WITHOUT fighting tracking.
    private static final double kD = 0.20;
    // Feedforward: power per rad/s of target speed. Pure feedforward now (kD no longer cancels it).
    // TUNE (step A): spin in place, aim ON. Turret trails -> raise. Leads / overshoots -> lower.
    private static final double kV = 0.17;
    // Static friction power. About half of breakaway power.
    private static final double kS = 0.08;
    // kS reaches full strength at this error and scales down linearly below it (no on/off chatter).
    private static final double KS_RAMP_RAD = Math.toRadians(2.0);
    // Integral: finishes the last degree against friction. 2-3 only if needed.
    private static final double kI = 0.0;                          // power per rad*s of error
    private static final double I_ZONE_RAD = Math.toRadians(4.0);  // integrate only inside this error
    private static final double I_HOLD_RAD = Math.toRadians(0.3);  // inside this: stop adding (no hunting)
    private static final double I_MAX_POWER = 0.10;                // most power the integral may add
    // Integral only builds while the turret is almost stopped, so it can't add overshoot on the way in.
    private static final double I_MAX_SPEED_RAD = Math.toRadians(15); // rad/s

    // MINIMUM POWER: turret off target by more than MIN_POWER_ERR_RAD and (almost) stopped
    // -> it gets at least MIN_POWER toward the target.
    private static final double MIN_POWER = 0.20;
    // BREAKAWAY KICK: from standstill the turret needs ~0.7 to start (Power Test), but only ~0.2 once moving.
    // Stuck off target by more than KICK_ERR_RAD -> power ramps up from MIN_POWER until it moves, then drops.
    private static final double KICK_POWER = 0.70;                         // ramp never goes above this
    // Anti-shake: was 2.5 / 1.0 deg. Still shakes -> KICK_POWER = MIN_POWER turns the kick off for a test.
    private static final double KICK_RAMP_PER_S = 1.0;                     // power per second
    private static final double KICK_ERR_RAD = Math.toRadians(2.0);       // no kick inside this error
    private static final double KICK_MOVING_RAD = Math.toRadians(5);      // rad/s: faster than this = moving
    // BACKLASH between turret ring and encoder gear. 0 = off.
    private static final double BACKLASH_RAD = Math.toRadians(0.0);
    private static final double BACKLASH_MOVE_RAD = Math.toRadians(3); // rad/s: faster = really moving
    private static final double MIN_POWER_ERR_RAD = Math.toRadians(0.4);   // no floor inside this error (no buzz)
    private static final double MIN_POWER_SPEED_RAD = Math.toRadians(20);  // rad/s: floor only when nearly stopped

    private static final double MAX_POWER = 0.96;
    private static final double SWING_POWER = 0.96;  // cap while error > SWING_ERR_DEG
    private static final double SWING_ERR_DEG = 25;
    private static final double TOLERANCE_RAD = Math.toRadians(0.22); // only used for the ON TARGET label
    private static final double SPIN_FILTER = 0.25;  // 0..1, higher = less smoothing (faster feedforward)
    // Largest believable goal-bearing rate. Stops a turret kick when the pose or goal jumps (X / B press).
    private static final double MAX_GOAL_RATE_RAD = Math.toRadians(400);   // rad/s
    // Kicker only fires while the turret is this close to its target. 180 = gate off.
    private static final double SHOOT_READY_DEG = 1.0;
    private static final double SHOOT_READY_RAD = Math.toRadians(SHOOT_READY_DEG);

    // ── Anti-shake (shooter vibration) ───────────────────────────────────────
    // HOLD ZONE: turret within HOLD_ENTER of its target, slow, and the goal not moving -> power 0 (BRAKE holds).
    // Starts correcting again only past HOLD_EXIT. HOLD_EXIT = SHOOT_READY so a held turret can always shoot.
    // Still shakes -> raise HOLD_ENTER_DEG to 0.7. Settles too far off -> lower it to 0.3.
    private static final double HOLD_ENTER_RAD = Math.toRadians(0.5);
    private static final double HOLD_EXIT_RAD = SHOOT_READY_RAD;
    // While SHOOTING (gamepad 2 R2 or auto-shoot) the hold lets the turret be knocked this far before
    // it corrects, and the breakaway kick is off. Still shakes while shooting -> raise to 3.0.
    // Balls drift off target during a burst -> lower to 1.5.
    private static final double SHOOT_HOLD_EXIT_RAD = Math.toRadians(2.0);
    private static final double HOLD_MAX_SPEED_RAD = Math.toRadians(5);      // rad/s: slower = settled
    private static final double HOLD_MAX_GOAL_RATE_RAD = Math.toRadians(5);  // rad/s: goal moving faster = no hold
    // Turret speed smoothing, 0..1 (higher = less smoothing). Lower it if vibration still shows in the speed.
    private static final double SPEED_FILTER = 0.3;

    // ── Side aim correction: ADJUSTABLE ZONES (one table per alliance) ──────
    // Each row = { fromIn, toIn, turnDeg }. A row applies when  fromIn <= rightOfGoalIn < toIn.
    //   rightOfGoalIn = how far the robot is to the RIGHT of the goal center sideways (x), in inches,
    //                   as seen from behind the shooter looking at the goal.
    //                   Positive = right side, NEGATIVE = left side. Shown in the "Goal side" telemetry.
    //   turnDeg       = extra turret turn. NEGATIVE = turret turns RIGHT, POSITIVE = turns LEFT.
    // The FIRST matching row wins, so keep narrow/custom zones ABOVE the wide ones.
    // A row of {0, 0, 0} is EMPTY and ignored. Use +-999 for "no limit".
    // If NO row matches, AIM_DEFAULT_DEG is used.

    // ONE TABLE PER ALLIANCE + PRESET (red 0, red 1, blue 0, blue 1). Tune each one on its own.
    // The table used is picked by the INIT alliance (dpad) and the preset (gamepad 1 B).
    // Rows are commented out = off. Uncomment a table's rows to turn its correction on.

    // RED preset 0.
    private static final double[][] RED0_AIM_ZONES = {
//            {-15,   15,  0.0},   // within 15 in of goal center -> aim dead center
//            {-30,  -15,  1.5},   // 15-30 in LEFT of goal       -> 1.5 deg left
//            {-999, -30,  3.0},   // more than 30 in LEFT        -> 3.0 deg left
//            { 15,   30, -1.5},   // 15-30 in RIGHT of goal      -> 1.5 deg right
//            { 30,  999, -3.0},   // more than 30 in RIGHT       -> 3.0 deg right
//            {-999, 999, 0.0},    // fallback, keep last
    };

    // RED preset 1.
    private static final double[][] RED1_AIM_ZONES = {
//            {-15,   15,  0.0},   // within 15 in of goal center -> aim dead center
//            {-30,  -15,  1.5},   // 15-30 in LEFT of goal       -> 1.5 deg left
//            {-999, -30,  3.0},   // more than 30 in LEFT        -> 3.0 deg left
//            { 15,   30, -1.5},   // 15-30 in RIGHT of goal      -> 1.5 deg right
//            { 30,  999, -3.0},   // more than 30 in RIGHT       -> 3.0 deg right
//            {-999, 999, 0.0},    // fallback, keep last
    };

    // BLUE preset 0.
    private static final double[][] BLUE0_AIM_ZONES = {
//            {-15,   15,  0.0},   // within 15 in of goal center -> aim dead center
//            {-30,  -15,  1.5},   // 15-30 in LEFT of goal       -> 1.5 deg left
//            {-999, -30,  3.0},   // more than 30 in LEFT        -> 3.0 deg left
//            { 15,   30, -1.5},   // 15-30 in RIGHT of goal      -> 1.5 deg right
//            { 30,  999, -3.0},   // more than 30 in RIGHT       -> 3.0 deg right
//            {-999, 999, 0.0},    // fallback, keep last
    };

    // BLUE preset 1.
    private static final double[][] BLUE1_AIM_ZONES = {
//            {-15,   15,  0.0},   // within 15 in of goal center -> aim dead center
//            {-30,  -15,  1.5},   // 15-30 in LEFT of goal       -> 1.5 deg left
//            {-999, -30,  3.0},   // more than 30 in LEFT        -> 3.0 deg left
//            { 15,   30, -1.5},   // 15-30 in RIGHT of goal      -> 1.5 deg right
//            { 30,  999, -3.0},   // more than 30 in RIGHT       -> 3.0 deg right
//            {-999, 999, 0.0},    // fallback, keep last
    };
    private static final double AIM_DEFAULT_DEG = 0.0;

    // PER-PRESET SIDE TRIM. Row = preset 0, preset 1. Each row = { leftOfGoalDeg, rightOfGoalDeg }.
    // POSITIVE = turret turns LEFT, NEGATIVE = turns RIGHT (same as AIM_ZONES). Added on top of AIM_ZONES.
    private static final double[][] RED_SIDE_TRIM = {
            // Tested values (before pivot was set): red 0 {0.0, -1.5}, red 1 {3.0, -0.5}
            { 0.0,  0.0},   // red 0
            { 0.0,  0.0},   // red 1
    };
    private static final double[][] BLUE_SIDE_TRIM = {
            // Tested values (before pivot was set): blue 0 {2.0, 0.0}, blue 1 {2.0, 0.0}
            { 0.0,  0.0},   // blue 0
            { 0.0,  0.0},   // blue 1
    };
    // Within this many inches of the goal center the trim blends smoothly from left to right (no jump).
    private static final double TRIM_BLEND_IN = 12.0;

    // ── Voltage compensation (turret, homing, shooter) ───────────────────────
    // Standard compensation: power x (NOMINAL_VOLTAGE / battery). Nominal is 14.0 V (not the usual 12.0)
    // because this robot was tuned on a 14 V battery. Below 14 V power goes up, above 14 V it goes down.
    //   Turret + homing: motor power x scale (capped at 1.0).
    //   Shooter: its PIDF F (feedforward) x scale, so setVelocity holds the same speed on a low battery.
    private static final double NOMINAL_VOLTAGE = 14.0;
    private static final double VOLT_READ_MS = 250;   // read the battery this often (each read costs loop time)
    private static final double VOLT_FILTER = 0.2;    // 0..1 smoothing, so current spikes don't jerk the scale
    private final ElapsedTime voltTimer = new ElapsedTime();
    private double batteryVolts = NOMINAL_VOLTAGE;
    private double voltScale = 1.0;
    private double shooterFScale = -1; // scale last sent to the shooter PIDF (-1 = never sent)

    // ── Turret state ─────────────────────────────────────────────────────────
    private DcMotorEx turret;
    private boolean aimOn = true;
    private boolean turretKilled = false; // L2 kill switch: all turret power cut
    private final ElapsedTime dtTimer = new ElapsedTime();
    private double lastWanted = 0;
    private double lastTarget = 0;
    private double spinRate = 0;
    private double errInt = 0; // integral of turret error (rad*s), only builds near the target
    private double stuckSinceSec = -1; // gameTimer time the turret got stuck off target (-1 = not stuck)
    private int lastMoveDir = 1;       // +1 / -1: direction the turret last really moved (backlash)
    private boolean holding = false;   // anti-shake hold zone active (power 0)
    private double speedF = 0;         // filtered turret speed, rad/s
    private double seenMinDeg = 0, seenMaxDeg = 0; // INIT limit finder
    // Turret angle when raw ticks = 0. zeroTurretHere() sets it so the touch-sensor spot reads HOME_SWITCH_RAD.
    private double turretOffsetRad = TURRET_START_RAD;
    private int homeRawTicks = 0;   // raw ticks at the moment the touch sensor was hit
    private boolean homed = false;  // true once the touch sensor has set the zero


    // ── Turret homing (on PLAY, see header for the step order) ───────────────
    private enum HomeStep { SERVO_UP, SEEK_SWITCH, SERVO_DOWN, DONE }
    private static final String HOME_SERVO_NAME = "homeservo";   // must match the Robot Configuration
    private static final String HOME_TOUCH_NAME = "touch";       // must match the Robot Configuration
    private static final double HOME_SERVO_UP = 0.394;    // sensor raised into the turret's path
    private static final double HOME_SERVO_DOWN = 0.0;    // sensor out of the way
    private static final double HOME_POWER = 0.6;         // homing power at NOMINAL_VOLTAGE (+ = left)
    // HOME OFFSET: the REAL zero is this many ticks from the touch-sensor spot. Average of 6 runs:
    // -275, -250, -265, -244, -255, -233 -> -254 (about -7.0 deg, spread +-21 ticks = +-0.6 deg).
    private static final int REAL_ZERO_TICKS_FROM_HOME = -254;
    private static final double HOME_SWITCH_RAD = -ENCODER_SIGN * REAL_ZERO_TICKS_FROM_HOME / TICKS_PER_RAD;
    private static final double HOME_SWITCH_DEG = Math.toDegrees(HOME_SWITCH_RAD);
    private static final double HOME_SERVO_MS = 300;      // wait for the servo to move (up and down)
    private static final double HOME_TIMEOUT_S = 5.0;     // give up looking for the sensor after this long
    private Servo homeServo;
    private TouchSensor homeTouch;
    private HomeStep homeStep = HomeStep.DONE;
    private boolean homeFailed = false;
    private final ElapsedTime homeTimer = new ElapsedTime();
    // Homing debug (telemetry only)
    private String homeEndReason = "not finished yet";
    private boolean touchEverPressed = false; // sensor seen pressed at any time since PLAY
    private int seekStartTicks = 0;           // raw ticks when the turret started turning toward the sensor
    private double zeroSetAtSec = -1;         // game time when the touch zero was set (-1 = never)

    // ── Alliance / preset ────────────────────────────────────────────────────
    private Alliance alliance = Alliance.RED;
    private int presetIndex = 0;

    // ── Shooter ──────────────────────────────────────────────────────────────
    private DcMotorEx leftShooter; // single shooter motor "ls"
    private static final double SHOOTER_P = 1.5;
    private static final double SHOOTER_F_LEFT = 13.0149;
    private static final double SHOOTER_START_VELOCITY = 1200; // preset the shooter starts at
    private double shooterTarget = SHOOTER_START_VELOCITY;
    private static final double VELOCITY_STEP = 50;
    private static final double VELOCITY_MIN = 0;
    private boolean shooterOn = false;

    // Game-long ramp: 0 -> GAME_RAMP_EXTRA over GAME_RAMP_SECS from PLAY, then holds.
    private static final double GAME_RAMP_SECS = 120;
    private static final double GAME_RAMP_EXTRA = 25;
    private final ElapsedTime gameTimer = new ElapsedTime();

    // ── Intake ───────────────────────────────────────────────────────────────
    private DcMotorEx leftIntake, rightIntake;
    private static final double INTAKE_POWER = 1.0;

    // ── Flower ───────────────────────────────────────────────────────────────
    private Servo flowerA, flowerB;
    private static final String FLOWER_A_NAME = "linkageright"; // FORWARD
    private static final String FLOWER_B_NAME = "linkageleft";  // REVERSED
    private static final double FLOWER_DOWN = 0.12;
    private static final double FLOWER_UP = 0.68;
    private boolean flowerUp = false;

    // ── Blocker / kicker ─────────────────────────────────────────────────────
    private Servo blocker;
    private static final double BLOCKER_CLOSED = 1.0;
    private static final double BLOCKER_OPEN = 0.6;

    //    private Servo kicker;
//    private static final double KICKER_BASE = 0.5;
//    private static final double KICKER_KICK = 0.8;
    private static final long KICK_DELAY_MS = 500;
    private final ElapsedTime kickTimer = new ElapsedTime();
    private boolean lastShooting = false;

    // ── Prism ────────────────────────────────────────────────────────────────
    private GoBildaPrismDriver prism;

    // ── Shoot trigger ────────────────────────────────────────────────────────
    private static final double TRIGGER_THRESHOLD = 0.5;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopMs = 0;

    // ── Rising-edge flags ────────────────────────────────────────────────────
    private boolean lastB, lastX1, lastRB1, lastL2;
    private boolean lastDpadUp, lastDpadLeft, lastDpadRight;
    private boolean lastY2;

    // ── Helpers ──────────────────────────────────────────────────────────────
    private Pose resetPose() {
        return (alliance == Alliance.RED ? RED_RESET_POSES : BLUE_RESET_POSES)[presetIndex];
    }

    private double[] goal() {
        return (alliance == Alliance.RED ? RED_GOALS : BLUE_GOALS)[presetIndex];
    }

    /**
     * Inches the robot is to the RIGHT of the goal center (negative = left), as seen from behind the
     * shooter looking at the goal. Preset 0 shoots toward +y, so right = +x; preset 1 toward -y, so right = -x.
     */
    private double rightOfGoalIn(Pose pose) {
        double dx = pose.x() - goal()[0];
        return pose.y() < goal()[1] ? dx : -dx;
    }

    /** The AIM_ZONES table for the current alliance AND preset (red 0, red 1, blue 0, blue 1). */
    private double[][] aimZones() {
        if (alliance == Alliance.RED) return presetIndex == 0 ? RED0_AIM_ZONES : RED1_AIM_ZONES;
        return presetIndex == 0 ? BLUE0_AIM_ZONES : BLUE1_AIM_ZONES;
    }

    /** Index of the first non-empty row in this alliance's table that contains rightIn, or -1 if none. */
    private int aimZoneIndex(double rightIn) {
        double[][] zones = aimZones();
        for (int i = 0; i < zones.length; i++) {
            double[] z = zones[i];
            if (z[0] == 0 && z[1] == 0 && z[2] == 0) continue; // empty slot
            if (rightIn >= z[0] && rightIn < z[1]) return i;
        }
        return -1;
    }

    /** "Turret off" = kill switch on, or aim switched off. Only then may R2 auto-drive. */
    private boolean turretOff() {
        return turretKilled || !aimOn;
    }

    private static double wrap(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    private static double angleDiff(double a, double b) {
        return Math.atan2(Math.sin(a - b), Math.cos(a - b));
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double deadzone(double v) {
        return Math.abs(v) < DRIVE_DEADZONE ? 0 : v;
    }

    /** Encoder reading in radians, before the offset. */
    private double rawTurretRad() {
        return ENCODER_SIGN * turret.getCurrentPosition() / TICKS_PER_RAD;
    }

    private double turretAngleRad() {
        // Backlash: zero was set while turning + (homing). After turning -, the ring is BACKLASH_RAD
        // further - than the encoder gear shows.
        return turretOffsetRad + rawTurretRad() + (lastMoveDir < 0 ? -BACKLASH_RAD : 0);
    }

    /** Remember which way the turret last really moved (for backlash). Call once per loop. */
    private void updateMoveDir() {
        double speed = turretSpeedRadPerSec();
        if (Math.abs(speed) > BACKLASH_MOVE_RAD) lastMoveDir = speed > 0 ? 1 : -1;
    }

    private double turretSpeedRadPerSec() {
        return ENCODER_SIGN * turret.getVelocity() / TICKS_PER_RAD;
    }

    /**
     * Touch sensor hit: the turret's position RIGHT NOW becomes angle HOME_SWITCH_RAD.
     * Done in software (offset from the current raw ticks), so it works no matter where the turret was at INIT.
     */
    private void zeroTurretHere() {
        homeRawTicks = turret.getCurrentPosition();
        turretOffsetRad = HOME_SWITCH_RAD - rawTurretRad();
        lastTarget = HOME_SWITCH_RAD;
        errInt = 0;
        lastMoveDir = 1; // homing turns + (left), the direction BACKLASH is measured from
        homed = true;
        zeroSetAtSec = gameTimer.seconds();
    }

    /** Encoder ticks from the touch-sensor zero (0 = at the sensor). */
    private int ticksFromHome() {
        return turret.getCurrentPosition() - homeRawTicks;
    }

    /** Turret angle (rad, turret frame) that points the shooter from this pose at point (gx, gy). Trim not included. */
    private double wantedTurretAngle(Pose pose, double gx, double gy) {
        double h = pose.heading();
        double px = pose.x() + PIVOT_X_IN * Math.cos(h) - PIVOT_Y_IN * Math.sin(h);
        double py = pose.y() + PIVOT_X_IN * Math.sin(h) + PIVOT_Y_IN * Math.cos(h);
        double fieldAngle = Math.atan2(gy - py, gx - px);
        return wrap(fieldAngle - h - TURRET_ZERO_OFFSET_RAD);
    }

    /**
     * Picks which equivalent angle (+-360 deg) the turret goes to: stays continuous with last loop's
     * target, and only jumps (unwinds) when that would leave the turret's range.
     * If nothing fits (gap in the range), waits at the nearest limit.
     */
    private double fitToRange(double angle, double prevTarget) {
        double w = wrap(angle);
        double twoPi = 2 * Math.PI;
        double cont = w + Math.round((prevTarget - w) / twoPi) * twoPi;
        if (cont >= TURRET_MIN_RAD && cont <= TURRET_MAX_RAD) return cont;

        double best = Double.NaN;
        for (int k = -1; k <= 1; k++) {
            double a = w + k * twoPi;
            if (a < TURRET_MIN_RAD || a > TURRET_MAX_RAD) continue;
            if (Double.isNaN(best) || Math.abs(a - prevTarget) < Math.abs(best - prevTarget)) best = a;
        }
        if (!Double.isNaN(best)) return best;
        return Math.max(TURRET_MIN_RAD, Math.min(TURRET_MAX_RAD, cont));
    }

    /** Lowest live reading of all hub voltage sensors, or -1 if none reads. */
    private double readBatteryVolts() {
        double v = -1;
        for (VoltageSensor s : hardwareMap.voltageSensor) {
            double r = s.getVoltage();
            if (r > 0 && (v < 0 || r < v)) v = r;
        }
        return v;
    }

    /** Updates batteryVolts / voltScale every VOLT_READ_MS. force = read now and skip the filter. */
    private void updateVoltScale(boolean force) {
        if (!force && voltTimer.milliseconds() < VOLT_READ_MS) return;
        voltTimer.reset();
        double v = readBatteryVolts();
        if (v <= 0) return; // no reading: keep last value
        batteryVolts = force ? v : batteryVolts + VOLT_FILTER * (v - batteryVolts);
        voltScale = NOMINAL_VOLTAGE / batteryVolts;

        // Shooter feedforward. Only resend when it changed by more than 0.5% (each send costs loop time).
        if (shooterFScale < 0 || Math.abs(voltScale - shooterFScale) > 0.005) {
            leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F_LEFT * voltScale));
            shooterFScale = voltScale;
        }
    }

    /** Motor power with voltage compensation, capped at +-1.0. */
    private double voltScaled(double power) {
        return clamp(power * voltScale, -1.0, 1.0);
    }

    private void nextHomeStep(HomeStep step) {
        homeStep = step;
        homeTimer.reset();
    }

    /**
     * Turret power while homing: servo up -> left until the touch sensor is hit
     * -> servo down (turret held still) -> DONE (tracking starts). L2 kill aborts.
     */
    private double homingPower() {
        if (turretKilled) { // abort: servo out of the way, no power
            homeServo.setPosition(HOME_SERVO_DOWN);
            homeEndReason = "ABORTED by L2 kill switch during " + homeStep;
            homeStep = HomeStep.DONE;
            lastTarget = turretAngleRad();
            errInt = 0;
            return 0;
        }
        switch (homeStep) {
            case SERVO_UP:
                homeServo.setPosition(HOME_SERVO_UP);
                if (homeTimer.milliseconds() >= HOME_SERVO_MS) {
                    seekStartTicks = turret.getCurrentPosition();
                    nextHomeStep(HomeStep.SEEK_SWITCH);
                }
                return 0;

            case SEEK_SWITCH:
                if (homeTouch.isPressed()) {
                    homeEndReason = String.format("TOUCH PRESSED after %.2f s, turned %d ticks",
                            homeTimer.seconds(), turret.getCurrentPosition() - seekStartTicks);
                    zeroTurretHere();                        // touch hit: THIS spot = turret 0
                    nextHomeStep(HomeStep.SERVO_DOWN);
                    return 0;
                }
                if (homeTimer.seconds() > HOME_TIMEOUT_S) { // never hit: keep INIT zero
                    homeEndReason = String.format("TIMEOUT %.1f s, sensor never pressed, turned %d ticks",
                            HOME_TIMEOUT_S, turret.getCurrentPosition() - seekStartTicks);
                    homeFailed = true;
                    nextHomeStep(HomeStep.SERVO_DOWN);
                    return 0;
                }
                return HOME_POWER; // left (voltage compensated in loop)

            case SERVO_DOWN: // turret held still until the sensor arm is clear
                homeServo.setPosition(HOME_SERVO_DOWN);
                if (homeTimer.milliseconds() >= HOME_SERVO_MS) {
                    homeStep = HomeStep.DONE; // tracking takes over next loop
                    lastTarget = turretAngleRad();
                    errInt = 0;
                }
                return 0;

            default:
                return 0;
        }
    }

    private void setFlowerPosition(double position) {
        flowerA.setPosition(position);
        flowerB.setPosition(position);
    }

    // ═════════════════════════════════════════════════════════════════════════
    // Shooting-zone helpers (from AutoShootDodgeRange)
    // ═════════════════════════════════════════════════════════════════════════

    /** Heading (radians) that points the back of the robot (turret at zero) at the goal from (x, y). */
    private double headingToGoal(double x, double y) {
        double[] g = goal();
        return Math.atan2(g[1] - y, g[0] - x) + TURRET_ZERO_OFFSET_RAD;
    }

    /**
     * Fastest legal shooting pose: the point inside the shooting zone nearest to the robot right now,
     * with the heading that lines the back of the robot up on the goal.
     */
    private Pose computeShootPose() {
        Pose cur = follower.pose();
        double xMin = alliance == Alliance.RED ? RED_X_MIN : BLUE_X_MIN;
        double xMax = alliance == Alliance.RED ? RED_X_MAX : BLUE_X_MAX;
        double x = clamp(cur.x(), xMin + ZONE_MARGIN, xMax - ZONE_MARGIN);
        double yLo = (presetIndex == 0) ? FIELD_MIN + WALL_MARGIN : PRESET1_MIN_Y + ZONE_MARGIN;
        double yHi = (presetIndex == 0) ? PRESET0_MAX_Y - ZONE_MARGIN : FIELD_MAX - WALL_MARGIN;
        double y = clamp(cur.y(), yLo, yHi); // fallback: nearest point in the zone

        // Back off to SHOOT_DISTANCE from the goal (keeping the same x) if that is possible.
        double[] g = goal();
        double dx = x - g[0];
        double centerDist = SHOOT_DISTANCE + SHOOTER_OFFSET_IN;
        if (centerDist > Math.abs(dx)) {
            double dy = Math.sqrt(centerDist * centerDist - dx * dx);
            double idealY = (presetIndex == 0) ? g[1] - dy : g[1] + dy;
            y = clamp(idealY, yLo, yHi);
        }
        return p.of(x, y, Math.toDegrees(headingToGoal(x, y)));
    }

    /** Aim error (radians) from the robot's CURRENT position: + = shot too far left. */
    private double aimErrorRad() {
        Pose now = follower.pose();
        return angleDiff(now.heading(), headingToGoal(now.x(), now.y()));
    }

    private void enterArrived() {
        alignState = AlignState.ARRIVED;
        aimLocked = false;
        arrivedTimer.reset();
        reaimTimer.reset();
    }

    /** True only in MANUAL. While ALIGNING the path runs; while ARRIVED Pedro holds the target. */
    private boolean driverHasControl() {
        return alignState == AlignState.MANUAL;
    }

    /**
     * Gamepad 1 R2 (hold, turret off) = auto-drive to the shooting zone.
     * MANUAL   — driver (plain stick drive)
     * ALIGNING — path is running
     * ARRIVED  — path done; Pedro HOLDS the target while R2 stays held, re-aims, then aim-lock gates shooting.
     */
    private void updateAlign() {
        boolean alignHeld = gamepad1.right_trigger > TRIGGER_THRESHOLD && turretOff();
        boolean rising = alignHeld && !alignWasHeld;
        alignWasHeld = alignHeld;

        switch (alignState) {

            case MANUAL:
                if (rising) {
                    alignTarget = computeShootPose();
                    Pose now = follower.pose();
                    turnOnly = Math.hypot(alignTarget.x() - now.x(), alignTarget.y() - now.y()) < 0.75;
                    if (turnOnly) {
                        schedule(hold(follower, alignTarget)); // already in the zone: hold and rotate onto the goal
                    } else {
                        scheduleRoute(alignTarget);
                    }
                    alignStarted = false;
                    alignState = AlignState.ALIGNING;
                }
                break;

            case ALIGNING:
                if (!alignHeld) {
                    Scheduler.reset(); // clears ALL scheduled commands
                    alignState = AlignState.MANUAL;
                } else if (turnOnly) {
                    Pose now = follower.pose();
                    boolean headingOk = Math.abs(angleDiff(now.heading(), alignTarget.heading())) < HEADING_TOL_RAD;
                    boolean posOk = Math.hypot(alignTarget.x() - now.x(), alignTarget.y() - now.y()) < POSITION_TOL_IN;
                    if (headingOk && posOk) enterArrived(); // hold already scheduled
                } else if (follower.isBusy()) {
                    alignStarted = true; // path is genuinely running
                } else if (alignStarted) {
                    schedule(hold(follower, alignTarget)); // reached the target: now hold it
                    enterArrived();
                }
                // !started and !busy: command queued, not started yet. Wait.
                break;

            case ARRIVED:
                if (!alignHeld) {
                    Scheduler.reset(); // stops the hold
                    alignState = AlignState.MANUAL;
                    aimLocked = false;
                    break;
                }

                // Re-aim from where the robot actually is (it may have stopped a little off, or been pushed).
                Pose now = follower.pose();
                double want = headingToGoal(now.x(), now.y());
                if (Math.abs(angleDiff(want, alignTarget.heading())) > REAIM_RAD
                        && reaimTimer.milliseconds() > REAIM_EVERY_MS) {
                    alignTarget = p.of(alignTarget.x(), alignTarget.y(), Math.toDegrees(want));
                    Scheduler.reset();
                    schedule(hold(follower, alignTarget));
                    reaimTimer.reset();
                }
                // Only shoot while really aimed. Pauses (and the kick delay restarts) if bumped off aim.
                double aimErr = Math.abs(aimErrorRad());
                double okTol = arrivedTimer.milliseconds() > AIM_LATE_MS ? AIM_LATE_RAD : AIM_OK_RAD;
                if (!aimLocked && aimErr < okTol) aimLocked = true;
                else if (aimLocked && aimErr > Math.max(AIM_LOST_RAD, okTol)) aimLocked = false;
                break;
        }
    }

    /** Straight line from the robot's current pose to the given target (heading turns as it drives). */
    private Path pathTo(Pose target) {
        Pose currentPose = follower.pose();
        return line(currentPose, target).linear(currentPose, target);
    }

    /** Schedules the drive to the target: straight if clear, else one rounded path around the structures. */
    private void scheduleRoute(Pose target) {
        Pose start = follower.pose();
        routeCorners = 0;
        if (isBlocked(start.x(), start.y(), target.x(), target.y())) {
            Path route = buildCurvedPath(start, target);
            if (route != null) {
                schedule(follow(follower, route));
                return;
            }
        }
        schedule(follow(follower, pathTo(target))); // clear, or no safe route: straight
    }

    /**
     * Builds the dodge path: lines joined by quadratic-Bezier rounded corners, with the heading turning
     * steadily from the current heading to the target's. Returns null if no safe route exists.
     */
    private Path buildCurvedPath(Pose start, Pose target) {
        double pad = ROBOT_MARGIN + CORNER_RADIUS * 0.5; // rounding cuts in by at most ~R/2
        for (int attempt = 0; attempt < PAD_TRIES; attempt++, pad += PAD_STEP) {
            List<double[]> corners = findCorners(start, target, pad);
            if (corners == null || corners.isEmpty()) return null;

            List<double[]> pts = new ArrayList<>();
            pts.add(new double[]{start.x(), start.y()});
            pts.addAll(corners);
            pts.add(new double[]{target.x(), target.y()});

            List<double[][]> segs = roundedSegments(pts);
            if (!segmentsClear(segs, start, target)) continue; // cut too close: widen, retry

            // Heading: spread the turn along the path in proportion to distance.
            double h0 = start.heading();
            double delta = Math.atan2(Math.sin(target.heading() - h0), Math.cos(target.heading() - h0));
            double total = 0;
            for (double[][] sg : segs) total += controlLength(sg);

            List<Path> parts = new ArrayList<>();
            double run = 0;
            for (double[][] sg : segs) {
                double len = controlLength(sg);
                double hA = h0 + delta * (run / total);
                double hB = h0 + delta * ((run + len) / total);
                run += len;

                Pose a = p.of(sg[0][0], sg[0][1], Math.toDegrees(hA));
                Pose b = p.of(sg[sg.length - 1][0], sg[sg.length - 1][1], Math.toDegrees(hB));
                if (sg.length == 2) {
                    parts.add(line(a, b).linear(a, b));
                } else {
                    Pose m = p.of(sg[1][0], sg[1][1], Math.toDegrees(hA)); // corner = control point
                    parts.add(curve(a, m, b).linear(a, b));
                }
            }
            routeCorners = corners.size();
            return path(parts.toArray(new Path[0]));
        }
        return null;
    }

    /** Polyline start -> corners -> target as segments: straight runs (2 pts) and rounded corners (3 pts). */
    private List<double[][]> roundedSegments(List<double[]> pts) {
        List<double[][]> out = new ArrayList<>();
        double[] cur = pts.get(0);
        for (int k = 1; k < pts.size() - 1; k++) {
            double[] a = pts.get(k - 1), c = pts.get(k), b = pts.get(k + 1);
            double lenIn = Math.hypot(c[0] - a[0], c[1] - a[1]);
            double lenOut = Math.hypot(b[0] - c[0], b[1] - c[1]);
            double rIn = Math.min(CORNER_RADIUS, lenIn / 2);
            double rOut = Math.min(CORNER_RADIUS, lenOut / 2);
            double[] entry = {c[0] - (c[0] - a[0]) / lenIn * rIn, c[1] - (c[1] - a[1]) / lenIn * rIn};
            double[] exit = {c[0] + (b[0] - c[0]) / lenOut * rOut, c[1] + (b[1] - c[1]) / lenOut * rOut};
            if (Math.hypot(entry[0] - cur[0], entry[1] - cur[1]) > 0.01) {
                out.add(new double[][]{cur, entry});
            }
            out.add(new double[][]{entry, c, exit});
            cur = exit;
        }
        out.add(new double[][]{cur, pts.get(pts.size() - 1)});
        return out;
    }

    private static double controlLength(double[][] sg) {
        double len = 0;
        for (int i = 1; i < sg.length; i++) {
            len += Math.hypot(sg[i][0] - sg[i - 1][0], sg[i][1] - sg[i - 1][1]);
        }
        return len;
    }

    /**
     * Picks which padded zone corners the robot wraps around, in order. Visibility graph over start,
     * target and every usable corner, shortest path by Dijkstra. Returns null if no route exists.
     */
    private List<double[]> findCorners(Pose start, Pose target, double pad) {
        List<double[]> nodes = new ArrayList<>();
        nodes.add(new double[]{start.x(), start.y()});
        nodes.add(new double[]{target.x(), target.y()});
        for (double[] z : KEEP_OUT_ZONES) {
            if (!active(z)) continue;
            double[][] corners = {
                    {z[0] - pad, z[1] - pad},
                    {z[2] + pad, z[1] - pad},
                    {z[2] + pad, z[3] + pad},
                    {z[0] - pad, z[3] + pad}
            };
            for (double[] c : corners) {
                boolean onField = c[0] >= FIELD_MIN && c[0] <= FIELD_MAX
                        && c[1] >= FIELD_MIN && c[1] <= FIELD_MAX;
                if (onField && !insideAnyZone(c[0], c[1])) nodes.add(c);
            }
        }

        int n = nodes.size();
        double[] dist = new double[n];
        int[] prev = new int[n];
        boolean[] done = new boolean[n];
        java.util.Arrays.fill(dist, Double.MAX_VALUE);
        java.util.Arrays.fill(prev, -1);
        dist[0] = 0;
        for (int iter = 0; iter < n; iter++) {
            int u = -1;
            for (int i = 0; i < n; i++) {
                if (!done[i] && dist[i] < Double.MAX_VALUE && (u == -1 || dist[i] < dist[u])) u = i;
            }
            if (u == -1) break;
            done[u] = true;
            for (int v = 0; v < n; v++) {
                if (done[v]) continue;
                double[] a = nodes.get(u), b = nodes.get(v);
                if (isBlocked(a[0], a[1], b[0], b[1])) continue;
                double d = dist[u] + Math.hypot(b[0] - a[0], b[1] - a[1]);
                if (d < dist[v]) { dist[v] = d; prev[v] = u; }
            }
        }
        if (dist[1] == Double.MAX_VALUE) return null;

        List<Integer> chain = new ArrayList<>();
        for (int at = 1; at != -1; at = prev[at]) chain.add(at);
        Collections.reverse(chain); // 0 (start) ... 1 (target)
        List<double[]> out = new ArrayList<>();
        for (int i = 1; i < chain.size() - 1; i++) out.add(nodes.get(chain.get(i)));
        return out;
    }

    /** Samples every path segment and checks it never comes within ROBOT_MARGIN of a structure. */
    private boolean segmentsClear(List<double[][]> segs, Pose start, Pose target) {
        final double eps = 0.01;
        for (double[][] sg : segs) {
            for (int i = 0; i <= CURVE_SAMPLES; i++) {
                double[] pt = bezierPoint(sg, (double) i / CURVE_SAMPLES);
                for (double[] z : KEEP_OUT_ZONES) {
                    if (!active(z)) continue;
                    double x0 = z[0] - ROBOT_MARGIN + eps, y0 = z[1] - ROBOT_MARGIN + eps;
                    double x1 = z[2] + ROBOT_MARGIN - eps, y1 = z[3] + ROBOT_MARGIN - eps;
                    if (inside(start.x(), start.y(), x0, y0, x1, y1)
                            || inside(target.x(), target.y(), x0, y0, x1, y1)) continue;
                    if (inside(pt[0], pt[1], x0, y0, x1, y1)) return false;
                }
            }
        }
        return true;
    }

    /** Point on a Bezier segment (2 or 3 control points) at t in [0,1], by de Casteljau. */
    private static double[] bezierPoint(double[][] ctrl, double t) {
        int n = ctrl.length;
        double[] x = new double[n], y = new double[n];
        for (int i = 0; i < n; i++) { x[i] = ctrl[i][0]; y[i] = ctrl[i][1]; }
        for (int k = n - 1; k > 0; k--) {
            for (int i = 0; i < k; i++) {
                x[i] = (1 - t) * x[i] + t * x[i + 1];
                y[i] = (1 - t) * y[i] + t * y[i + 1];
            }
        }
        return new double[]{x[0], y[0]};
    }

    /** True if the segment a-b passes through any padded keep-out zone. */
    private boolean isBlocked(double ax, double ay, double bx, double by) {
        final double eps = 0.01; // so points exactly on a padded edge count as outside
        for (double[] z : KEEP_OUT_ZONES) {
            if (!active(z)) continue;
            double x0 = z[0] - ROBOT_MARGIN + eps, y0 = z[1] - ROBOT_MARGIN + eps;
            double x1 = z[2] + ROBOT_MARGIN - eps, y1 = z[3] + ROBOT_MARGIN - eps;
            // If an end is already inside this zone, don't let that zone block the leg (or we could never leave).
            if (inside(ax, ay, x0, y0, x1, y1) || inside(bx, by, x0, y0, x1, y1)) continue;
            if (segmentHitsRect(ax, ay, bx, by, x0, y0, x1, y1)) return true;
        }
        return false;
    }

    private boolean insideAnyZone(double x, double y) {
        final double eps = 0.01;
        for (double[] z : KEEP_OUT_ZONES) {
            if (!active(z)) continue;
            if (inside(x, y, z[0] - ROBOT_MARGIN + eps, z[1] - ROBOT_MARGIN + eps,
                    z[2] + ROBOT_MARGIN - eps, z[3] + ROBOT_MARGIN - eps)) return true;
        }
        return false;
    }

    /** A zone left as {0,0,0,0} is a placeholder and is ignored. */
    private static boolean active(double[] z) {
        return !(z[0] == 0 && z[1] == 0 && z[2] == 0 && z[3] == 0);
    }

    private static boolean inside(double x, double y, double x0, double y0, double x1, double y1) {
        return x > x0 && x < x1 && y > y0 && y < y1;
    }

    /** Liang–Barsky segment vs. axis-aligned rectangle test. */
    private static boolean segmentHitsRect(double ax, double ay, double bx, double by,
                                           double x0, double y0, double x1, double y1) {
        double t0 = 0, t1 = 1;
        double dx = bx - ax, dy = by - ay;
        double[] pp = {-dx, dx, -dy, dy};
        double[] qq = {ax - x0, x1 - ax, ay - y0, y1 - ay};
        for (int i = 0; i < 4; i++) {
            if (pp[i] == 0) {
                if (qq[i] < 0) return false;
            } else {
                double t = qq[i] / pp[i];
                if (pp[i] < 0) {
                    if (t > t1) return false;
                    if (t > t0) t0 = t;
                } else {
                    if (t < t0) return false;
                    if (t < t1) t1 = t;
                }
            }
        }
        return true;
    }

    // ═════════════════════════════════════════════════════════════════════════
    // init
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        alliance = Alliance.RED;
        presetIndex = 0;
        follower.setPose(resetPose());
        alignTarget = resetPose();
        alignState = AlignState.MANUAL;
        alignWasHeld = false;
        alignStarted = false;

        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);

        turret = hardwareMap.get(DcMotorEx.class, TURRET_MOTOR);
        turret.setDirection(TURRET_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // No encoder reset here: turret 0 is set by the touch sensor on PLAY, not by where it sits at INIT.
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        homeRawTicks = turret.getCurrentPosition();
        turretOffsetRad = TURRET_START_RAD - rawTurretRad(); // fallback only (homing failed): INIT spot = 0
        homed = false;

        leftShooter = hardwareMap.get(DcMotorEx.class, "ls");
        leftShooter.setDirection(DcMotorSimple.Direction.FORWARD);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, 0, 0, SHOOTER_F_LEFT));

        leftIntake = hardwareMap.get(DcMotorEx.class, "li");
        rightIntake = hardwareMap.get(DcMotorEx.class, "ri");
        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flowerA = hardwareMap.get(Servo.class, FLOWER_A_NAME);
        flowerB = hardwareMap.get(Servo.class, FLOWER_B_NAME);
        flowerA.setDirection(Servo.Direction.FORWARD);
        flowerB.setDirection(Servo.Direction.REVERSE);
        flowerUp = true; // flower starts UP at INIT
        setFlowerPosition(FLOWER_UP);

        blocker = hardwareMap.get(Servo.class, "blocker");
        blocker.setPosition(BLOCKER_CLOSED);

//        kicker = hardwareMap.get(Servo.class, "kicker");
//        kicker.setPosition(KICKER_BASE);

        homeServo = hardwareMap.get(Servo.class, HOME_SERVO_NAME);
        homeTouch = hardwareMap.get(TouchSensor.class, HOME_TOUCH_NAME);
        homeStep = HomeStep.DONE;

        telemetry.addLine("Gamepad 1: dpad UP = RED, dpad DOWN = BLUE");
        telemetry.addLine("Turret homes on PLAY: left to touch sensor (= 0), servo down, then tracking");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up) alliance = Alliance.RED;
        if (gamepad1.dpad_down) alliance = Alliance.BLUE;
        follower.setPose(resetPose());

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start pose", "x=%.1f y=%.1f h=%.0f deg",
                resetPose().x(), resetPose().y(), Math.toDegrees(resetPose().heading()));
        // LIMIT FINDER: turn the turret by hand to each hard stop and read the angle.
        double angle = Math.toDegrees(turretAngleRad());
        seenMinDeg = Math.min(seenMinDeg, angle);
        seenMaxDeg = Math.max(seenMaxDeg, angle);
        telemetry.addData("Turret angle", "%.1f deg   raw ticks %d", angle, turret.getCurrentPosition());
        telemetry.addData("Lowest / highest seen", "%.1f / %.1f deg", seenMinDeg, seenMaxDeg);
        telemetry.addData("Touch sensor", homeTouch.isPressed() ? "PRESSED" : "not pressed");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(resetPose());
        alignTarget = resetPose();
        lastWanted = wantedTurretAngle(follower.pose(), goal()[0], goal()[1]);
        lastTarget = turretAngleRad();
        errInt = 0;
        homeFailed = false;
        homeEndReason = "not finished yet";
        touchEverPressed = false;
        zeroSetAtSec = -1;
        updateVoltScale(true); // real battery reading before homing starts
        nextHomeStep(HomeStep.SERVO_UP);
        gameTimer.reset();
        dtTimer.reset();
        loopTimer.reset();
    }

    // ═════════════════════════════════════════════════════════════════════════
    // loop
    // ═════════════════════════════════════════════════════════════════════════
    @Override
    public void loop() {
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // ── Gamepad 1 buttons (rising edges) ─────────────────────────────
        boolean b = gamepad1.b, x1 = gamepad1.x, rb1 = gamepad1.right_bumper;
        boolean l2 = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        if (b && !lastB) presetIndex = 1 - presetIndex;
        if (x1 && !lastX1 && driverHasControl()) follower.setPose(resetPose());
        if (rb1 && !lastRB1) aimOn = !aimOn;
        if (l2 && !lastL2) turretKilled = !turretKilled; // L2 = turret kill switch
        lastB = b; lastX1 = x1; lastRB1 = rb1; lastL2 = l2;

        // ── Auto-drive to shooting zone (gamepad 1 R2, turret off) ────────
        updateAlign();

        // ── Drive ────────────────────────────────────────────────────────
        // Pedro: +lateral = left, +turn = CCW. Sticks are + to the right, so negate.
        if (driverHasControl()) {
            follower.manual(deadzone(-gamepad1.left_stick_y),
                    -deadzone(gamepad1.left_stick_x),
                    -deadzone(gamepad1.right_stick_x));
        }
        follower.update();
        Scheduler.execute();

        // ── Turret ───────────────────────────────────────────────────────
        Pose pose = follower.pose();
        double dt = Math.max(dtTimer.seconds(), 1e-3);
        dtTimer.reset();

        // Rate the wanted angle changes (robot spin + translation), filtered.
        // Clamped so a pose/goal jump (X or B press) cannot kick the turret through the feedforward.
        double[] g0 = goal();
        double wanted = wantedTurretAngle(pose, g0[0], g0[1]);
        double rawRate = clamp(wrap(wanted - lastWanted) / dt, -MAX_GOAL_RATE_RAD, MAX_GOAL_RATE_RAD);
        lastWanted = wanted;
        spinRate += SPIN_FILTER * (rawRate - spinRate);

        // Side correction from this alliance's AIM_ZONES table (first matching row wins).
        double rightIn = rightOfGoalIn(pose);
        int zoneIdx = aimZoneIndex(rightIn);
        double sideOffset = Math.toRadians(zoneIdx >= 0 ? aimZones()[zoneIdx][2] : AIM_DEFAULT_DEG);
        // Per-preset side trim (blends from the left value to the right value across the goal center).
        double[] trim = (alliance == Alliance.RED ? RED_SIDE_TRIM : BLUE_SIDE_TRIM)[presetIndex];
        double t = clamp((rightIn + TRIM_BLEND_IN) / (2 * TRIM_BLEND_IN), 0, 1); // 0 = left, 1 = right
        sideOffset += Math.toRadians(trim[0] + t * (trim[1] - trim[0]));

        // tracking = turret follows the goal. Otherwise it is held at zero (parked / locked for auto-drive),
        // or, if killed and not auto-driving, left with no power at all.
        boolean homing = homeStep != HomeStep.DONE;
        boolean touchNow = homeTouch.isPressed();
        if (touchNow) touchEverPressed = true;
        HomeStep stepBefore = homeStep;
        double stepSecs = homeTimer.seconds();
        boolean tracking = aimOn && !turretKilled;
        boolean autoDriving = alignState != AlignState.MANUAL;
        boolean powerCut = turretKilled && !autoDriving;

        updateMoveDir();
        double current = turretAngleRad();
        double target = tracking ? fitToRange(wanted + sideOffset, lastTarget) : 0;
        lastTarget = tracking ? target : current; // tracking turned back ON starts from where the turret really is
        double err = target - current;

        // Shoot when gamepad 2 R2 is held, OR automatically once the robot has arrived at the shooting
        // zone with gamepad 1 R2 still held and the aim is locked.
        boolean shooting = gamepad2.right_trigger > TRIGGER_THRESHOLD
                || (alignState == AlignState.ARRIVED && aimLocked);

        double power = 0;
        boolean minPowerActive = false;
        boolean kickActive = false;
        // Shooter vibration shakes the encoder reading: smooth the speed before kD / kick / floor use it.
        speedF += SPEED_FILTER * (turretSpeedRadPerSec() - speedF);
        double speed = speedF;
        // kD brakes the DIFFERENCE between target speed and turret speed, so it does not fight
        // tracking while the robot moves. kV pushes the turret along at the target speed.
        double targetRate = tracking ? spinRate : 0;

        // HOLD ZONE: close to the target and the goal is not moving -> power 0, BRAKE holds the turret.
        // It only starts correcting again once the error grows past HOLD_EXIT_RAD, so shooter vibration
        // cannot start a shake. Driving / turning (goal moving) always leaves the hold.
        boolean goalStill = Math.abs(targetRate) < HOLD_MAX_GOAL_RATE_RAD;
        if (powerCut || homing) {
            holding = false;
        } else if (holding) {
            // While shooting, balls and feeding knock the turret: allow more before it corrects.
            double exitRad = shooting ? SHOOT_HOLD_EXIT_RAD : HOLD_EXIT_RAD;
            if (Math.abs(err) > exitRad || !goalStill) holding = false;
        } else if (Math.abs(err) < HOLD_ENTER_RAD && Math.abs(speed) < HOLD_MAX_SPEED_RAD && goalStill) {
            holding = true;
        }

        if (!powerCut && !holding) {
            // kS ramps in smoothly (full at KS_RAMP_RAD) so there is no on/off step at the target.
            double ksRamp = Math.max(-1.0, Math.min(1.0, err / KS_RAMP_RAD));
            power = kP * err + kD * (targetRate - speed) + kV * targetRate + ksRamp * kS;

            // Integral: only near the target (no windup during big swings), capped to I_MAX_POWER.
            // Inside I_HOLD_RAD it holds its value instead of growing, so it does not hunt back and forth.
            // kI = 0: the clamp limits become +-infinity, but kI * errInt is still 0.
            if (Math.abs(err) >= I_ZONE_RAD) {
                errInt = 0;
            } else if (Math.abs(err) > I_HOLD_RAD && Math.abs(speed) < I_MAX_SPEED_RAD) {
                errInt = clamp(errInt + err * dt, -I_MAX_POWER / kI, I_MAX_POWER / kI);
            }
            power += kI * errInt;

            // Minimum power floor: stuck short of the target -> push with at least MIN_POWER.
            if (Math.abs(err) > MIN_POWER_ERR_RAD && Math.abs(speed) < MIN_POWER_SPEED_RAD
                    && Math.abs(power) < MIN_POWER) {
                power = Math.signum(err) * MIN_POWER;
                minPowerActive = true;
            }

            double cap = Math.abs(err) > Math.toRadians(SWING_ERR_DEG) ? SWING_POWER : MAX_POWER;
            power = Math.max(-cap, Math.min(cap, power));

            // Breakaway kick: stuck off target -> ramp power up until it moves.
            // No kick while shooting: its big pushes are what turns shot knocks into a shake.
            boolean stuck = !shooting && Math.abs(err) > KICK_ERR_RAD && Math.abs(speed) < KICK_MOVING_RAD;
            if (!stuck) {
                stuckSinceSec = -1;
            } else {
                double now = gameTimer.seconds();
                if (stuckSinceSec < 0) stuckSinceSec = now;
                double kick = Math.min(KICK_POWER, MIN_POWER + KICK_RAMP_PER_S * (now - stuckSinceSec));
                if (kick > Math.abs(power)) {
                    power = Math.signum(err) * kick;
                    kickActive = true;
                }
            }
        } else {
            errInt = 0;
            stuckSinceSec = -1;
        }
        if (homing) power = homingPower(); // homing overrides aiming until the sequence is DONE
        updateVoltScale(false);
        double sentPower = voltScaled(power); // voltage compensation (tracking and homing)
        turret.setPower(POWER_SIGN * sentPower);

        // Ready to shoot: never while homing; manual with turret off = always;
        // tracking or auto-driving = within SHOOT_READY_RAD.
        boolean turretReady = !homing
                && ((tracking || autoDriving) ? Math.abs(err) <= SHOOT_READY_RAD : true);

        // ── Shooter mode (gamepad 2 dpad) ────────────────────────────────
        boolean dUp = gamepad2.dpad_up, dLeft = gamepad2.dpad_left, dRight = gamepad2.dpad_right;
        if (dUp && !lastDpadUp) shooterOn = !shooterOn;
        if (dRight && !lastDpadRight) shooterTarget += VELOCITY_STEP;
        if (dLeft && !lastDpadLeft) shooterTarget = Math.max(VELOCITY_MIN, shooterTarget - VELOCITY_STEP);
        lastDpadUp = dUp; lastDpadRight = dRight; lastDpadLeft = dLeft;

        // (shooting is decided above, before the turret, so the turret can stay calm while balls fly)

        double shooterVelocity = shooterOn ? shooterTarget : 0;
        String shooterMode = shooterOn ? "ON" : "OFF";

        double gameRamp = GAME_RAMP_EXTRA * Math.min(1.0, gameTimer.seconds() / GAME_RAMP_SECS);
        double commandedVelocity = shooterOn ? (shooterVelocity + gameRamp) : 0;
        leftShooter.setVelocity(commandedVelocity);

        // ── SHOOT (held) ─────────────────────────────────────────────────
        if (shooting && !lastShooting) kickTimer.reset();
        lastShooting = shooting;
        boolean kickReady = shooting && turretReady && kickTimer.milliseconds() >= KICK_DELAY_MS;

        double intakePower;
        String intakeMode;
        if (shooting) {
            intakePower = INTAKE_POWER;
            intakeMode = "SHOOT-FEED";
            blocker.setPosition(BLOCKER_OPEN);
//            kicker.setPosition(kickReady ? KICKER_KICK : KICKER_BASE);
        } else {
            if (gamepad2.a) {
                intakePower = INTAKE_POWER;
                intakeMode = "IN";
            } else if (gamepad2.b) {
                intakePower = -INTAKE_POWER;
                intakeMode = "OUT";
            } else {
                intakePower = 0;
                intakeMode = "STOPPED";
            }
            blocker.setPosition(BLOCKER_CLOSED);
//            kicker.setPosition(KICKER_BASE);
        }
        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);

        // ── Flower (gamepad 2 Y toggle) ──────────────────────────────────
        boolean y2 = gamepad2.y;
        if (y2 && !lastY2) {
            flowerUp = !flowerUp;
            setFlowerPosition(flowerUp ? FLOWER_UP : FLOWER_DOWN);
        }
        lastY2 = y2;

        // ── Telemetry ────────────────────────────────────────────────────
        double[] g = goal();
        telemetry.addData("Loop time", "%.1f ms", loopMs);
        telemetry.addData("Shooter", "%s target=%.0f commanded=%.0f actual=%.0f",
                shooterMode, shooterVelocity, commandedVelocity, leftShooter.getVelocity());
        telemetry.addData("Shooter set velocity", "%.0f  (F x%.3f)", shooterTarget, shooterFScale);
        telemetry.addData("Game ramp", "+%.1f ticks/sec (%.0f s elapsed)", gameRamp, gameTimer.seconds());
        telemetry.addData("Shooting", shooting ? (gamepad2.right_trigger > TRIGGER_THRESHOLD ? "YES (G2 R2)" : "YES (auto)") : "no");
        if (shooting) {
            telemetry.addData("Kick delay", "%.0f / %d ms %s",
                    Math.min(kickTimer.milliseconds(), KICK_DELAY_MS), KICK_DELAY_MS,
                    kickReady ? "(FIRING)" : (turretReady ? "(waiting)" : "(WAITING FOR TURRET)"));
        }
        telemetry.addData("Intake", intakeMode);
        telemetry.addData("Blocker", shooting ? "OPEN" : "CLOSED");
        telemetry.addData("Kicker", kickReady ? "KICK" : "BASE");
        telemetry.addData("Flower", flowerUp ? "UP" : "DOWN");
        telemetry.addLine("---");
        telemetry.addData("Alliance / preset", "%s #%d  goal=(%.0f, %.0f)", alliance, presetIndex, g[0], g[1]);
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f deg",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Distance to goal", "%.1f in", Math.hypot(g[0] - pose.x(), g[1] - pose.y()));
        telemetry.addData("Goal side", "%.1f in %s of goal  %s zone row=%s  offset=%.1f deg",
                Math.abs(rightIn), rightIn >= 0 ? "RIGHT" : "LEFT", alliance,
                zoneIdx >= 0 ? String.valueOf(zoneIdx + 1) : "none(default)", Math.toDegrees(sideOffset));
        telemetry.addData("TURRET", turretKilled ? (autoDriving ? "KILLED (locked for auto-drive)" : "KILLED (no power)")
                : (aimOn ? "AIM ON" : "AIM OFF (parked)"));
        telemetry.addData("Turret homing", "%s%s", homeStep,
                homeFailed ? "  FAILED - sensor not hit, using INIT zero" : "");
        telemetry.addData("Auto-drive (G1 R2)", "%s busy=%b route=%s", alignState, follower.isBusy(),
                alignState == AlignState.MANUAL ? "-" : (routeCorners > 0 ? "curving (" + routeCorners + " pts)" : "straight"));
        if (autoDriving) {
            telemetry.addData("Shoot target", "x=%.1f y=%.1f h=%.0f deg",
                    alignTarget.x(), alignTarget.y(), Math.toDegrees(alignTarget.heading()));
            telemetry.addData("Aim error", "%.2f deg %s", Math.toDegrees(aimErrorRad()),
                    alignState == AlignState.ARRIVED ? (aimLocked ? "(LOCKED - shooting)" : "(waiting to line up)") : "");
        }
        telemetry.addData("Turret", "now=%.1f target=%.1f err=%.2f deg %s",
                Math.toDegrees(current), Math.toDegrees(target), Math.toDegrees(err),
                homing ? "(HOMING)" : powerCut ? "(NO POWER)" : (!tracking ? "(parked)" : Math.abs(err) <= TOLERANCE_RAD ? "(ON TARGET)"
                        : (Math.abs(err) > Math.toRadians(SWING_ERR_DEG) ? "(far off)" : "")));
        double goalBearing = Math.atan2(g[1] - pose.y(), g[0] - pose.x());
        telemetry.addData("Turret heading", "%.1f deg (field)  goal bearing %.1f deg",
                Math.toDegrees(wrap(pose.heading() + current + TURRET_ZERO_OFFSET_RAD)), Math.toDegrees(goalBearing));
        telemetry.addData("Turret ticks from REAL zero", "%d  (from sensor %d, raw %d, %s)",
                ticksFromHome() - REAL_ZERO_TICKS_FROM_HOME, ticksFromHome(),
                turret.getCurrentPosition(), homed ? "zeroed by touch sensor" : "NOT HOMED - INIT spot is 0");
        telemetry.addData("Goal rate / power / I", "%.0f deg/s  %.2f%s  %.3f",
                Math.toDegrees(spinRate), power, holding ? " (HOLD)" : kickActive ? " (KICK)" : minPowerActive ? " (MIN)" : "", kI * errInt);

        // ── HOMING DEBUG ─────────────────────────────────────────────────
        telemetry.addLine("--- HOMING DEBUG ---");
        telemetry.addData("Home step", "%s  (%.2f s in step)", stepBefore, stepSecs);
        telemetry.addData("Touch sensor NOW", touchNow ? "PRESSED" : "not pressed");
        telemetry.addData("Touch ever pressed", touchEverPressed ? "YES" : "NO");
        telemetry.addData("Zero reset by touch", homed
                ? String.format("YES at %.2f s", zeroSetAtSec) : "NO (INIT spot is still 0)");
        telemetry.addData("Home result", homeEndReason);
        telemetry.addData("Home servo cmd", "%.3f  (UP=%.3f DOWN=%.3f)",
                homeServo.getPosition(), HOME_SERVO_UP, HOME_SERVO_DOWN);
        telemetry.addData("Turret power sent", "%.2f %s", POWER_SIGN * sentPower, homing ? "(homing)" : "");
        telemetry.addData("Battery", "%.2f V  compensation x%.3f (nominal %.1f V)",
                batteryVolts, voltScale, NOMINAL_VOLTAGE);
        telemetry.addData("Raw ticks / at touch", "%d / %d", turret.getCurrentPosition(), homeRawTicks);
        telemetry.addData("Ticks since seek start", "%d", turret.getCurrentPosition() - seekStartTicks);
        telemetry.addData("Offset", "%.1f deg   HOME_SWITCH_DEG=%.1f", Math.toDegrees(turretOffsetRad), HOME_SWITCH_DEG);
        telemetry.addData("Killed (L2)", turretKilled ? "YES" : "no");
        telemetry.update();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        if (turret != null) turret.setPower(0);
        if (homeServo != null) homeServo.setPosition(HOME_SERVO_DOWN);
        leftShooter.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        setFlowerPosition(FLOWER_DOWN);
        blocker.setPosition(BLOCKER_CLOSED);
//        kicker.setPosition(KICKER_BASE);
    }
}
