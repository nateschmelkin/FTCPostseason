package org.firstinspires.ftc.teamcode.hardware.mechanismClasses;

import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.stuyfission.fissionlib.motion.MotionProfiledDcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.DcMotorMotionProfiled;

public class Lifts extends Mechanism {

    DcMotorMotionProfiled frontLift = new DcMotorMotionProfiled();
    DcMotorMotionProfiled backLift = new DcMotorMotionProfiled();

    private static final double OUTPUT_RADIUS = 0.7519685;
    private static final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_REV = 537.6;

    public static double MAX_VEL = 100;
    public static double MAX_ACCEL = 100;
    public static double RETRACTION_MULTIPLIER = 0.75;

    public static double kP = 0.25;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    // TODO CHANGE THESE FOR NEW MOTION
    public static int LOW_POS = 12;
    public static int MEDIUM_POS = 24;
    public static int HIGH_POS = 32;
    public static int PICKUP_POS = 0;

    @Override
    public void init(HardwareMap hwMap) {
        frontLift.initialize(hwMap, "frontLift");
        frontLift.setOutputConstants(OUTPUT_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        frontLift.setRetractionMultiplier(RETRACTION_MULTIPLIER);
        frontLift.setPIDCoefficients(kP, kI, kD, kF);

        // TODO set directions
        frontLift.setDirection(DcMotorSimple.Direction.FORWARD);

        backLift.initialize(hwMap, "backLift");
        backLift.setOutputConstants(OUTPUT_RADIUS, GEAR_RATIO, TICKS_PER_REV);
        backLift.setRetractionMultiplier(RETRACTION_MULTIPLIER);
        backLift.setPIDCoefficients(kP, kI, kD, kF);

        // TODO set directions
        backLift.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setMotionConstraints(double vel, double accel) {
        frontLift.setMotionConstraints(vel, accel);
        backLift.setMotionConstraints(vel, accel);
    }

    public void extendToLow(DcMotorMotionProfiled lift) {
        lift.setTargetPosition(LOW_POS);
    }

    public void extendToMedium(DcMotorMotionProfiled lift) {
        lift.setTargetPosition(MEDIUM_POS);
    }

    public void extendToHigh(DcMotorMotionProfiled lift) {
        lift.setTargetPosition(HIGH_POS);
    }

    public void extendToPickup(DcMotorMotionProfiled lift) {
        lift.setTargetPosition(PICKUP_POS);
    }

    public void extendToPosition(DcMotorMotionProfiled lift, int position) {
        lift.setTargetPosition(position);
    }

    public void dropPos(DcMotorMotionProfiled lift) {
        lift.setTargetPosition(lift.getPosition() - 1); //TODO DETERMINE AMT
    }

    public void ascendPos(DcMotorMotionProfiled lift) {
        lift.setTargetPosition(lift.getPosition() + 1); //TODO DETERMINE AMT
    }

    public void setPower(DcMotorMotionProfiled lift, double power) {
        lift.setPower(power);
    }

    public void updatePositions() {
        frontLift.update();
        backLift.update();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Front Lift Position: ", frontLift.getPosition());
        telemetry.addData("Back Lift Position: ",backLift.getPosition());
    }
}
