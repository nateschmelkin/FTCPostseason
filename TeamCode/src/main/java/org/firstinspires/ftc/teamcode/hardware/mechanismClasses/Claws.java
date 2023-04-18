package org.firstinspires.ftc.teamcode.hardware.mechanismClasses;

import com.aimrobotics.aimlib.subsystems.ContinuousServoIntake;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.GamepadConstants;

public class Claws extends Mechanism {

    ContinuousServoIntake frontIntake = new ContinuousServoIntake();
    ContinuousServoIntake backIntake = new ContinuousServoIntake();

    @Override
    public void init(HardwareMap hwMap) {
        frontIntake.initialize(hwMap, "frontLeftClaw", "frontRightClaw");
        backIntake.initialize(hwMap, "backLeftClaw", "backRightClaw");

        frontIntake.setDirections(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD);
        backIntake.setDirections(DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop(Gamepad gamepad) {
        if (gamepad.a || gamepad.right_trigger > GamepadConstants.GAMEPAD2_TRIGGER_DEADZONE) {
            frontIntake.intake(1);
            backIntake.intake(1);
        } else if (gamepad.b || gamepad.left_trigger > GamepadConstants.GAMEPAD2_TRIGGER_DEADZONE) {
            frontIntake.outtake(1);
            backIntake.outtake(1);
        } else {
            frontIntake.stop();
            backIntake.stop();
        }
    }
}
