package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.mechanismClasses.ScoringSystem;


@TeleOp(name="LiftClawMechanismTest", group="Samples")
public class LiftClawMechanismTest extends OpMode {

    private ScoringSystem scoringSystem;

    @Override
    public void init() {
        scoringSystem = new ScoringSystem();
        scoringSystem.init(hardwareMap);
    }

    @Override
    public void loop() {
        scoringSystem.loop(gamepad1);

        scoringSystem.telemetry(telemetry);
//        telemetry.update();
    }
}
