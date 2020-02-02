package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by kyliestruth 10/27/18.
 */

@Disabled
@TeleOp(name = "Strafing Test Auto", group = "TankDrive")
public class TestingStrafingAuto extends OpMode {
    private HardwareMapStrafingTest robot = new HardwareMapStrafingTest();

    @Override
    public void init() {

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
    }

    public void init_loop() {

    }

    public void loop() {

        robot.rightBack.setPower(1);
        robot.rightFront.setPower(-1);
        robot.leftBack.setPower(-1);
        robot.leftFront.setPower(1);
    }

    public void stop() {

    }
}
