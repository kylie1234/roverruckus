package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by kyliestruth 10/27/18.
 */
@Disabled
@TeleOp(name = "Arm Test", group = "TankDrive")
public class TestingArm extends OpMode {
    private HardwareBeep robot = new HardwareBeep();

    @Override
    public void init() {

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
    }

    public void init_loop() {

        robot.arm.setPower(0);
    }

    public void loop() {

        if (gamepad2.left_stick_y > 0) {
            robot.arm.setPower(1);
        } else if (gamepad2.left_stick_y < 0)
            robot.arm.setPower(-1);
        else if (gamepad2.left_stick_y == 0)
            robot.arm.setPower(0);
    }

    public void stop() {

        robot.arm.setPower(0);
    }
}
// Quinn was here...
