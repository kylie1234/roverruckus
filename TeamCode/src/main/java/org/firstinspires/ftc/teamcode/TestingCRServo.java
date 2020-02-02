package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by kyliestruth 10/27/18.
 */
@Disabled
@TeleOp(name = "TestingCRServo", group = "TankDrive")
public class TestingCRServo extends OpMode {
    private HardwareMapTestingCRServo robot = new HardwareMapTestingCRServo();

    @Override
    public void init() {

        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");
    }

    public void init_loop() {

        robot.basket.setPosition(.46);
    }

    public void loop() {

        if (gamepad2.dpad_down && !gamepad2.dpad_up) {
            robot.basket.setPosition(1);
        } else if (gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.basket.setPosition(-1);
        } else if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
            robot.basket.setPosition(0.46);
        }
    }

    public void stop() {

        robot.basket.setPosition(.46);
    }
}
