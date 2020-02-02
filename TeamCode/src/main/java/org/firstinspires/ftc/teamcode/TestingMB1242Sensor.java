package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
@Autonomous(name = "Testing Ultrasonic Sensor", group = "Exercises")
public class TestingMB1242Sensor extends LinearOpMode {

    public HardwareBeep robot = new HardwareBeep();

//    Parameters

//    address: the 7-bit I2C device address of the device to transmit to and from.
//    quantity: the number of bytes to request.
//    value: a value to send as a single byte.

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();

        robot.init(hardwareMap);


        waitForStart();

    }
}