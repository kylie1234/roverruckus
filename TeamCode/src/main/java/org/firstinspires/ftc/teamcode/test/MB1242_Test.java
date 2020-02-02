package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareBeep;
import org.firstinspires.ftc.teamcode.sensors.SensorMB1242;

@Disabled
@TeleOp(name = "Rear Ultrasonic Testing", group = "Test")
//@Disabled
public class MB1242_Test extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    HardwareBeep robot = new HardwareBeep();
    //    SensorMB1242 rearUS
    boolean last = false;
    boolean readLeftSensor = false;
    int i = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

//        rearUS = hardwareMap.get(SensorMB1242.class,"rear");
        SensorMB1242 leftSonic = robot.leftSonic;
        SensorMB1242 rightSonic = robot.rightSonic;

        leftSonic.startAutoPing(40);
        rightSonic.startAutoPing(40);
        sleep(2000);
        telemetry.addData("Start AutoPing", "");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("waitForStart()", i++);
        telemetry.update();
        //rearUS.run();
//        telemetry.addData("rearUS.run", "");
//        telemetry.update();
//        sleep(2000);
        telemetry.addData("Should have read sensor", "");
        telemetry.update();
        sleep(2000);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && !isStopRequested()) {

            if (!readLeftSensor && runtime.milliseconds() > 100) {
                telemetry.addData("Left Distance", leftSonic.getDistance());
                telemetry.addData("Incrementor", i++);
                telemetry.update();
                leftSonic.ping();
                readLeftSensor = true;

            }

            if (runtime.milliseconds() > 200) {

                telemetry.addData("Right Distance", rightSonic.getDistance());
                telemetry.addData("Incrementor", i++);
                telemetry.update();
                rightSonic.ping();
                runtime.reset();
                readLeftSensor = false;
            }


            /**
             * Create a new Elapsed Timer obj
             * reset it
             *
             * inside loop
             * check if timer is > 100ms
             *   write8()
             *   distance = getDistance()

             */
        }

        leftSonic.close();
        rightSonic.close();
    }
}