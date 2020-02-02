package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@Autonomous(name = "Michael Gryo Testing 1", group = "Exercises")
public class MichaelGyroTurnTest extends LinearOpMode {

    public HardwareBeep robot = new HardwareBeep();
    public LibraryGyro gyro = new LibraryGyro();
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    double angle_variable;
    boolean aButton, bButton, touched;
    LibraryGridNavigation gridNav = new LibraryGridNavigation();

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();

        robot.init(hardwareMap);
        gyro.init(robot, telemetry);
        gridNav.init(robot, gyro, telemetry);


        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();

        // get a reference to REV Touch sensor.
//        touch = hardwareMap.digitalChannel.get("touch_sensor");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        robot.imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyroTurn is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.

        waitForStart();

        /*
        try{
            gridNav = new LibraryGridNavigation();
        } catch (Exception e) {
            telemetry.addData("GridNavException: ", e.getStackTrace());
            telemetry.update();
        }
        */

        telemetry.addData("Telemtry", "gridNav init complete");
        telemetry.update();


        gridNav.setGridPosition(0, 0, 0);
        telemetry.addData("Telemtry", "gridNav SetPosition complete");
        telemetry.update();

        sleep(3000);


        gridNav.driveToPosition(0, 2, power);
        telemetry.addData("Telemtry", "gridNav GridNavImplemented complete");
        telemetry.update();

        sleep(3000);

        gridNav.driveToPosition(2, 2, power);
        sleep(3000);

        gridNav.driveToPosition(2, 0, power);
        sleep(3000);

        gridNav.driveToPosition(0, 0, power);
        sleep(3000);


        while (opModeIsActive()) {

            getAngle();
            angle_variable = getAngle();
            telemetry.addData("Mode", "running");
            telemetry.addData("Current Angle", getAngle());
            telemetry.addData("Current Angle", angle_variable);
            telemetry.update();

            // drive until end of period.
        }
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void GyroTurn(int degrees) {

        resetAngle();

        double realign;

        if (degrees < 0) {
            //rotates left
            while (opModeIsActive() && getAngle() > degrees) {

                double Proportional = (degrees - getAngle()) / degrees;
                double speed = Math.round(Proportional);

                telemetry.addData("Telemetry 1", "Active");
                robot.leftFront.setPower(-speed);
                robot.leftBack.setPower(-speed);
                robot.rightFront.setPower(speed);
                robot.rightBack.setPower(speed);
                // set leftPower to -speed;
                // set rightPower to speed;
                telemetry.addData("Telemetry 2", "Active");


            }

            telemetry.addData("Telemetry 3", "Active");

            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);
            // set leftPower to 0;
            // set rightPower to 0;

            realign = degrees - getAngle();

            while (opModeIsActive() && realign > 0) {

                robot.leftFront.setPower(.1);
                robot.leftBack.setPower(.1);
                robot.rightFront.setPower(-.1);
                robot.rightBack.setPower(-.1);
                // set leftPower to .1
                // set rightPower to -.1
            }
        } else if (degrees > 0) {
            //rotates right

            while (opModeIsActive() && getAngle() < degrees) {

                double Proportional = (degrees - getAngle()) / degrees;
                double speed = Math.round(Proportional);

                telemetry.addData("Telemetry 4", "Active");

                robot.leftFront.setPower(speed);
                robot.leftBack.setPower(speed);
                robot.rightFront.setPower(speed);
                robot.rightBack.setPower(speed);
                // set leftPower to speed;
                // set rightPower to -speed;
                telemetry.addData("Telemetry 5", "Active");


            }

            telemetry.addData("Telemetry 6", "Active");

            robot.leftFront.setPower(0);
            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);
            // set leftPower to 0;
            // set rightPower to 0;

            realign = degrees - getAngle();

            while (opModeIsActive() && realign > 0) {

                robot.leftFront.setPower(-.1);
                robot.leftBack.setPower(-.1);
                robot.rightFront.setPower(.1);
                robot.rightBack.setPower(.1);
                // set leftPower to -.1
                // set rightPower to .1

            }
        } else return;


        resetAngle();


    }
}