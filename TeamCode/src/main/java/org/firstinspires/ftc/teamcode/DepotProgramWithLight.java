package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @author Beep Patrol
 * <p>
 * <b>Summary:</b>
 * <p>
 * This is our autonomous program for the depot side on both sides of the field. This program runs
 * without the phone light for Tensor Flow. This is the go to program. This programs lands, hits off
 * the gold mineral, deposits the team marker, and parks in the other alligances crater.
 */
@Autonomous(name = "Depot Program With Light", group = "Beep")
public class DepotProgramWithLight extends LinearOpMode {

    // Declaring a timer
    public ElapsedTime runtime = new ElapsedTime();
    public String foundTargetName = "";
    //Calling our hardware map
    HardwareBeep robot = new HardwareBeep();
    // Calling the Library Gyro program to use the gyro turn function
    LibraryGyro gyroTurn = new LibraryGyro();
    // Calling the Library Gyro Drive program to use the gyro drive function
    LibraryGyroDrive gyroDrive = new LibraryGyroDrive();
    // Calling the Library Grid Nav Library to use the grid navigation functions
    LibraryGridNavigation gridNavigation = new LibraryGridNavigation();
    // Calling the Library Tensor Flow No Light to use the Tensor Flow function without
    // initializing the light
    LibraryTensorFlowObjectDetectionWithLight tensorFlow =
            new LibraryTensorFlowObjectDetectionWithLight(robot, telemetry);
    // Declaring gold position value to read what position Tensor Flow sees the gold mineral in
    String goldPosition = "";

    /**
     * This method is the main body of our code which contains the set of commands carried out in our crater side autonomous program.
     */
    @Override
    public void runOpMode() {


        telemetry.addData("Telemetry", "robot initializing");
        telemetry.update();
        //initializing the hardware map
        robot.init(hardwareMap);
        //initializing the grid Nav function
        gridNavigation.init(robot, gyroTurn, telemetry);
        //initializing the gyro turn function
        gyroTurn.init(robot, telemetry);
        //initializing the gyro drive function
        gyroDrive.init(robot, telemetry, robot.rightBack);
        telemetry.addData("Telemetry", "run opMode start");
        telemetry.update();

        // setting initial latch power to 0
        robot.latch.setPower(0);

        //wait for start
        waitForStart();

        // landing our robot
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(-12500);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);

        // Start up Tensor Flow to read mineral position while landing
        getMineralPosition();

        // stop running lift once it gets to target position
        robot.lift.setPower(0);
        runtime.reset();
        // run servo to open latch
        robot.latch.setPower(-1);

        //wait for 1.15 seconds
        while (runtime.seconds() < 1.15) {

        }
        // set servo power to 0
        robot.latch.setPower(0);
        robot.lift.setPower(0);
        runtime.reset();

        // took the hypotenuse line to the center of the robot is 20.625 inches
        // Set initial Grid Nav position
        gridNavigation.setGridPosition(.6076, .6076, 45);

        int X = 0;
        int Y = 1;
        double ARMTIMEOUT = .8;

        // Left mineral pos
        double[] RED_DEPOT_LEFT = {1.2, 2.4};
        // right mineral pos
        double[] RED_DEPOT_RIGHT = {1.2916, .9114};
        // center mineral pos
        double[] RED_DEPOT_CENTER = {1.1, 1.1};

        // right marker pos
        double[] RIGHT_DEPOT_MARKER = {1.6, 2.7};
        // center marker pos
        double[] CENTER_DEPOT_MARKER = {1.6, 2.6};
        // left marker pos
        double[] LEFT_DEPOT_MARKER = {1.6, 2.4};

        // Parking pos for all mineral positions
        double[] LEFT_DEPOT_PARKING = {-.7, 2.65};
        double[] RIGHT_DEPOT_PARKING = {-.7, 2.45};
        double[] CENTER_DEPOT_PARKING = {-.6, 2.65};

        // This is a switch block that plays the program in relation to the mineral position that
        // Tensor Flow reads
        switch (goldPosition) {

            // If Tensor Flow reads the left mineral position then it plays this case
            case "LEFT":
                telemetry.addData("Telemetry", "Gold Pos = LEFT");
                printTelemetry(20);
                if (goldPosition == "LEFT") {
                    gridNavigation.driveToPosition(.8, .8, .5);
                    lowerLift();
                    gridNavigation.driveToPosition(RED_DEPOT_LEFT[X], RED_DEPOT_LEFT[Y], .5);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_LEFT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_LEFT[Y]);
                    gridNavigation.driveToPosition(LEFT_DEPOT_MARKER[X], LEFT_DEPOT_MARKER[Y], .5);
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-420);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.arm.setPower(.111);
                    robot.intake.setPower(1);
                    sleep(1000);
                    robot.intake.setPower(0);
                    runtime.reset();
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(420);//-644 is to come from mat to stopping point
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    runtime.reset();
                    while (robot.arm.isBusy() || runtime.seconds() < ARMTIMEOUT) {
                        telemetry.addData("Seconds Elapsed", runtime.seconds());
                        telemetry.update();
                    }
                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.arm.setPower(.111);
                    // drive to crater parking position
                    gridNavigation.driveToPosition(LEFT_DEPOT_PARKING[X], LEFT_DEPOT_PARKING[Y], .7);
                    // bring arm down to park
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-686);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.arm.setPower(-.15);

                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(30);
                }

                break;

            // If Tensor Flow reads the right mineral position then it plays this case
            case "RIGHT":
                telemetry.addData("Telemetry", "Gold Pos = RIGHT");
                printTelemetry(40);
                if (goldPosition == "RIGHT") {
                    // drive away from lander
                    gridNavigation.driveToPosition(.8, .8, .5);
                    // start lower lift
                    lowerLift();
                    // drop arm
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-643);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                    // drive to right mineral pos
                    gridNavigation.driveToPosition(RED_DEPOT_RIGHT[X], RED_DEPOT_RIGHT[Y], .5);
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_RIGHT[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_RIGHT[Y]);
                    // lift arm
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(660);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    // set power to .111 to keep arm up
                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.arm.setPower(.111);
                    // drive around minerals to get to depot
                    gridNavigation.driveToPosition(.6, 1.5, .5);
                    gridNavigation.driveToPosition(.5, 2.5, .5);
                    // drive towards depot to deposit marker
                    gridNavigation.driveToPosition(RIGHT_DEPOT_MARKER[X], RIGHT_DEPOT_MARKER[Y], .5);
                    // bring arm down
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-420);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    // hold arm position
                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.arm.setPower(.111);
                    // run intake to deposit marker
                    robot.intake.setPower(1);
                    sleep(1000);
                    robot.intake.setPower(0);
                    runtime.reset();
                    // bring arm up
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(420);//-644 is to come from mat to stopping point
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    runtime.reset();
                    while (robot.arm.isBusy() || runtime.seconds() < ARMTIMEOUT) {
                        telemetry.addData("Seconds Elapsed", runtime.seconds());
                        telemetry.update();
                    }
                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.arm.setPower(.111);
                    // drive to crater parking position
                    gridNavigation.driveToPosition(RIGHT_DEPOT_PARKING[X], RIGHT_DEPOT_PARKING[Y], .7);
                    // bring arm down to park
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-686);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.arm.setPower(-.15);
                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(50);
                }

                break;

            // If Tensor Flow reads the center mineral position then it plays this case
            case "CENTER":
                telemetry.addData("Telemetry", "Gold Pos = CENTER");
                printTelemetry(60);
                if (goldPosition == "CENTER") {
                    telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_CENTER[X]);
                    telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_CENTER[Y]);
                    // bring arm down
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-643);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setPower(0);
                    // drive to center mineral position
                    gridNavigation.driveToPosition(RED_DEPOT_CENTER[X], RED_DEPOT_CENTER[Y], .5);
                    // start lowering the lift
                    lowerLift();
                    // bring arm up
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(660);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.arm.setPower(.111);
                    // drive around right mineral to get to depot
                    gridNavigation.driveToPosition(.65, 1.5, .5);
                    gridNavigation.driveToPosition(.2, 2.5, .5);
                    // drive to depot to deposit marker
                    gridNavigation.driveToPosition(CENTER_DEPOT_MARKER[X], CENTER_DEPOT_MARKER[Y], .5);
                    // bring arm down
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-420);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    // hold arm position
                    robot.arm.setPower(.111);
                    // run intake to deposit intake
                    robot.intake.setPower(1);
                    sleep(1000);
                    robot.intake.setPower(0);
                    runtime.reset();
                    // bring arm up
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(420);//-644 is to come from mat to stopping point
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    runtime.reset();
                    while (robot.arm.isBusy() || runtime.seconds() < ARMTIMEOUT) {
                        telemetry.addData("Seconds Elapsed", runtime.seconds());
                        telemetry.update();
                    }
                    robot.arm.setPower(.111);
                    // drive to crater parking position
                    gridNavigation.driveToPosition(CENTER_DEPOT_PARKING[X], CENTER_DEPOT_PARKING[Y], .7);
                    // bring arm down to park
                    robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.arm.setTargetPosition(-686);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.arm.setPower(1);
                    while (robot.arm.isBusy()) {
                    }
                    robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    robot.arm.setPower(-.15);
                } else {
                    telemetry.addData("Telemetry", "No Position Found");
                    printTelemetry(70);
                }

                telemetry.addData("Grid Nav Goto Pos X", RED_DEPOT_LEFT[X]);
                telemetry.addData("Grid Nav Goto Pos Y", RED_DEPOT_LEFT[Y]);

                telemetry.update();
                break;
            // should never get to this case but in case it can't find the mineral position
            // it goes to this default case
            default:
                telemetry.addData("Telemetry", "Didn't see gold pos");
                telemetry.update();
                break;
        }

        // Once it goes through the case block it does the following
        telemetry.addData("Parked Ready to pull out arm", "");
        telemetry.update();

        // We stop using Grid Nav at this point and drive backwards to pull out our arm
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setTargetPosition(-1100);
        robot.leftBack.setTargetPosition(-1100);
        robot.rightFront.setTargetPosition(-1100);
        robot.rightBack.setTargetPosition(-1100);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setPower(.3);
        robot.leftFront.setPower(.3);
        robot.rightBack.setPower(.3);
        robot.rightFront.setPower(.3);

        // Wait until wheels encoders have gone to -537
        while (robot.rightFront.isBusy()) {

        }
        // Shut off motors
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);

        // bring arm up
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setTargetPosition(385);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(1);
        while (robot.arm.isBusy()) {
        }
        robot.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.arm.setPower(.15);

        // start intake
        robot.intake.setPower(1);

        // driving forward
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setTargetPosition(600);
        robot.leftBack.setTargetPosition(600);
        robot.rightFront.setTargetPosition(600);
        robot.rightBack.setTargetPosition(600);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setPower(.5);
        robot.leftFront.setPower(.5);
        robot.rightBack.setPower(.5);
        robot.rightFront.setPower(.5);

        while (robot.rightFront.isBusy()) {

        }
        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightBack.setPower(0);
        robot.rightFront.setPower(0);
    }

    /**
     * This method prints telemetry for our autonomous program
     *
     * @param codePos This is the value we use in telemetry to see where in the code we are
     */
    private void printTelemetry(int codePos) {
        telemetry.addData("Gold Pos", goldPosition);
        telemetry.addData("Code Position", codePos);
        telemetry.update();
    }

    /**
     * This method lowers the lift while reading the sampling
     */
    private void lowerLift() {
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setTargetPosition(11760);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(1);

    }

    /**
     * This method calls Tensor Flow in order to read gold mineral position
     */
    public void getMineralPosition() {
        int debounceCount = 0;
        long startTime = 0;
        String previousPosition;
        goldPosition = tensorFlow.findMineral();

        // Switch block that indicated which mineral position it reads
        switch (goldPosition) {
            case ("LEFT"):
                telemetry.addData("Telemetry", "Left Position");
                telemetry.update();
                break;
            case ("RIGHT"):
                telemetry.addData("Telemetry", "Right Position");
                telemetry.update();
                break;
            case ("CENTER"):
                telemetry.addData("Telemetry", "Center Position");
                telemetry.update();
                break;

            // If it reads unknown than it goes to this default case
            default:
                telemetry.addData("Telemetry", "Unknown Position");
                telemetry.update();
                // sets mineral pos to center as default
                goldPosition = "CENTER";
                break;
        }

        telemetry.update();
    }
}