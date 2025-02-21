package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="Drive")
public class Drive extends OpMode {

    int positionOfSlides = 0;




    HardwareSoftware hw = new HardwareSoftware();
    @Override
    public void init() {
        hw.init(hardwareMap);

        hw.gyro().calibrateImu();
        hw.gyro().resetTracking();


        /*
        This code is being refactored according to the sheet for the controller
        (Old is dark green, new is light)
         */
        currentSetPosition = positions[0];

        hw.Rinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.Linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.Rinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.Linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    double DS = .9;
    boolean D = true, D2=true;

    // FixedLinear Vars
    double[] pidValues = {2, 0, 0};
    int[] encoderValues = {3286, 3264};
    double height = 28.5d;
    double[] positions = {0,13,25};
    double currentSetPosition;
    int[] positionsEncoderValues = {0,0};

    boolean wasBumperPressed = false;  // Track previous bumper state

    private static ElapsedTime myStopwatch = new ElapsedTime();
    private ElapsedTime sigmaboy = new ElapsedTime();
    boolean isAButtonPressed = false;
    boolean isBButtonPressed = false;
    boolean isYButtonPressed = false;

    boolean actionPerformedA = false;
    boolean actionPerformedB = false;
    boolean actionPerformedY = false;




    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        SparkFunOTOS.Pose2D pos = hw.gyro().getPosition();

        double botHeading = -Math.toRadians(pos.h) + Math.PI;

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
        telemetry.addData("newX", rotX);
        telemetry.addData("newY", rotY);
        telemetry.addData("rx", rx);
        rotX = rotX * 1.1;  // Counteract imperfect strafing


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        hw.FLdrive().setPower(((rotY + rotX + rx) / denominator) * DS);
        hw.BLdrive().setPower(((rotY - rotX + rx) / denominator) * DS);
        hw.FRdrive().setPower(((rotY - rotX - rx) / denominator) * DS);
        hw.BRdrive().setPower(((rotY + rotX - rx) / denominator) * DS);

        if(gamepad2.right_trigger > .1){
            hw.MrMini().setPosition(1);
        }
        if(gamepad2.left_trigger > .1){
            hw.MrMini().setPosition(0);
        }


        if (gamepad1.right_bumper && !wasBumperPressed) {  // Check if bumper is pressed and was not previously pressed
            if (D) {
                DS = 0.4;
                D = false;
            } else {
                DS = .9;
                D = true;
            }
            wasBumperPressed = true;  // Set flag to indicate bumper was pressed
        } else if (!gamepad1.right_bumper) {
            wasBumperPressed = false;  // Reset flag when bumper is released
        }

        // Gamepad #2
        // Intake Linear Slide

        if (gamepad1.left_trigger > .1) {
            hw.InLinear().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hw.InLinear().setPower(gamepad1.left_trigger*.8);
        } else if (gamepad1.right_trigger > .1){
            hw.InLinear().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hw.InLinear().setPower(-gamepad1.right_trigger * .6);
        } else if (hw.InLinear().getMode() == DcMotorEx.RunMode.RUN_USING_ENCODER) {
            hw.InLinear().setPower(0);
        }

        if (gamepad1.dpad_left) {  //specimen
            hw.Lucket.setPosition(.82);
            hw.Rucket.setPosition(.90);

        }
        if (gamepad1.dpad_up) {  //specimen
            hw.Linear.setTargetPosition(1900);
            hw.Rinear.setTargetPosition(1900);

            hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            hw.Linear.setPower(1);
            hw.Rinear.setPower(1);

        }
        if (gamepad1.dpad_down) {  //specimen
            hw.Linear().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hw.Rinear().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hw.Linear().setPower(-.8);
            hw.Rinear().setPower(-.8);
            hw.MrMini().setPosition(0);

        } else if (hw.Linear().getMode() == DcMotorEx.RunMode.RUN_USING_ENCODER) {
            hw.Linear().setPower(0);
            hw.Rinear().setPower(0);
        }


        if (gamepad2.left_bumper) {
            hw.Intake.setPower(-.8);
        } else if (gamepad2.right_bumper) {
            hw.Intake.setPower(.8);
        } else {
            hw.Intake.setPower(0);
        }


        if (gamepad2.dpad_down && !(hw.Linear.getCurrentPosition() > 500) && !(hw.Rinear.getCurrentPosition() > 500)) {  //bottom
            hw.Lucket.setPosition(.04);  //originally 1
            hw.Rucket.setPosition(.11);  //originally 1

        } else if (gamepad2.dpad_up) {  //top
            hw.Lucket.setPosition(.64);
            hw.Rucket.setPosition(.72);
        }

        if (gamepad2.x) {
            hw.Linear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            hw.Rinear.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            hw.Linear.setPower(-.7);
            hw.Rinear.setPower(-.7);
        }
        if(gamepad2.x && hw.Linear.getMode() == DcMotorEx.RunMode.RUN_WITHOUT_ENCODER) {
            hw.Linear.setPower(0);
            hw.Rinear.setPower(0);
        }

        if (gamepad2.share) {
            hw.Linear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            hw.Rinear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (gamepad1.share) {
            hw.InLinear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (gamepad1.a) {
            hw.InLinear.setTargetPosition(-175);


            hw.InLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            hw.InLinear.setPower(-1);
        }


// For button Y press

        if (gamepad2.a) {
            hw.Lucket.setPosition(0.16);
            hw.Rucket.setPosition(0.25);

            hw.Linear.setTargetPosition(40);
            hw.Rinear.setTargetPosition(40);
            hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            hw.Linear.setPower(1);
            hw.Rinear.setPower(1);
        }

        if (gamepad2.b) {
            if (!isBButtonPressed) {
                // Move Lucket and Rucket immediately when the button is pressed
                hw.Lucket.setPosition(.71);
                hw.Rucket.setPosition(.79);

                // Start the stopwatch to track the 0.5 seconds delay
                myStopwatch.reset();
                myStopwatch.startTime();

                // Mark the button as pressed to prevent repeated action
                isBButtonPressed = true;
                actionPerformedB = false;  // Reset action performed flag
            }

            // Perform the slide action after 0.5 seconds
//            if (myStopwatch.seconds() > 0.5 && !actionPerformedB) {
                // Move the slide motors to their target positions
                hw.Linear.setTargetPosition(1250);
                hw.Rinear.setTargetPosition(1250);

                hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                hw.Linear.setPower(1);
                hw.Rinear.setPower(1);

                // Mark that the action has been performed, preventing repeat execution
                actionPerformedB = true;
//            }
        } else {
            // Reset the flag when the button is released
            isBButtonPressed = false;
        }

        if (gamepad2.y) {
            if (!isYButtonPressed) {
                // Move Lucket and Rucket immediately when the button is pressed
                hw.Lucket.setPosition(.71);
                hw.Rucket.setPosition(.79);

                // Start the stopwatch to track the 0.5 seconds delay
                myStopwatch.reset();
                myStopwatch.startTime();

                // Mark the button as pressed to prevent repeated action
                isYButtonPressed = true;
                actionPerformedY = false;  // Reset action performed flag
            }

            // Perform the slide action after 0.5 seconds
            if (!actionPerformedY) {
                //myStopwatch.seconds() > 0.5 &&
                // Move the slide motors to their target positions
                hw.Linear.setTargetPosition(3075);
                hw.Rinear.setTargetPosition(3075);

                hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                hw.Linear.setPower(1);
                hw.Rinear.setPower(1);

                // Mark that the action has been performed, preventing repeat execution
                actionPerformedY = true;
            }
        } else {
            // Reset the flag when the button is released
            isYButtonPressed = false;
        }

// Reset all flags and action flags when no button is pressed (this part is optional depending on the behavior you want)
        if (!gamepad2.b && !gamepad2.y) {
            // Reset flags for button presses
            isBButtonPressed = false;
            isYButtonPressed = false;

            // Reset action flags so that actions can be triggered again on the next button press
            actionPerformedB = false;
            actionPerformedY = false;
        }


//             else if (gamepad2.b) {
//                hw.Lucket.setPosition(.63);  //originally 0
//                hw.Rucket.setPosition(.71);  //originally 0
//
//                hw.Linear.setTargetPosition(1250);
//                hw.Rinear.setTargetPosition(1250);
//
//                hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                hw.Linear.setPower(1);
//                hw.Rinear.setPower(1);
//            } else if (gamepad2.y) {
//                hw.Lucket.setPosition(.63);  //originally 0
//                hw.Rucket.setPosition(.71);  //originally 0
//
//                hw.Linear.setTargetPosition(3150);
//                hw.Rinear.setTargetPosition(3150);
//
//                hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//                hw.Linear.setPower(1);
//                hw.Rinear.setPower(1);
//            }
//        else if (gamepad1.left_bumper) {
//            hw.Linear.setTargetPosition(1000);
//            hw.Rinear.setTargetPosition(1000);
//
//            hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            hw.Rinear.setPower(1);
//            hw.Linear.setPower(1);
//        }

        if (gamepad1.y && gamepad1.b) {
            hw.gyro().resetTracking();
        }


            telemetry.addData("bot heading", botHeading);
            telemetry.addData("Lucket", hw.Lucket.getPosition());
            telemetry.addData("Rucket", hw.Rucket.getPosition());
            telemetry.addData("Timer", myStopwatch.seconds());
            telemetry.update();
        }


    }




