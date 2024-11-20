package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="Drive")
public class Drive extends OpMode {




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

    double DS = 1;
    boolean D = true;

    // FixedLinear Vars
    double[] pidValues = {2, 0, 0};
    int[] encoderValues = {3286, 3264};
    double height = 28.5d;
    double[] positions = {0,13,25};
    double currentSetPosition;
    int[] positionsEncoderValues = {0,0};

    boolean wasBumperPressed = false;  // Track previous bumper state


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

        rotX = rotX * 1.1;  // Counteract imperfect strafing


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        hw.FLdrive().setPower(((rotY + rotX + rx) / denominator) * DS);
        hw.BLdrive().setPower(((rotY - rotX + rx) / denominator) * DS);
        hw.FRdrive().setPower(((rotY + rotX - rx) / denominator) * DS);
        hw.BRdrive().setPower(((rotY - rotX - rx) / denominator) * DS);


        if (gamepad1.right_bumper && !wasBumperPressed) {  // Check if bumper is pressed and was not previously pressed
            if (D) {
                DS = 0.2;
                D = false;
            } else {
                DS = 1;
                D = true;
            }
            wasBumperPressed = true;  // Set flag to indicate bumper was pressed
        } else if (!gamepad1.right_bumper) {
            wasBumperPressed = false;  // Reset flag when bumper is released
        }

        // Gamepad #2
        // Intake Linear Slide
        if(gamepad1.left_trigger > .1) {
            hw.InLinear.setPower(gamepad1.left_trigger);
        }
        else if (gamepad1.right_trigger > .1) {
            hw.InLinear.setPower(-gamepad1.right_trigger);
        } else{
            hw.InLinear.setPower(0);
        }





        if (gamepad2.left_bumper) {
            hw.Intake.setPower(-.8);
        } else if (gamepad2.right_bumper) {
            hw.Intake.setPower(.8);
        } else {
            hw.Intake.setPower(0);
        }



        if (gamepad2.dpad_down && !(hw.Linear.getCurrentPosition() > 500) && !(hw.Rinear.getCurrentPosition() > 500)) {  //bottom
            hw.Lucket.setPosition(.72);   //originally .72
            hw.Rucket.setPosition(.92); //originally .92

        }
        else if (gamepad2.dpad_up && !(hw.Linear.getCurrentPosition() > 500) && !(hw.Rinear.getCurrentPosition() > 500)) {  //top
            hw.Lucket.setPosition(.15);//originally .1
            hw.Rucket.setPosition(.35);  //originally .3
        }

        //DO NOT CHANGE THIS WITHOUT A VERY GOOD REASON AND TALKING TO ME OR MRS. FIELD
        //stop trying to use pid's you're making it confusing for no reason and comment my code out dont delete it dude
        if (gamepad2.a) {
            hw.Linear.setTargetPosition(25);
            hw.Rinear.setTargetPosition(25);

            hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            hw.Linear.setPower(1);
            hw.Rinear.setPower(1);
        } else if (gamepad2.b) {
            hw.Linear.setTargetPosition(1250);
            hw.Rinear.setTargetPosition(1250);

            hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            hw.Linear.setPower(1);
            hw.Rinear.setPower(1);
        } else if (gamepad2.y) {
            hw.Linear.setTargetPosition(3200);
            hw.Rinear.setTargetPosition(3200);

            hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            hw.Linear.setPower(1);
            hw.Rinear.setPower(1);
        }


//        if (gamepad2.a) {
//            hw.Linear.setTargetPosition((int) (positions[0] * height / encoderValues[0]));
//            hw.Rinear.setTargetPosition((int) (positions[0] * height / encoderValues[1]));
//
//            hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            hw.Linear.setPower(1);
//            hw.Rinear.setPower(1);
//        } else if (gamepad2.b) {
//            hw.Linear.setTargetPosition((int) (positions[1] * height / encoderValues[0]));
//            hw.Rinear.setTargetPosition((int) (positions[1] * height / encoderValues[1]));
//
//            hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            hw.Linear.setPower(1);
//            hw.Rinear.setPower(1);
//        } else if (gamepad2.y) {
//            hw.Linear.setTargetPosition((int) (positions[2] * height / encoderValues[0]));
//            hw.Rinear.setTargetPosition((int) (positions[2] * height / encoderValues[1]));
//
//            hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            hw.Linear.setPower(1);
//            hw.Rinear.setPower(1);
//        }



        telemetry.addData("bot heading", botHeading);
        telemetry.addData("Lucket", hw.Lucket.getPosition());
        telemetry.addData("Rucket", hw.Rucket.getPosition());
        telemetry.update();
    }


}




