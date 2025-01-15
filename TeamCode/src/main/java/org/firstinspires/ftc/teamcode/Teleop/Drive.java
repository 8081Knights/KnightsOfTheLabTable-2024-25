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


        if (gamepad1.right_bumper && !wasBumperPressed) {  // Check if bumper is pressed and was not previously pressed
            if (D) {
                DS = 0.2;
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
            hw.InLinear.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > .1) {
            hw.InLinear.setPower(-gamepad1.right_trigger * .6);
        } else {
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
            hw.Lucket.setPosition(.08);  //originally 1
            hw.Rucket.setPosition(.17);  //originally 1

        } else if (gamepad2.dpad_up && !(hw.Linear.getCurrentPosition() > 500) && !(hw.Rinear.getCurrentPosition() > 500)) {  //top
            hw.Lucket.setPosition(.73);  //originally 0
            hw.Rucket.setPosition(.81);  //originally 0
        }

        if (gamepad2.x){
            hw.Linear().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hw.Rinear().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hw.Linear().setPower(-.5);
            hw.Rinear().setPower(-.5);
        } else if (hw.Linear().getMode() == DcMotorEx.RunMode.RUN_USING_ENCODER){
            hw.Linear().setPower(0);
            hw.Rinear().setPower(0);
        }

        if(gamepad2.start){
            hw.Linear().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            hw.Rinear().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }



// For button Y press

        if (gamepad2.a) {
//            hw.Rucket.setPosition(.18);
//            hw.Lucket.setPosition(.27);

            positionOfSlides = 25;


        }
        if (gamepad2.b) {

            hw.Rucket.setPosition(.18);
            hw.Lucket.setPosition(.27);

            positionOfSlides = 1250;



        }
        if (gamepad2.y) {
            hw.Rucket.setPosition(.18);
            hw.Lucket.setPosition(.27);

            positionOfSlides = 3300;



        }


        hw.Rinear.setTargetPosition(positionOfSlides);
        hw.Linear.setTargetPosition(positionOfSlides);

        hw.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hw.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hw.Rinear.setPower(1);
        hw.Linear.setPower(1);





        telemetry.addData("bot heading", botHeading);
            telemetry.addData("Lucket", hw.Lucket.getPosition());
            telemetry.addData("Rucket", hw.Rucket.getPosition());
            telemetry.addData("Timer", myStopwatch.seconds());
            telemetry.update();
        }



}




