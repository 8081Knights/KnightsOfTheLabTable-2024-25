package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="LinearTest")
public class LinearTest extends OpMode {


    double DS = .9;
    boolean D = true, D2=true;

    boolean wasBumperPressed = false;  // Track previous bumper state

    HardwareSoftware hw = new HardwareSoftware();


    @Override
    public void init() {



        hw.init(hardwareMap);

        hw.Rinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.Linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.Rinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.Linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }



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

        if(gamepad2.right_trigger > .1) {
            hw.Linear.setPower(gamepad2.right_trigger);
            hw.Rinear.setPower(gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger > .1) {
            hw.Linear.setPower(-gamepad2.left_trigger);
            hw.Rinear.setPower(-gamepad2.left_trigger);
        }
        else{
            hw.Linear.setPower(0);
            hw.Rinear.setPower(0);
        }

        if (gamepad1.left_trigger > .1) {
            hw.InLinear.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > .1) {
            hw.InLinear.setPower(-gamepad1.right_trigger * .6);
        } else {
            hw.InLinear.setPower(0);
        }

        telemetry.addData("Linear", hw.Linear.getCurrentPosition());
        telemetry.addData("Rinear", hw.Rinear.getCurrentPosition());
        telemetry.addData("InLinear", hw.InLinear.getCurrentPosition());
        telemetry.update();


    }


}




