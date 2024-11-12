package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


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
    }

    double DS = 1;
    boolean D = true;


    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        SparkFunOTOS.Pose2D pos = hw.gyro().getPosition();

        double botHeading = -Math.toRadians(pos.h);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        hw.FLdrive().setPower(((rotY + rotX + rx) / denominator) * DS);
        hw.BLdrive().setPower(((rotY - rotX + rx) / denominator) * DS);
        hw.FRdrive().setPower(((rotY + rotX - rx) / denominator) * DS);
        hw.BRdrive().setPower(((rotY - rotX - rx) / denominator) * DS);

        if(gamepad2.right_bumper) {
            hw.Linear().setPower(1);
            hw.Rinear().setPower(1);
        }
        else if (gamepad2.left_bumper) {
            hw.Linear().setPower(-1);
            hw.Rinear().setPower(-1);
        }
        else{
            hw.Linear().setPower(0);
            hw.Rinear().setPower(0);
        }

        if (gamepad1.right_bumper && D == true) {
            DS = .2;
            D = false;
        }

        else if (gamepad1.right_bumper && D ==false){
            DS = 1;
            D = true;
        }

        if(gamepad2.a) {
            hw.InLinear().setPower(1);
        }
        else if (gamepad2.x) {
            hw.InLinear().setPower(-1);
        }
        else{
            hw.InLinear().setPower(0);
        }

        if(gamepad2.right_trigger > .1) {
            hw.Intake().setPower(gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger > .1) {
            hw.Intake().setPower(-gamepad2.left_trigger);
        }
        else{
            hw.Intake().setPower(0);
        }

        if (gamepad2.y)  //up
        {
            hw.Lucket().setPosition(1);  //originally 0
            hw.Rucket().setPosition(-.75);  //originally .5
        }
        else if (gamepad2.b) //down
        {
            hw.Lucket().setPosition(0);   //originally 1
            hw.Rucket().setPosition(.25); //originally -1
        }


// add reset in case headless needs to be reset

        telemetry.addData("bot heading", botHeading);
        telemetry.addData("Lucket", hw.Lucket().getPosition());
        telemetry.addData("Rucket", hw.Rucket().getPosition());
        telemetry.update();
    }


}




