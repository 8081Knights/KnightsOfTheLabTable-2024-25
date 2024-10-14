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
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        hw.FLdrive().setPower(((rotY + rotX + rx) / denominator) * DS);
        hw.BLdrive().setPower(((rotY - rotX + rx) / denominator) * DS);
        hw.FRdrive().setPower(((rotY + rotX - rx) / denominator) * DS);
        hw.BRdrive().setPower(((rotY - rotX - rx) / denominator) * DS);

        if(gamepad2.right_trigger > .1) {
            hw.Linear1().setPower(gamepad2.right_trigger);
            hw.Linear2().setPower(gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger > .1) {
            hw.Linear1().setPower(-gamepad2.left_trigger);
            hw.Linear2().setPower(-gamepad2.left_trigger);
        }
        else{
            hw.Linear1().setPower(0);
            hw.Linear2().setPower(0);
        }
        if (gamepad1.right_bumper && D == true) {
            DS = .2;
            D = false;
        }

        else if (gamepad1.right_bumper && D ==false){
            DS = 1;
            D = true;
        }


// add reset in case headless needs to be reset
    }


}




