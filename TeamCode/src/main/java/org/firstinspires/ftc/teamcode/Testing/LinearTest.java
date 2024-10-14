package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="LinearTest")
public class LinearTest extends OpMode {




    HardwareSoftware hw = new HardwareSoftware();
    DcMotorEx Linear1   = null;
    DcMotorEx Linear2   = null;


    @Override
    public void init() {



        Linear1 = hardwareMap.get(DcMotorEx.class, "Linear1");
        Linear2 = hardwareMap.get(DcMotorEx.class, "Linear2");

        Linear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Linear2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }



    @Override
    public void loop() {

        if(gamepad2.right_trigger > .1) {
            Linear1.setPower(gamepad2.right_trigger);
            Linear2.setPower(-gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger > .1) {
            Linear1.setPower(-gamepad2.left_trigger);
            Linear2.setPower(gamepad2.left_trigger);
        }
        else{
            Linear1.setPower(0);
            Linear2.setPower(0);
        }


    }


}




