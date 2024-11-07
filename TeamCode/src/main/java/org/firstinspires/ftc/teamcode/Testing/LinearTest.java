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
    DcMotorEx Linear   = null;
    DcMotorEx Rinear   = null;


    @Override
    public void init() {



        Linear = hardwareMap.get(DcMotorEx.class, "Linear");
        Rinear = hardwareMap.get(DcMotorEx.class, "Rinear");

        Linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Rinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }



    @Override
    public void loop() {

        if(gamepad2.right_trigger > .1) {
            Linear.setPower(gamepad2.right_trigger);
            Rinear.setPower(-gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger > .1) {
            Linear.setPower(-gamepad2.left_trigger);
            Rinear.setPower(gamepad2.left_trigger);
        }
        else{
            Linear.setPower(0);
            Rinear.setPower(0);
        }


    }


}




