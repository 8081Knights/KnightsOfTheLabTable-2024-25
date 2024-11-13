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

        if(gamepad2.right_trigger > .1) {
            hw.Linear.setPower(gamepad2.right_trigger);
            hw.Rinear.setPower(-gamepad2.right_trigger);
        }
        else if (gamepad2.left_trigger > .1) {
            hw.Linear.setPower(-gamepad2.left_trigger);
            hw.Rinear.setPower(gamepad2.left_trigger);
        }
        else{
            hw.Linear.setPower(0);
            hw.Rinear.setPower(0);
        }


    }


}




