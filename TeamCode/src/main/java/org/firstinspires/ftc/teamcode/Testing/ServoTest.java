package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="Servo Test")
public class ServoTest extends OpMode {




    HardwareSoftware hw = new HardwareSoftware();
    @Override
    public void init() {
        hw.init(hardwareMap);


    }


    @Override
    public void loop() {

        if (gamepad2.dpad_down && !(hw.Linear.getCurrentPosition() > 500) && !(hw.Rinear.getCurrentPosition() > 500)) {  //bottom
            hw.Lucket.setPosition(1);   //originally .72
            hw.Rucket.setPosition(1); //originally .92

        }
        else if (gamepad2.dpad_up && !(hw.Linear.getCurrentPosition() > 500) && !(hw.Rinear.getCurrentPosition() > 500)) {  //top
            hw.Lucket.setPosition(0);//originally .1
            hw.Rucket.setPosition(0);  //originally .3
        }


        telemetry.addData("Lucket", hw.Lucket.getPosition());
        telemetry.addData("Rucket", hw.Rucket.getPosition());
        telemetry.update();
    }


}




