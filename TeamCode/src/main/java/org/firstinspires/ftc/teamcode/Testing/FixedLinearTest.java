package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name="FixedLinearTest")
public class FixedLinearTest extends OpMode {

    double[] pidValues = {.1, 0, 0};
    int[] encoderValues = {0, 0};

    DcMotorEx Linear1;
    DcMotorEx Linear2;




    @Override
    public void init() {
        Linear1 = hardwareMap.get(DcMotorEx.class, "Linear1");
        Linear2 = hardwareMap.get(DcMotorEx.class, "Linear2");

        Linear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Linear2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Linear1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Linear2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Linear1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Linear2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }

    @Override
    public void loop() {
        // add telemetry to record the encoder values as we see how far they go.
        encoderValues[0] = Linear1.getCurrentPosition();
        encoderValues[1] = Linear2.getCurrentPosition();

        telemetry.addData("Encoder Values Linear1", Linear1.getCurrentPosition());
        telemetry.addData("Encoder Values Linear2", Linear2.getCurrentPosition());

        telemetry.update();

    }
}
