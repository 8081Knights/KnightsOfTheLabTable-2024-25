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

    DcMotorEx Linear;
    DcMotorEx Rinear;




    @Override
    public void init() {
        Linear = hardwareMap.get(DcMotorEx.class, "Linear");
        Rinear = hardwareMap.get(DcMotorEx.class, "Rinear");

        Linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Rinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }

    @Override
    public void loop() {
        // add telemetry to record the encoder values as we see how far they go.
        encoderValues[0] = Linear.getCurrentPosition();
        encoderValues[1] = Rinear.getCurrentPosition();

        telemetry.addData("Encoder Values Linear", Linear.getCurrentPosition());
        telemetry.addData("Encoder Values Rinear", Rinear.getCurrentPosition());

        telemetry.update();

    }
}
