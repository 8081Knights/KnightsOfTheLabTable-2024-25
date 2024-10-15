package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="FixedLinearTest")
public class FixedLinearTest extends OpMode {

    double[] pidValues = {6, 0, 0};
    int[] encoderValues = {3286, -3264};
    double height = 28.5d;
    DcMotor Linear1 = null;
    DcMotor Linear2 = null;

    double[] positions = {0,13,28};

    double currentSetPosition;

    int[] positionsEncoderValues = {0,0};




    @Override
    public void init() {
        Linear1 = hardwareMap.get(DcMotor.class, "Linear1");
        Linear2 = hardwareMap.get(DcMotor.class, "Linear2");

        Linear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Linear2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Linear1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Linear2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentSetPosition = positions[0];


    }

    @Override
    public void loop() {


        // add telemetry to record the encoder values as we see how far they go.
        if (gamepad1.a) {
            currentSetPosition = positions[0];
        } else if (gamepad1.b) {
            currentSetPosition = positions[1];
        } else if (gamepad1.y) {
            currentSetPosition = positions[2];
        }

        positionsEncoderValues[0] = (int) (currentSetPosition * (encoderValues[0] / height));
        positionsEncoderValues[1] = (int) (currentSetPosition * (encoderValues[1] / height));

        // pid control for linear slides

        Linear1.setPower(((double) (positionsEncoderValues[0] - Linear1.getCurrentPosition()) / encoderValues[0] ) * pidValues[0]);
        Linear2.setPower(-((double) (positionsEncoderValues[1] - Linear2.getCurrentPosition()) / encoderValues[1] ) * pidValues[0]);


        telemetry.addData("Encoder Values Linear1", Linear1.getCurrentPosition());
        telemetry.addData("Encoder Values Linear2", Linear2.getCurrentPosition());
        telemetry.addData("encoderSet1",positionsEncoderValues[0]);
        telemetry.addData("encoderSet2",positionsEncoderValues[1]);
        telemetry.addData("encoderPower1",((double) (positionsEncoderValues[0] - Linear1.getCurrentPosition()) / Math.abs(encoderValues[0]) ) * pidValues[0]);
        telemetry.addData("encoderPower2",((double) (positionsEncoderValues[1] - Linear2.getCurrentPosition()) / Math.abs(encoderValues[1]) ) * pidValues[0]);
        telemetry.addData("gameleft", gamepad1.left_stick_y);
        telemetry.addData("gameright", gamepad1.right_stick_y);

        telemetry.update();




    }
}
