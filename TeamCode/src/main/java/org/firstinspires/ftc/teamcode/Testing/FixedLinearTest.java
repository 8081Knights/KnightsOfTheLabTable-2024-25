package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="FixedLinearTest")
public class FixedLinearTest extends OpMode {

    double[] pidValues = {2, 0, 0};
    int[] encoderValues = {3286, 3264};
    double height = 28.5d;


    double[] positions = {0,13,28};

    double currentSetPosition;

    int[] positionsEncoderValues = {0,0};

    HardwareSoftware hw = new HardwareSoftware();


    @Override
    public void init() {
        hw.init(hardwareMap);



        hw.Rinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.Linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.Rinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.Linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        hw.Rinear.setPower(((double) (positionsEncoderValues[0] - hw.Rinear.getCurrentPosition()) / encoderValues[0] ) * pidValues[0]);
        hw.Linear.setPower(((double) (positionsEncoderValues[1] - hw.Linear.getCurrentPosition()) / encoderValues[1] ) * pidValues[0]);


        telemetry.addData("Encoder Values hw.Rinear", hw.Rinear.getCurrentPosition());
        telemetry.addData("Encoder Values hw.Linear", hw.Linear.getCurrentPosition());
        telemetry.addData("encoderSet1",positionsEncoderValues[0]);
        telemetry.addData("encoderSet2",positionsEncoderValues[1]);
        telemetry.addData("encoderPower1",((double) (positionsEncoderValues[0] - hw.Rinear.getCurrentPosition()) / Math.abs(encoderValues[0]) ) * pidValues[0]);
        telemetry.addData("encoderPower2",((double) (positionsEncoderValues[1] - hw.Linear.getCurrentPosition()) / Math.abs(encoderValues[1]) ) * pidValues[0]);
        telemetry.addData("gameleft", gamepad1.left_stick_y);
        telemetry.addData("gameright", gamepad1.right_stick_y);

        telemetry.update();




    }
}
