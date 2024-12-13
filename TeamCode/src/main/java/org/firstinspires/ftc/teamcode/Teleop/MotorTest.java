package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="Motor Test")
public class MotorTest extends OpMode {




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
        currentSetPosition = positions[0];

        hw.Rinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.Linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hw.Rinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hw.Linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    double DS = 1;
    boolean D = true, D2=true;

    // FixedLinear Vars
    double[] pidValues = {2, 0, 0};
    int[] encoderValues = {3286, 3264};
    double height = 28.5d;
    double[] positions = {0,13,25};
    double currentSetPosition;
    int[] positionsEncoderValues = {0,0};

    boolean wasBumperPressed = false;  // Track previous bumper state


    @Override
    public void loop() {

        if (gamepad1.left_trigger > .1) {
            hw.FLdrive().setPower(1);
        }
        else{
            hw.FLdrive().setPower(0);
        }

        if (gamepad1.left_bumper) {
            hw.BLdrive().setPower(1);
        }
        else{
            hw.BLdrive().setPower(0);
        }

        if (gamepad1.right_trigger > .1) {
            hw.FRdrive().setPower(1);
        }
        else{
            hw.FRdrive().setPower(0);
        }

        if (gamepad1.right_bumper) {
            hw.BRdrive().setPower(1);
        }
        else{
            hw.BRdrive().setPower(0);
        }

    }


}




