package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="Skibidi Drive")
public class SkibidiDrive extends OpMode {




    HardwareSoftware hw = new HardwareSoftware();
    @Override
    public void init() {


        hw.init(hardwareMap);








    }


    double y = 0;
    double rx = 0;


    double x = 0;


    int p = -25;


    @Override
    public void loop() {


        y = gamepad1.left_stick_y;
        rx = gamepad1.right_stick_x;
        x = gamepad1.left_stick_x;


        hw.FLdrive().setPower(-(y-rx-x));
        hw.FRdrive().setPower(-(y+rx+x));
        hw.BLdrive().setPower(-(y-rx+x));
        hw.BRdrive().setPower(-(y+rx-x));


        if (gamepad1.right_bumper){
            hw.Linear().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hw.Linear().setPower(-1);
        }
        else if (gamepad1.left_bumper){
            hw.Linear().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hw.Linear().setPower(.5);

        }
        else if (hw.Linear().getMode() == DcMotorEx.RunMode.RUN_USING_ENCODER){
            hw.Linear().setPower(0);
        }


        if (gamepad1.dpad_left) {
            p = -1000;
            hw.Linear().setTargetPosition(p);
            hw.Linear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.Linear().setVelocity(2000);
        }
        if (gamepad1.dpad_up) {
            p = -1500;
            hw.Linear().setTargetPosition(p);
            hw.Linear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.Linear().setVelocity(2000);
        }
        if (gamepad1.dpad_right) {
            p = -2000;
            hw.Linear().setTargetPosition(p);
            hw.Linear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.Linear().setVelocity(2000);
        }
        if (gamepad1.dpad_down) {
            p = -25;
            hw.Linear().setTargetPosition(p);
            hw.Linear().setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hw.Linear().setVelocity(2000);
            telemetry.addData("gyatt", hw.Linear().getMode());
        }

        if (gamepad1.a) {
            hw.Flippy().setPosition(hw.FlipMid);
        }
        if (gamepad1.b) {
            hw.Flippy().setPosition(hw.FlipDown);
        }
        if (gamepad1.x) {
            hw.Flippy().setPosition(hw.FlipUp);
        }
        if (gamepad1.right_trigger > 0.1) {
            hw.Grippy().setPosition(hw.GripClose);
        }
        if (gamepad1.left_trigger > 0.1) {
            hw.Grippy().setPosition(hw.GripOpen);
        }
    }


}




