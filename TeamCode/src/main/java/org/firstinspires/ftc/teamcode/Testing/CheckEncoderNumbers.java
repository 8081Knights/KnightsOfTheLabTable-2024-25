package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name = "EncoderRead")
public class CheckEncoderNumbers extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();

    // set the motor to be what its gonna be
    DcMotor testMotor = robot.Intake;
    double power;
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        power = gamepad2.right_stick_y;
        testMotor.setPower(power);

        telemetry.addData("Encoder", testMotor.getCurrentPosition());
    }
}
