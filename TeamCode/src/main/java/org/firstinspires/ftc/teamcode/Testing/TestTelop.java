package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name="testConfig")
public class TestTelop extends OpMode {
    HardwareSoftware robot = new HardwareSoftware();

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.FLdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.BLdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FRdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.BRdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.gyro.setLinearUnit(DistanceUnit.INCH);
        robot.gyro.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        robot.gyro.setOffset(offset);
        robot.gyro.setLinearScalar(1.0);
        robot.gyro.setAngularScalar(1.0);
        robot.gyro.calibrateImu();
        robot.gyro.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        robot.gyro.setPosition(currentPosition);

    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        SparkFunOTOS.Pose2D pos = robot.gyro.getPosition();

        double botHeading = pos.h;

        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        // Rotate the movement direction counter to the bot's rotation
        double rotY = y * Math.cos(botHeading) - x * Math.sin(botHeading);
        double rotX = y * Math.sin(botHeading) + x * Math.cos(botHeading);
        telemetry.addData("newX", rotX);
        telemetry.addData("newY", rotY);
        telemetry.addData("rx", rx);

        telemetry.addData("gyX", pos.x);
        telemetry.addData("gyY", pos.y);
        telemetry.addData("gyR", pos.h);

        rotX = rotX * 1.1;  // Counteract imperfect strafing


        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        robot.FLdrive.setPower((( -rotY - rotX - rx) / denominator) * .8);
        robot.BLdrive.setPower((( -rotY + rotX - rx) / denominator) * .8);
        robot.FRdrive.setPower((( -rotY + rotX + rx) / denominator) * .8);
        robot.BRdrive.setPower((( -rotY - rotX + rx) / denominator) * .8);

//        robot.FLdrive.setPower(y);
//        robot.BLdrive.setPower(y);
//        robot.FRdrive.setPower(y);
//        robot.BRdrive.setPower(y);
    }
}
