package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

public class NDrive extends OpMode {
    HardwareSoftware robot = new HardwareSoftware();

    double x,y,rx, botHeading, rotX, rotY, denominator;
    SparkFunOTOS.Pose2D pos;

    double DS = 1;
    boolean D = true;

    // FixedLinear Vars
    double[] pidValues = {2, 0, 0};
    int[] encoderValues = {3286, 3264};
    double height = 28.5d;
    double[] positions = {0,13,25};
    double currentSetPosition;
    int[] positionsEncoderValues = {0,0};

    boolean wasBumperPressed = false;  // Track previous bumper state


    @Override
    public void init() {
        robot.init(hardwareMap);

        robot.gyro.calibrateImu();
        robot.gyro.resetTracking();

        robot.Rinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Rinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        // Calculate joystick inputs and heading
        y = -gamepad1.left_stick_y; // Y stick is reversed
        x = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;

        pos = robot.gyro.getPosition();
        botHeading = -Math.toRadians(pos.h) + Math.PI;

        // Rotate joystick inputs based on heading
        rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Adjust strafing for better control
        rotX *= 1.1;

        // Calculate denominator to normalize power
        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        // Set motor powers based on normalized inputs and speed multiplier
        robot.FLdrive.setPower(((rotY + rotX + rx) / denominator) * DS);
        robot.BLdrive.setPower(((rotY - rotX + rx) / denominator) * DS);
        robot.FRdrive.setPower(((rotY + rotX - rx) / denominator) * DS);
        robot.BRdrive.setPower(((rotY - rotX - rx) / denominator) * DS);

        if (gamepad1.right_bumper && !wasBumperPressed) {  // Check if bumper is pressed and was not previously pressed
            if (D) {
                DS = 0.2;
                D = false;
            } else {
                DS = 1;
                D = true;
            }
            wasBumperPressed = true;  // Set flag to indicate bumper was pressed
        } else if (!gamepad1.right_bumper) {
            wasBumperPressed = false;  // Reset flag when bumper is released
        }


    }
}
