package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="Uhhh, BrUh (not comp)")
public class TestAuto1 extends LinearOpMode {

    double[] initPositions = {0,0,0};
    int currentInstruction;
    HardwareSoftware robot = new HardwareSoftware();
    List<NewPositionOfRobot> robotPoses = new ArrayList<>();
    SparkFunOTOS.Pose2D pos;


    CurrentRobotPose currentPose = new CurrentRobotPose();


    public void initThis() {
        robot.init(hardwareMap);
        currentPose.init(robot,initPositions[0],initPositions[1],initPositions[2]);

        robotPoses.add(new NewPositionOfRobot(2,2,Math.PI));


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
    public void runOpMode() throws InterruptedException {

        initThis();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            pos = robot.gyro.getPosition();
            telemetry.addData("Posx", pos.x);
            telemetry.addData("Posy", pos.y);
            telemetry.addData("Posh", pos.h);
            telemetry.update();

            currentPose.gyX = pos.x;
            currentPose.gyY = pos.y;
            currentPose.gyR = pos.h;

            currentPose.updateRealRobotPositions(pos);
            telemetry.addData("cerror ",currentPose.moveToSetPosition(robotPoses.get(0)));




        }

    }



    /**
     * Represents the new position and rotation of the robot.
     */
    public class NewPositionOfRobot {
        boolean justDrive;
        double oldx, newx, oldy, newy;
        double oldRotation, newRotation;
        /**
         * Sets the robot's future position and rotation.
         *
         * @param nx     The new x-coordinate.
         * @param ny     The new y-coordinate.
         * @param newRot The new rotation angle.
         */
        NewPositionOfRobot(double nx, double ny, double newRot) {
            this.newx = nx;
            this.newy = ny;
            this.newRotation = newRot;
            justDrive = true;
        }

    }

    /**
     * Represents the current pose of the robot.
     */
    public class CurrentRobotPose {
        HardwareSoftware robotHardwaremap;
        double realRobotX, realRobotY, realRobotHeading;
        double gyX, gyY, gyR;

        double initX, initY, initZ;

        /**
         * Initializes the robot's pose.
         *
         * @param hwMap The hardware map.
         * @param inX   The initial x-coordinate.
         * @param inY   The initial y-coordinate.
         * @param inz   The initial z-coordinate.
         */
        public void init(HardwareSoftware hwMap, double inX, double inY, double inz) {
            this.robotHardwaremap = hwMap;
            this.initX = inX;
            this.initY = inY;
            this.initZ = inz;
        }

        /**
         * Updates the robot's real positions based on gyroscope values.
         *
         * @param gyroValue The current gyroscope position.
         */
        public void updateRealRobotPositions(SparkFunOTOS.Pose2D gyroValue) {
            gyX = gyroValue.x;
            gyY = gyroValue.y;
            gyR = gyroValue.h;

            realRobotX = initX + gyX;
            realRobotY = initY + gyY;
            realRobotHeading = initZ + normalizeAngle(gyR);
        }

        /**
         * Moves the robot to a set position and returns the current error.
         *
         * @param setPose The target position and rotation.
         * @return The current error between the robot's position and the target position.
         */
        double moveToSetPosition(NewPositionOfRobot setPose) {
            double currentError = 0;
            double powY, powX, rx =0;
            double powdY, powdX;

            powdX = setPose.newx - realRobotX;
            powdY = setPose.newy - realRobotY;

            if (powdX > 3) {
                powX = Math.signum(powdX);
            } else {
                powX = powdX;
            }

            if (powdY > 3) {
                powY = Math.signum(powdY);
            } else {
                powY = powdY;
            }

            double[] altAngles = new double[3];
            double[] diffAngles = new double[3];

            altAngles[0] =  setPose.newRotation - 2*Math.PI;
            altAngles[1] =  setPose.newRotation            ;
            altAngles[2] =  setPose.newRotation + 2*Math.PI;

            for (int i = 0; i < 3; ++i) {
                diffAngles[i] = altAngles[i] - realRobotHeading;
            }

            Arrays.sort(diffAngles);

            int goodindex = 0;

            if (Math.abs(diffAngles[1]) < Math.abs(diffAngles[2])) {
                goodindex =1;
            }

            rx = diffAngles[goodindex];
            //TODO: tune for similar logic above

            telemetry.addData("powx", powX);
            telemetry.addData("powy", powY);
            telemetry.addData("rx", rx);



            robotHardwaremap.FLdrive.setPower((powY + powX - rx) / 3 * .8);
            robotHardwaremap.BLdrive.setPower((powY - powX - rx) / 3 * .8);
            robotHardwaremap.FRdrive.setPower((powY + powX + rx) / 3 * .8);
            robotHardwaremap.BRdrive.setPower((powY - powX + rx) / 3 * .8);
            currentError = Math.abs(powdX) + Math.abs(powdY) + Math.abs(rx);

            return currentError;
        }
    }

    public class Action {


    }

    public static double normalizeAngle(double angle) {
        while (angle < 0) {
            angle += 2 * Math.PI;
        }
        while (angle >= 2 * Math.PI) {
            angle -= 2 * Math.PI;
        }
        return angle;
    }

}
