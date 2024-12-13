package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="Uhhh, BrUh (not comp)")
public class TestAuto1 extends LinearOpMode {

    double[] initPositions = {0,0,0};
    int currentInstruction = 0;
    HardwareSoftware robot = new HardwareSoftware();
    List<NewPositionOfRobot> robotPoses = new ArrayList<>();
    SparkFunOTOS.Pose2D pos;


    CurrentRobotPose currentPose = new CurrentRobotPose();

    private static ElapsedTime myStopwatch1 = new ElapsedTime();

    private static ElapsedTime myStopwatch2 = new ElapsedTime();


    public void initThis() {
        robot.init(hardwareMap);

        robot.FLdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.BLdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FRdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.BRdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        robotPoses.add(new NewPositionOfRobot(5  ,5 ,0));
        robotPoses.add(new NewPositionOfRobot(-7  ,5 , Math.PI * 3 / 4 , .4));
        robotPoses.add(new NewPositionOfRobot(-16.2,2 ,Math.PI * 3 / 4 , .3));
        robotPoses.add(new NewPositionOfRobot(-7,38.5 ,Math.PI * 2 / 4 , .6));


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


        currentPose.init(robot,initPositions[0],initPositions[1],initPositions[2]);

    }



    @Override
    public void runOpMode() throws InterruptedException {

        initThis();


        waitForStart();


        double cerror;

        while (opModeIsActive() && !isStopRequested()) {
            pos = robot.gyro.getPosition();
            telemetry.addData("Posx", pos.x);
            telemetry.addData("Posy", pos.y);
            telemetry.addData("Posh", pos.h);
            telemetry.addData("Step", currentInstruction);
            telemetry.update();

            currentPose.gyX = pos.x;
            currentPose.gyY = pos.y;
            currentPose.gyR = pos.h;

            currentPose.updateRealRobotPositions(pos);

            cerror = currentPose.moveToSetPosition(robotPoses.get(currentInstruction));

            telemetry.addData("cerror", cerror);

            if (currentInstruction == 0) {
                robot.Lucket.setPosition(.685);
                robot.Rucket.setPosition(.65  );
                myStopwatch1.reset();
                myStopwatch1.startTime();
            }


            if (currentInstruction == 1) {


                if(myStopwatch1.seconds() > 1){
                    robot.Linear.setTargetPosition(3200);
                    robot.Rinear.setTargetPosition(3200);

                    robot.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.Linear.setPower(1);
                    robot.Rinear.setPower(1);
                }

                myStopwatch2.reset();
                myStopwatch2.startTime();

            }

            if (currentInstruction == 3) {


                if(myStopwatch2.seconds() > 1){
                    robot.Linear.setTargetPosition(25);
                    robot.Rinear.setTargetPosition(25);

                    robot.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    robot.Linear.setPower(1);
                    robot.Rinear.setPower(1);
                }

                if(myStopwatch2.seconds() > 1 && robot.Linear.getCurrentPosition() < 40 && robot.Rinear.getCurrentPosition() < 40){
                    robot.Lucket.setPosition(.06);
                    robot.Rucket.setPosition(.06);
                }

            }

            if (currentInstruction == 5) {
                robot.Lucket.setPosition(.685);
                robot.Rucket.setPosition(.65  );
                myStopwatch1.reset();
                myStopwatch1.startTime();
            }


//literally has less than .2 like 1/4 of the time so im making it larger
            if (Math.abs(cerror) < 2 && currentInstruction != 4) {
                currentInstruction++;
            }
            telemetry.addData("cu", currentInstruction);


        }

    }



    /**
     * Represents the new position and rotation of the robot.
     */
    public class NewPositionOfRobot {
        boolean justDrive;
        double newx, newy;
        double newRotation;
        double speed = .8;
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
        NewPositionOfRobot(double nx, double ny, double newRot, double setspeed) {
            this.newx = nx;
            this.newy = ny;
            this.newRotation = newRot;
            this.speed = setspeed;
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

            if (powdX > 3 || powdX < -3) {
                powX = Math.signum(powdX);
            } else {
                powX = powdX/3;
            }

            if (powdY > 3 || powdY < -3) {
                powY = Math.signum(powdY);
            } else {
                powY = powdY/3;
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

            if (Math.abs(diffAngles[1]) < Math.abs(diffAngles[0])) {
                goodindex =1;
            }
            if (Math.abs(diffAngles[2]) < Math.abs(diffAngles[0])) {
                goodindex = 2;
            }
            if (Math.abs(diffAngles[2]) < Math.abs(diffAngles[1])) {
                goodindex = 2;
            }
            if (Math.abs(diffAngles[1]) < Math.abs(diffAngles[2])) {
                goodindex = 1;
            }

            rx = diffAngles[goodindex];

            telemetry.addData("diffIn0", diffAngles[0]);
            telemetry.addData("diffIn1", diffAngles[1]);
            telemetry.addData("diffIn2", diffAngles[2]);


            double realSetY = powY * Math.cos(realRobotHeading) - powX * Math.sin(realRobotHeading);
            double realSetX = powY * Math.sin(realRobotHeading) + powX * Math.cos(realRobotHeading);

            telemetry.addData("powx", powX);
            telemetry.addData("powy", powY);
            telemetry.addData("realSetX", realSetX);
            telemetry.addData("realSetY", realSetY);
            telemetry.addData("rx", rx);




            double denominator = Math.max(Math.abs(powY) + Math.abs(powX) + Math.abs(rx), 1);

            robot.FLdrive.setPower((( -realSetY - realSetX - rx) / denominator) * setPose.speed);
            robot.BLdrive.setPower((( -realSetY + realSetX - rx) / denominator) * setPose.speed);
            robot.FRdrive.setPower((( -realSetY + realSetX + rx) / denominator) * setPose.speed);
            robot.BRdrive.setPower((( -realSetY - realSetX + rx) / denominator) * setPose.speed);
            currentError = Math.abs(powdX) + Math.abs(powdY) + Math.abs(rx);

            return currentError;
        }
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
