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
@Autonomous(name="RUN TSTSTS")
public class ConnerAuto extends LinearOpMode {

    double[] initPositions = {0,0,0};
    int currentInstruction = 0;
    boolean isOkToMoveOn = true;
    HardwareSoftware robot = new HardwareSoftware();
    List<NewPositionOfRobot> robotPoses = new ArrayList<>();
    SparkFunOTOS.Pose2D pos;


    CurrentRobotPose currentPose = new CurrentRobotPose();

    private ElapsedTime myStopwatch1 = new ElapsedTime();
    private ElapsedTime myStopwatch2 = new ElapsedTime();
    private ElapsedTime pickupWatch1 = new ElapsedTime();
    boolean pickup1Started = false;

    private ElapsedTime dropoffWatch1 = new ElapsedTime();
    ElapsedTime caseStopwatch = new ElapsedTime();
    boolean timerGoodToGo = true;
    boolean dropoff1Started = false;

    private ElapsedTime servoWatch1 = new ElapsedTime();
    boolean servoStarted = false;

    double cTresh = 1;



    public void initThis() {
        robot.init(hardwareMap);

        robot.FLdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.BLdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.FRdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.BRdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.InLinear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.InLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.InLinear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robotPoses.add(new NewPositionOfRobot(5    ,5   ,0));
        robotPoses.add(new NewPositionOfRobot(0   ,11   ,Math.PI * 3 / 4 , .5));
        robotPoses.add(new NewPositionOfRobot(-16,3  ,Math.PI * 3 / 4 , .6));
        robotPoses.add(new NewPositionOfRobot(10   ,21   ,1.19));
        robotPoses.add(new NewPositionOfRobot(10   ,21   ,1.18));

        robotPoses.add(new NewPositionOfRobot(10   ,21   ,1.19));
        robotPoses.add(new NewPositionOfRobot(0    ,20   ,0, .7));
        robotPoses.add(new NewPositionOfRobot(0    ,20   ,0, .6));
        robotPoses.add(new NewPositionOfRobot(-5    ,7   ,Math.PI * 3 / 4 , .7));
        robotPoses.add(new NewPositionOfRobot(-14,-0  ,Math.PI * 3 / 4 , .6));

        robotPoses.add(new NewPositionOfRobot(5    ,29   ,Math.PI / 2     , .6));
        robotPoses.add(new NewPositionOfRobot(5    ,29   ,Math.PI / 2     , .6));
        robotPoses.add(new NewPositionOfRobot(5    ,29   ,Math.PI / 2     , .6));
        robotPoses.add(new NewPositionOfRobot(0    ,20   ,0, .7));
        robotPoses.add(new NewPositionOfRobot(0    ,20   ,0, .7));

        robotPoses.add(new NewPositionOfRobot(-5    ,7   ,Math.PI * 3 / 4 , .7));
        robotPoses.add(new NewPositionOfRobot(-15,3  ,Math.PI * 3 / 4 , .6));
        robotPoses.add(new NewPositionOfRobot(0    ,20   ,0, .7));




//        robotPoses.add(new NewPositionOfRobot(-7   ,30,Math.PI / 2 , .8));
//        robotPoses.add(new NewPositionOfRobot( 2   ,30,Math.PI / 2 , .7));
//        robotPoses.add(new NewPositionOfRobot( 2   ,30,Math.PI / 2 , .7));
//        robotPoses.add(new NewPositionOfRobot(-5   ,21  ,Math.PI / 2 , .7));
//        robotPoses.add(new NewPositionOfRobot(-7   ,5   ,Math.PI * 3 / 4 , .5));
//        robotPoses.add(new NewPositionOfRobot(-7   ,8   ,Math.PI * 3 / 4 , .5));
//        robotPoses.add(new NewPositionOfRobot(-16.2,4   ,Math.PI * 3 / 4 , .5));
//        robotPoses.add(new NewPositionOfRobot( 2   ,34.0,0 , .5));
//        robotPoses.add(new NewPositionOfRobot(-11  ,34.0,Math.PI / 2 , .7));
//        robotPoses.add(new NewPositionOfRobot(-7  ,34.0,Math.PI / 2 , .6));
//        robotPoses.add(new NewPositionOfRobot(-7  ,34.0,Math.PI / 2 , .6));






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
            telemetry.update();

            currentPose.gyX = pos.x;
            currentPose.gyY = pos.y;
            currentPose.gyR = pos.h;

            currentPose.updateRealRobotPositions(pos);

            cerror = currentPose.moveToSetPosition(robotPoses.get(currentInstruction));

            telemetry.addData("cerror", cerror);


            switch (currentInstruction){

                case 0:{
                    robot.MrMini.setPosition(1);
                    robot.Lucket.setPosition(.63);  //originally 0
                    robot.Rucket.setPosition(.71);  //originally 0
                    cTresh = 1.5;
                    if(caseStopwatch.seconds() < 2) {
                        isOkToMoveOn = false;
                    } else {
                        isOkToMoveOn = true;
                        robot.MrMini.setPosition(1);
                    }
                    break;
                }
                case 1:{
                    if(caseStopwatch.seconds() < .5){
                        robot.Linear.setTargetPosition(3150);
                        robot.Rinear.setTargetPosition(3150);

                        robot.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        robot.Linear.setPower(1);
                        robot.Rinear.setPower(1);
                        isOkToMoveOn = false;
                    } else {
                        robot.MrMini.setPosition(1);
                    }
                    if (robot.Linear.getCurrentPosition()-3300 < 200) {
                        isOkToMoveOn = true;
                    }
                break;
                }
                case 17:
                case 3:
                case 10: {
                    robot.MrMini.setPosition(0);
                    cTresh = 3;
                    if (caseStopwatch.seconds() > .5) {
                        robot.MrMini.setPosition(1);
                        robot.Lucket.setPosition(0.16);
                        robot.Rucket.setPosition(0.25);
                        robot.Linear.setTargetPosition(60);
                        robot.Rinear.setTargetPosition(60);

                        robot.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        robot.Linear.setPower(.8);
                        robot.Rinear.setPower(.8);
                        isOkToMoveOn = false;
                        if (robot.Linear.getCurrentPosition()-60 < 100) {
                            isOkToMoveOn = true;
                            robot.MrMini.setPosition(1);
                        }
                    }
                    break;
                }
                case 11:
                case 4: {
                    robot.MrMini.setPosition(1);


                    robot.Intake.setPower(.8);
                    if (caseStopwatch.seconds() > 1) {
                        robot.Lucket.setPosition(.04);
                        robot.Rucket.setPosition(.11);
                    }

                    if(caseStopwatch.seconds() > 2) {
                        robot.InLinear.setTargetPosition(-1600);
                        robot.InLinear.setPower(.8);
                        robot.InLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    isOkToMoveOn = robot.InLinear.getCurrentPosition() + 1600 < 200;
                    break;
                }
                case 12:
                case 5: {
                    if(caseStopwatch.seconds() < 1.5) {
                        isOkToMoveOn = false;
                    } else {
                        isOkToMoveOn = true;
                    }
                    break;
                }
                case 13:
                case 6:{
                    robot.MrMini.setPosition(0);
                    robot.InLinear.setTargetPosition(-200);
                    robot.InLinear.setPower(.8);
                    robot.InLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.Intake.setPower(0);



                    if (robot.InLinear.getCurrentPosition() > -300) {
                        robot.Intake.setPower(-.8);
                    } else {
                        robot.Intake.setPower(0);
                    }
                    isOkToMoveOn = robot.InLinear.getCurrentPosition() > -200;
                    if (isOkToMoveOn) {
                        robot.InLinear.setTargetPosition(0);
                        robot.InLinear.setPower(.8);
                        robot.InLinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    telemetry.addData("SILLY ", robot.InLinear.getCurrentPosition() > -200);
                    break;
                }
                case 14:
                case 7: {
                    robot.Lucket.setPosition(.04);
                    robot.Rucket.setPosition(.11);
                    robot.Intake.setPower(-.8);
                    if(caseStopwatch.seconds() < 1.5) {
                        isOkToMoveOn = false;
                        if (caseStopwatch.seconds() > 1) {
                            robot.MrMini.setPosition(1);
                            telemetry.addData("SILLY", "RUNNING DOWN|||||");
                        }
                    } else {
                        isOkToMoveOn = true;
                        robot.Intake.setPower(0);
                        robot.MrMini.setPosition(1);
                    }
                    break;
                }
                case 15:
                case 8: {
                    robot.MrMini.setPosition(1);
                    robot.Lucket.setPosition(.63);  //originally 0
                    robot.Rucket.setPosition(.71);  //originally 0
                    if(caseStopwatch.seconds() < 1){
                        robot.MrMini.setPosition(1);
                        robot.Linear.setTargetPosition(3150);
                        robot.Rinear.setTargetPosition(3150);

                        robot.Linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        robot.Rinear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        robot.Linear.setPower(1);
                        robot.Rinear.setPower(1);
                        isOkToMoveOn = false;
                    }
                    if (robot.Linear.getCurrentPosition()-3300 < 200) {
                        isOkToMoveOn = true;
                    }
                    break;
                }

            }



            if ((Math.abs(cerror) < cTresh) && (currentInstruction != 20) && isOkToMoveOn) {
                caseStopwatch.reset();
                caseStopwatch.startTime();
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
