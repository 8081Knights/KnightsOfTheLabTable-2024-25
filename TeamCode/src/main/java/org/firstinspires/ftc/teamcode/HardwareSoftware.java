package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareSoftware {

    private HardwareMap hw = null;

    /*
    Hey, this is sam, I have a couple things to comment here

    could you take the DcMotorEx public and remove the &&drive() methods at the end
    You then don't need to address the motors as motor().something(),
    rather just motor.something() when you write your code.

    It'll also make the retrieval of encoders and setting of power much faster compared to the
    way it works now.
     */

//yo it's Luke, if you want unneeded changes make them yourself, and changing how we call the motors won't make retrieving info from encoders much faster

    DcMotorEx FRdrive    = null;
    DcMotorEx BRdrive     = null;
    DcMotorEx BLdrive      = null;
    DcMotorEx FLdrive     = null;

    DcMotorEx Linear1   = null;
    DcMotorEx Linear2   = null;

    SparkFunOTOS gyro;


    public void init(HardwareMap ahw){


        hw = ahw;

        FLdrive = hw.get(DcMotorEx.class, "FLdrive");
        FRdrive = hw.get(DcMotorEx.class, "FRdrive");
        BLdrive = hw.get(DcMotorEx.class, "BLdrive");
        BRdrive = hw.get(DcMotorEx.class, "BRdrive");

        FLdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BRdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FRdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BLdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        FLdrive.setDirection(DcMotorEx.Direction.REVERSE);
        FRdrive.setDirection(DcMotorEx.Direction.FORWARD);
        BLdrive.setDirection(DcMotorEx.Direction.FORWARD);
        BRdrive.setDirection(DcMotorEx.Direction.REVERSE);

        Linear1 = hw.get(DcMotorEx.class, "Linear1");
        Linear2 = hw.get(DcMotorEx.class, "Linear2");

        // should DcMotor be DcMotorEx?
        // yes it should be, velocity becomes avlailable

        FLdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        gyro = hw.get(SparkFunOTOS.class, "gyro");

        Linear1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Linear2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

    }

    public DcMotorEx FLdrive(){
        return FLdrive;
    }

    public DcMotorEx FRdrive(){
        return FRdrive;
    }

    public DcMotorEx BLdrive(){
        return BLdrive;
    }

    public DcMotorEx BRdrive(){
        return BRdrive;
    }

    public DcMotorEx Linear1(){
        return Linear1;
    }

    public DcMotorEx Linear2(){
        return Linear2;
    }

    public SparkFunOTOS gyro(){return gyro;}

}



