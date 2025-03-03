package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareSoftware {

    private HardwareMap hw = null;




    public DcMotorEx FRdrive    = null;
    public DcMotorEx BRdrive     = null;
    public DcMotorEx BLdrive      = null;
    public DcMotorEx FLdrive     = null;

    public DcMotor Intake    = null;

    public DcMotorEx InLinear    = null;


    public DcMotorEx Linear   = null;
    public DcMotorEx Rinear   = null;

    public Servo Lucket     = null;
    public Servo Rucket     = null;

    public Servo MrMini     = null;

    public SparkFunOTOS gyro;


    public void init(HardwareMap ahw){


        hw = ahw;

        FLdrive = hw.get(DcMotorEx.class, "FLdrive");
        FRdrive = hw.get(DcMotorEx.class, "FRdrive");
        BLdrive = hw.get(DcMotorEx.class, "BLdrive");
        BRdrive = hw.get(DcMotorEx.class, "BRdrive");

        InLinear = hw.get(DcMotorEx.class, "InLinear");

        Intake = hw.get(DcMotor.class, "Intake");

        Linear = hw.get(DcMotorEx.class, "Linear");
        Rinear = hw.get(DcMotorEx.class, "Rinear");

        Lucket = hw.get(Servo.class, "Lucket");
        Rucket = hw.get(Servo.class, "Rucket");

        MrMini = hw.get(Servo.class, "MrMini");

        FLdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BRdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FRdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BLdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        Linear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        Rinear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

//        Linear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
//        Rinear.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        InLinear.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        InLinear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        FLdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BLdrive.setDirection(DcMotorSimple.Direction.FORWARD);
        FRdrive.setDirection(DcMotorSimple.Direction.REVERSE);
        BRdrive.setDirection(DcMotorSimple.Direction.FORWARD);

        Linear.setDirection(DcMotorSimple.Direction.REVERSE);


        FLdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLdrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        gyro = hw.get(SparkFunOTOS.class, "gyro");

        Linear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Rinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        InLinear.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Lucket.setDirection(Servo.Direction.REVERSE);

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

    public DcMotor Intake(){
        return Intake;
    }

    public DcMotorEx Linear(){
        return Linear;
    }

    public DcMotorEx Rinear(){
        return Rinear;
    }

    public DcMotorEx InLinear(){
        return InLinear;
    }


    public SparkFunOTOS gyro(){return gyro;}

    public Servo MrMini(){return MrMini;}

}



