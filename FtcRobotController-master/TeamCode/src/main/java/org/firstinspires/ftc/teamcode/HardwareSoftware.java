package org.firstinspires.ftc.teamcode;

//import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareSoftware {

    private HardwareMap hw = null;

    Servo LeftHangServo = null;
    DcMotor HangMotor = null;

    Servo RightHangServo = null;


    DcMotorEx FRdrive    = null;
    DcMotorEx BRdrive     = null;
    DcMotorEx BLdrive      = null;
    DcMotorEx FLdrive     = null;

    DcMotorEx Linear     = null;

    CRServo Conveyor = null;

    Servo Grippy = null;
    Servo Flippy = null;
    Servo Plane = null;

    Servo PurplePixel = null;


    DcMotorEx Lodom = null;
    DcMotorEx Rodom = null;



//    AHRS gyro = null;

    public double GripClose = 0.7;
    public double GripOpen = 0.4;

    public double FlipUp = 0.8;
    public double FlipMid = 0.4;
    public double FlipDown = 0.175;

    public double LeftHangServoReleased = 0;
    public double LeftHangServoDown = 0.55;

    public double RightHangServoReleased = 1;
    public double RightHangServoDown = 0.45;

    public double PurplePixelDown = -0.2;
    public double PurplePixelReleased = 0.95;




    public void init(HardwareMap ahw){


        hw = ahw;

        FLdrive = hw.get(DcMotorEx.class, "FLdrive");
        FRdrive = hw.get(DcMotorEx.class, "FRdrive");
        BLdrive = hw.get(DcMotorEx.class, "BLdrive");
        BRdrive = hw.get(DcMotorEx.class, "BRdrive");

        Linear = hw.get(DcMotorEx.class, "Linear");

        Rodom = hw.get(DcMotorEx.class, "Rodom");
        Lodom = hw.get(DcMotorEx.class, "Lodom");

        Grippy = hw.get(Servo.class, "Grippy");
        Flippy = hw.get(Servo.class, "Flippy");
        Plane = hw.get(Servo.class, "Plane");

        Conveyor = hw.get(CRServo.class, "Conveyor");

        LeftHangServo = hw.get(Servo.class, "LeftHangServo");
        HangMotor = hw.get(DcMotor.class, "HangMotor");

        RightHangServo = hw.get(Servo.class, "RightHangServo");

        PurplePixel = hw.get(Servo.class, "PurplePixel");


//        gyro = AHRS.getInstance(hw.get(NavxMicroNavigationSensor.class, "gyro"),
//                AHRS.DeviceDataType.kProcessedData);

        FLdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BRdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        FRdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BLdrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //back odometry, right odometry, left odometry
        Rodom.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Lodom.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        Linear.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Linear.setDirection(DcMotorSimple.Direction.FORWARD);

        Flippy.setDirection(Servo.Direction.REVERSE);

        HangMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        FLdrive.setDirection(DcMotorEx.Direction.REVERSE);
        FRdrive.setDirection(DcMotorEx.Direction.FORWARD);
        BLdrive.setDirection(DcMotorEx.Direction.REVERSE);
        BRdrive.setDirection(DcMotorEx.Direction.FORWARD);



        FLdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLdrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Rodom.setMode(DcMotor.RunMode.RESET_ENCODERS);
        Lodom.setMode(DcMotor.RunMode.RESET_ENCODERS);

        Linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftHangServo.setPosition(LeftHangServoDown);
        RightHangServo.setPosition(RightHangServoDown);

        Grippy.setPosition(0.7);



        PurplePixel.setPosition(PurplePixelDown);

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

    public DcMotorEx Rodom(){
        return Rodom;
    }
    public DcMotorEx Lodom(){
        return Lodom;
    }

//    public AHRS gyro(){return gyro;}

    public DcMotorEx Linear(){
        return Linear;
    }

    public Servo Flippy(){
        return Flippy;
    }
    public Servo Grippy(){
        return Grippy;
    }
    public Servo Plane(){return Plane;}

    public CRServo Conveyor(){
        return Conveyor;
    }

    public Servo LeftHangServo(){return LeftHangServo;}
    public DcMotor HangMotor(){return HangMotor;}

    public Servo RightHangServo(){return RightHangServo;}

    public Servo PurplePixel(){return PurplePixel;}


}



