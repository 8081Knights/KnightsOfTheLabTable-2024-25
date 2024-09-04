package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HardwareSoftware;


@Autonomous(name="Skibidi Auto")
public class SkibidiAuto extends LinearOpMode {


    HardwareSoftware hw = new HardwareSoftware();

    double p = 0;

    @Override
    public void runOpMode() {


        hw.init(hardwareMap);

        waitForStart();

        p = .5;

        hw.FLdrive().setPower(p);
        hw.FRdrive().setPower(p);
        hw.BLdrive().setPower(p);
        hw.BRdrive().setPower(p);

        sleep(1500);

        p = 0;

        hw.FLdrive().setPower(p);
        hw.FRdrive().setPower(p);
        hw.BLdrive().setPower(p);
        hw.BRdrive().setPower(p);

        sleep(200);

        p = .5;

        hw.FLdrive().setPower(p);
        hw.FRdrive().setPower(-p);
        hw.BLdrive().setPower(-p);
        hw.BRdrive().setPower(p);

        sleep(1500);

        p = 0;

        hw.FLdrive().setPower(p);
        hw.FRdrive().setPower(p);
        hw.BLdrive().setPower(p);
        hw.BRdrive().setPower(p);

        sleep(200);

        p = .5;

        hw.FLdrive().setPower(-p);
        hw.FRdrive().setPower(-p);
        hw.BLdrive().setPower(-p);
        hw.BRdrive().setPower(-p);

        sleep(1500);

        p = 0;

        hw.FLdrive().setPower(p);
        hw.FRdrive().setPower(p);
        hw.BLdrive().setPower(p);
        hw.BRdrive().setPower(p);

        sleep(200);

        p = .5;

        hw.FLdrive().setPower(-p);
        hw.FRdrive().setPower(p);
        hw.BLdrive().setPower(p);
        hw.BRdrive().setPower(-p);

        sleep(1500);

        p = 0;

        hw.FLdrive().setPower(p);
        hw.FRdrive().setPower(p);
        hw.BLdrive().setPower(p);
        hw.BRdrive().setPower(p);

        sleep(200);


    }
}




