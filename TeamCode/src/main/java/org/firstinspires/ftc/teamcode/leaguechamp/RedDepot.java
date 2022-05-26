package org.firstinspires.ftc.teamcode.leaguechamp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Red/Blue Depot")
@Disabled
public class RedDepot extends HwMap {


    @Override
    public void runOpMode()
    {
        initHwMap();
        setModeAll(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(!isStarted()&&!isStopRequested()) //init
        {
            updateGyro();
        }
        if(opModeIsActive()) //run
        {
            //extend.setPower(.4);
            driveByTime(.6,2);
        }
    }
}
