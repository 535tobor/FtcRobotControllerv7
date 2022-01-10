package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="RightSidePark")
@Disabled
public class LeftBlue extends HwMap {


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
            driveByTime(.5, .75);
           // extend.setPower(0);
            turn(-69); // not to be funny, we are professionals

            driveByTime(.5, 2);
        }
    }
}
