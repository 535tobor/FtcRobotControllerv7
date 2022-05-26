package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Park")
//@Disabled
public class ParkProgram extends HwMapLinearState {


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
            //pincerGrip();
            driveByTime(.7, 1);
            //turn(90, 5);
            //driveByTime(.5, 1.5);
        }
    }
}
