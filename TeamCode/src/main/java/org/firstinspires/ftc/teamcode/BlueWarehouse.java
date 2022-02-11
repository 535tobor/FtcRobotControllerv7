package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Blue Warehouse")
@Disabled
public class BlueWarehouse extends HwMap {


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

            driveByTime(-.6,.5);
            robotTurn(-90, 5);
            driveByTime(.6,2);
        }
    }
}
