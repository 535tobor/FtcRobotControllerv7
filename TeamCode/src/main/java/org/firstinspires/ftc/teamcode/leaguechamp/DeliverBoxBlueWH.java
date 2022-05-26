package org.firstinspires.ftc.teamcode.leaguechamp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "DeliverBlueWarehouse")
@Disabled
public class DeliverBoxBlueWH extends HwMap {
    @Override
    public void runOpMode()
    {
        side = "left";
        initHwMap();
        while(!isStarted()&&!isStopRequested())
        {
            runtime.reset();
            while(runtime.time()<3)
            {
                distanceArrayAverage();
                break;
            }
            barcodeDetect();
            updateGyro();
            telemetry.update();
        }
        if(opModeIsActive())
        {
            deliverBox();
            robotTurn(-70,4);
            driveByTime(-.8, 3.5);

        }
    }
}

