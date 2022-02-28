package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DeliverRedWarehouse")
//@Disabled
public class DeliverBoxRedWH extends  HwMap{
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode()
    {
        side = "right";
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
        }
    }
}

