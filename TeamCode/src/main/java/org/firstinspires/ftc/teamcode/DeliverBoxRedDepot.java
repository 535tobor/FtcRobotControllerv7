package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DeliverRedDepot")
//@Disabled
public class DeliverBoxRedDepot extends  HwMap{
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode()
    {
        side = "left";
        initHwMap();
        while(!isStarted()&&!isStopRequested())
        {
            barcodeDetect();
            updateGyro();
            telemetry.addData("", pincer.getPosition());
            telemetry.update();
        }
        if(opModeIsActive())
        {
            deliverBox(-60, 32);
            //if red depot:
//            motorRTP(extend, extend.getCurrentPosition()-10000, -.7);
//            encoderDrive(-.7, -.7, 8, 5, 1);
//            turn(-60, 4);
//            encoderDrive(-.7, -.7, 25, 6, 1);
//            /*
//                    - back up for a few seconds or inches to get arm out of way
//                    - turn so back of robot is facing depot
//                    - back into carousel using distance sensor and wall
//                    - spin the carousel
//                        - run carouSpin for a few seconds at power
//                    - drive forwards to back away from carousel
//                    - turn to face parking
//                    - drive forwards to park
//             */
        }
    }
}

