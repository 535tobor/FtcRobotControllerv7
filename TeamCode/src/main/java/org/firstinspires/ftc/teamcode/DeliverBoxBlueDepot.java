package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "DeliverBlueDepot")
//@Disabled
public class DeliverBoxBlueDepot extends  HwMap{
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode()
    {
        side = "right";
        initHwMap();
        while(!isStarted()&&!isStopRequested())
        {
            updateGyro();
            telemetry.addData("", pincer.getPosition());
            telemetry.update();
        }
        if(opModeIsActive())
        {
            deliverBox(timer, 60, 32);

//            //if blue depot:
//            motorRTP(extend, extend.getCurrentPosition()-10000, -.7);
//            encoderDrive(-.7, -.7, 8, 5, 1);
//            turnCond(60, 4, !extend.isBusy(), extend);
//            encoderDrive(-.7, -.7, 25, 6, 1);
//            /*
//                    - back up for a few seconds or inches to get arm out of way
//                    - angled turn so back of robot is facing carousel at an angle
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

