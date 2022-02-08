package org.firstinspires.ftc.teamcode.objdetection.tfrectm.tfod;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HwMap;

import java.util.List;

@Autonomous(name = "Autov2")
@Disabled
public class AutoV2 extends HwMap {
    String currObj = "";
    boolean isCloseUp = false, isObjectsDetected =true;
    double height, camAngle, speed;
    double DUCK_CLOSEUP_HEIGHT = 320;
    double BALL_CLOSEUP_HEIGHT = 430;
    double CUBE_CLOSEUP_HEIGHT = 375;
    double currObjCloseUpHeight;
    double initDividend =0;
    double roboAngle = 0;

    //think about  what if the camera puts the box around half of the object or just not the entire object; max height is not accurate ^^
    //find threshold so the height is when the camera is centered on certain object and it is or almost is the height of the frame (closest to the object)


    private ElapsedTime timer = new ElapsedTime();
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY = "AUSkSu//////AAABmbCDZkCjMUZdqlgBwBf0R6QmC3soC8rFfreNCDvJb7mhs7v6sWWIDBGTsR+tQeD9bSVikOsd2FpCDCK5qtLAy1U9ZgJZYN5O1IY3tuB6mnInb759EdsgxKJJT4OVFT1+QnozHYvi5BFK+Fwke9UKEohiv7baoXoYZbwDnjkTz6t1b5lg1em2Ebk2KGP3jOKS7fJkjQACDxIH9ikJ8/ShRnhMzVYge98MMhNxNTGM6T4rrdeBXZrS/pHAW9xu0k846P0/njOAVxgxgUywkX3GbbyqRuqio2KsQX9qCu+bGGEh08moFoMGdcX91l2QzOMkF7zjfFvmZfW8Aeth3sCt2+KonhX5vGAxnMeec0WZ105Q";

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initVuforia();
        initTfod();
        initHwMap();

        if (tfod != null) {
            tfod.activate();

            //tfod.setZoom(2, 16.0/9.0);
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        while (!isStarted() && !isStopRequested()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    //isObjectsDetected = updatedRecognitions.size()>0;
                    telemetry.addData("object detected?: ", isObjectsDetected);
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                        telemetry.addData("angle: ",recognition.estimateAngleToObject(AngleUnit.DEGREES));
                        telemetry.addData("height: ",recognition.getHeight());
                        currObj = recognition.getLabel();
                        camAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);
                        height = recognition.getHeight();
                        //roboAngle = getIntegratedHeading();
                        if(currObj.equals("Ball"))
                        {
                            currObjCloseUpHeight = BALL_CLOSEUP_HEIGHT;
                        }
                        else if(currObj.equals("Cube"))
                        {
                            currObjCloseUpHeight = CUBE_CLOSEUP_HEIGHT;
                        }
                        else {
                            currObjCloseUpHeight = DUCK_CLOSEUP_HEIGHT;
                        }

                        initDividend = currObjCloseUpHeight-height;
                        telemetry.addData("max object height ", currObjCloseUpHeight);
                        telemetry.addData("da dividend (max-currentHeight) ",initDividend);
                        updateGyro();


                        //if()//assuming you start facing the barcode and ducks this conditional checks for a duck in robot's proximity(check for proximity like don't check if too far away from object)
                    }
                    telemetry.update();
                    //gyroAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


                }

            }
        }
        timer.reset();
        if (opModeIsActive()) {
                while(opModeIsActive()&&!isCloseUp) // drive robot to object while it is not close (at the threshold) so that it reaches the object and ends up close
                {
                    if (tfod != null) {
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            isObjectsDetected= updatedRecognitions.size()>0;
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {
                                i++;//
                                height = recognition.getHeight();
                                telemetry.addData("is the robot close up to an object?: ", isCloseUp);
                                telemetry.addData("according to what object, max height: ", currObjCloseUpHeight);
                                telemetry.addData("curr height: ", height);
                                speed = ((currObjCloseUpHeight-height)/initDividend);
                                fl.setPower(speed);
                                fr.setPower(speed);
                                bl.setPower(speed);
                                br.setPower(speed);
                                /*
                                if(angleToObject>0)
                                {
                                    flSpeed = ((currObjCloseUpHeight-height)/initheight)+((angleToObject-currentAngle)/60);
                                    frSpeed = (currObjCloseUpHeight-height)/initheight;
                                    //algorithm for speed slowing down as the robot approaches the object but we will want to factor in the angle of the robot so that it is facing the object
                                }
                                else
                                {
                                    frSpeed = ((currObjCloseUpHeight-height)/initheight)+((angleToObject-currentAngle)/60);
                                    flSpeed = (currObjCloseUpHeight-height)/initheight;
                                }
                                fl.setPower(speed);
                                fr.setPower(speed);
                                 */
                            }
                            telemetry.update();
                        }
                    }
                    if(!isObjectsDetected)
                    {
                        isCloseUp=true;
                        fl.setPower(0);
                        fr.setPower(0);
                    }
                }

            //turn to drive to

        }


    }

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
