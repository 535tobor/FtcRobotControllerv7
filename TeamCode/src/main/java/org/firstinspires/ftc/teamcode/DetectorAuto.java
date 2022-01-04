package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tfrec.Detector;
import org.firstinspires.ftc.teamcode.tfrec.classification.Classifier;

import java.util.List;

@Autonomous(name = "DetectorAuto")
@Disabled
public class DetectorAuto extends LinearOpMode{

    // Declare OpMode members.
    private Detector tfDetector = null;
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime time = new ElapsedTime();

    private static String MODEL_FILE_NAME = "custom.tflite";
    private static String LABEL_FILE_NAME = "custom_labels.txt";
    private static Classifier.Model MODEl_TYPE = Classifier.Model.FLOAT_EFFICIENTNET;

    @Override
    public void runOpMode() {
        try {
            try {
                tfDetector = new Detector(MODEl_TYPE, MODEL_FILE_NAME, LABEL_FILE_NAME, hardwareMap.appContext, telemetry);
                tfDetector.activate();
            }
            catch (Exception ex){
                telemetry.addData("Error", String.format("Unable to initialize Detector. %s", ex.getMessage()));
                sleep(3000);
                return;
            }

            telemetry.addData("Detector", "Ready");
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            while (!isStarted()&&!isStopRequested()) {
                List<Classifier.Recognition> results = tfDetector.getLastResults();
                if (results == null || results.size() == 0){
                    telemetry.addData("Info", "No results");
                }
                else {
                    for (Classifier.Recognition r : results) {
                        String item = String.format("%s: %.2f", r.getTitle(), r.getConfidence());
                        telemetry.addData("Found", item);

                    }
                }
                telemetry.update();
            }
            runtime.reset();

            // run & end when done // if in while(opmodeisactive) it will run until the end of the match (driver presses STOP)
            if(opModeIsActive())
            {
                time.reset();
            }

        }
        catch (Exception ex){
            telemetry.addData("Init Error", ex.getMessage());
            telemetry.update();
        }
        finally {
            if (tfDetector != null){
                tfDetector.stopProcessing();
            }
        }
    }
}