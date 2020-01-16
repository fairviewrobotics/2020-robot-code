package frc.robot.vision

import org.opencv.core.Mat
import org.opencv.imgproc.Imgproc

import edu.wpi.cscore.CvSource
import edu.wpi.first.wpilibj.CameraServer
import org.opencv.videoio.Videoio
import frc.robot.vision.HighGoalVision

// OPENCV
import org.opencv.videoio.VideoCapture


import org.opencv.core.*

class Streaming: Thread() {


    lateinit var frontCamera: VideoCapture

    lateinit var frontCameraStream: CvSource
    lateinit var frontImageMat: Mat

    init {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME)
         CameraServer.getInstance().startAutomaticCapture()

        this.frontCamera = VideoCapture(0)
        this.frontCamera.set(Videoio.CAP_PROP_EXPOSURE, 10.0)
        this.frontCamera.set(Videoio.CAP_PROP_BRIGHTNESS, 1.0)

        if(!this.frontCamera.isOpened()){
            println("Failed to Open Camera")
        }
        else {
            println("Opened Camera\n");
        }

        this.frontImageMat = Mat()
    }


    fun stream(){
        if (this.frontCamera.isOpened()) {
            this.frontCamera.read(frontImageMat)
            Imgproc.resize(frontImageMat, frontImageMat, Size(320.0, 200.0))
            val targets = HighGoalVision.highGoalDetect(frontImageMat, Size(160.0, 120.0), Scalar(53.0, 213.0, 100.0), Scalar(100.0, 255.0, 255.0), 1, 5, 0.001, Math.toRadians(68.5), 16.0, 9.0)
            //Imgproc.cvtColor(frontImageMat, frontImageMat, Imgproc.COLOR_BGR2GRAY)
            println(targets.size)
            //frontCameraStream.putFrame(frontImageMat)
        } else {
            //println("Not open")
        }
    }

    override fun run(){
        while(true){
            this.stream()
        }
    }

}