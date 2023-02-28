package frc.robot.wrappers;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;

public class DriverCamera {

    // private final UsbCamera camera;
    // private final MjpegServer server;
    // private final CameraServer server;

    public DriverCamera(String name, int port){
        // camera = new UsbCamera(name, port);
        // server = new MjpegServer(name, 1181);
        // server.setSource(camera);
        // camera.setResolution(320, 240);
        // camera.setFPS(15);
        CameraServer.startAutomaticCapture();
        // server.setQuality(50);
        // server.startAutomaticCapture("cam0");
        
    }
}
