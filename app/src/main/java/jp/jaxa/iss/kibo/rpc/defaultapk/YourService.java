package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.util.Log;


import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    final int LOOP_MAX = 50;
    int qrloopCounter = 0;
    int arloopCounter = 0;
    String p_raw = null;
    String x_raw = null;
    String y_raw = null;
    String z_raw = null;
    String qrcheck = null;

    @Override
    protected void runPlan1() {
        api.startMission();

        moveToWrapper(11.3277, -9.8422, 4.8726, 0, 0, -0.707, 0.707);

        do {
            qrcheck = detectQR();
            qrloopCounter++;
        }
        while (qrcheck == null && qrloopCounter < LOOP_MAX);

        Log.i("QR_DATA", qrcheck);

        api.sendDiscoveredQR(qrcheck);


        Log.i("KINEMATICS : ", "POS : " + api.getTrustedRobotKinematics());

        try {
            JSONObject jsonObj = new JSONObject(qrcheck);
            p_raw = jsonObj.getString("p");
            x_raw = jsonObj.getString("x");
            y_raw = jsonObj.getString("y");
            z_raw = jsonObj.getString("z");
        } catch (Exception e) {
            Log.e("STATUS : ", "JSONerror");
        }

        int pattern = Integer.parseInt(p_raw);
        double pos_x = Double.parseDouble(x_raw);
        double pos_y = Double.parseDouble(y_raw);
        double pos_z = Double.parseDouble(z_raw);

        if (pattern == 1 || pattern == 2) {
            moveToWrapper(pos_x, pos_y, pos_z - 0.46, 0, 0, -0.707, 0.707);
            Log.i("KINEMATICS : ", "POS1 : " + api.getTrustedRobotKinematics());
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
        } else if (pattern == 3) {
            moveToWrapper(pos_x, pos_y, pos_z - 0.41, 0, 0, -0.707, 0.707);
            Log.i("KINEMATICS : ", "POS1 : " + api.getTrustedRobotKinematics());
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
        } else if (pattern == 4) {
            moveToWrapper(pos_x, pos_y, pos_z - 0.45, 0, 0, -0.707, 0.707);
            Log.i("KINEMATICS : ", "POS1 : " + api.getTrustedRobotKinematics());
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
        } else if (pattern == 5) {
            double x_kiz_left = pos_x - 0.35;
            moveToWrapper(x_kiz_left, pos_y, pos_z - 0.68, 0, 0, -0.707, 0.707);
            Log.i("KINEMATICS : ", "POS1 : " + api.getTrustedRobotKinematics());
            moveToWrapper(x_kiz_left, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("KINEMATICS : ", "POS2 : " + api.getTrustedRobotKinematics());
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
        } else if (pattern == 6) {
            double x_kiz_left = pos_x - 0.35;
            moveToWrapper(x_kiz_left, pos_y, pos_z - 0.64, 0, 0, -0.707, 0.707);
            Log.i("KINEMATICS : ", "POS1 : " + api.getTrustedRobotKinematics());
            moveToWrapper(x_kiz_left, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("KINEMATICS : ", "POS2 : " + api.getTrustedRobotKinematics());
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
        } else if (pattern == 7) {
            double x_kiz_right = pos_x + 0.19;
            moveToWrapper(pos_x + 0.0277, pos_y, pos_z - 0.78, 0, 0, -0.707, 0.707);
            Log.i("KINEMATICS : ", "POS1 : " + api.getTrustedRobotKinematics());
            moveToWrapper(x_kiz_right, pos_y, pos_z - 0.78, 0, 0, -0.707, 0.707);
            Log.i("KINEMATICS : ", "POS2 : " + api.getTrustedRobotKinematics());
            moveToWrapper(x_kiz_right, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("KINEMATICS : ", "POS3 : " + api.getTrustedRobotKinematics());
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
        } else if (pattern == 8) {
            moveToWrapper(pos_x, pos_y, pos_z - 0.43, 0, 0, -0.707, 0.707);
            Log.i("KINEMATICS : ", "POS1 : " + api.getTrustedRobotKinematics());
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
        }


        Log.i("KINEMATICS : ", "POS_FINISHAR : " + api.getTrustedRobotKinematics());


        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }

    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w) {

        final int LOOP_MAX = 5;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);

        Result result = api.moveTo(point, quaternion, true);

        int loopCounter = 0;
        while (!result.hasSucceeded() && loopCounter < LOOP_MAX) {
            result = api.moveTo(point, quaternion, true);
            ++loopCounter;
        }
    }

    private String detectQR() {
        String QRtag = "QR_STATUS";
        Bitmap bMap = api.getBitmapNavCam();
        String pos_x;
        com.google.zxing.Result qrcode = null;
        Bitmap crop_bMap = Bitmap.createBitmap(bMap, 520, 560, 200, 200);
        Log.i(QRtag, "CROPPED_QR");
        int[] intArray = new int[crop_bMap.getWidth() * crop_bMap.getHeight()];
        crop_bMap.getPixels(intArray, 0, crop_bMap.getWidth(), 0, 0, crop_bMap.getWidth(), crop_bMap.getHeight());
        try {
            LuminanceSource source = new RGBLuminanceSource(crop_bMap.getWidth(), crop_bMap.getHeight(), intArray);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
            qrcode = new QRCodeReader().decode(bitmap);
            Log.i(QRtag, "QR_DETECTED");
        } catch (Exception e) {
            Log.e(QRtag, "QR_NOT_DETECTED");
            Log.e("QR_LOG_ERROR", "ERROR : " + e);
        }

        assert qrcode != null;
        pos_x = qrcode.getText();
        return pos_x;
    }
}

