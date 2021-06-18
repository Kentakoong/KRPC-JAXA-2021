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
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.calib3d.Calib3d;

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    final String QRlog = "QR_STATUS";
    final String ARlog = "AR_STATUS";
    int QRLC = 0;
    final int LM = 5;
    final int AR_LM = 5;
    com.google.zxing.Result qr = null;
    String QR_str = null;
    String pattern_raw = null;
    String pos_x_raw = null;
    String pos_y_raw = null;
    String pos_z_raw = null;
    int pattern;
    double euler_x;
    double euler_y;
    double euler_z;
    @Override
    protected void runPlan1() {
        api.startMission();

        moveToWrapper(11.3277, -9.8422, 4.8726, 0, 0, -0.707, 0.707);

        do{
            detectQR();
            QRLC++;
        }
        while (qr == null && QRLC < LM);

        QR_str = String.valueOf(qr);

        Log.i(QRlog, QR_str);
        api.sendDiscoveredQR(QR_str);

        try{
            JSONObject split = new JSONObject(QR_str);
            pattern_raw = split.getString("p");
            pos_x_raw = split.getString("x");
            pos_y_raw = split.getString("y");
            pos_z_raw = split.getString("z");
        } catch (JSONException e) {
            Log.e(QRlog, "QR_NOT_CONVERTED");
        }
        pattern = Integer.parseInt(pattern_raw);
        Log.i(QRlog,"PATTERN : " + pattern);
        double pos_x = Double.parseDouble(pos_x_raw);
        Log.i(QRlog,"POS_X : " + pos_x);
        double pos_y = Double.parseDouble(pos_y_raw);
        Log.i(QRlog,"POS_Y : " + pos_y);
        double pos_z = Double.parseDouble(pos_z_raw);
        Log.i(QRlog,"POS_Z : " + pos_z);

        if (pattern == 1) {
            moveToWrapper(pos_x, pos_y, pos_z - 0.46, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            try {
                detectAR();
            } catch (JSONException e) {
                e.printStackTrace();
                Log.e(ARlog,"AR_FAILED");
            }
            moveToEuler(pos_x, pos_y, pos_z, euler_x, euler_y, euler_z);
            api.laserControl(true);
            api.takeSnapshot();
            api.laserControl(false);
            moveToWrapper(pos_x, pos_y, pos_z-0.1, 0, 0, -0.707, 0.707);
            moveToWrapper(10.6,pos_y,4.5,0, 0, -0.707, 0.707);
            moveToWrapper(10.6,-8,4.5,0, 0, -0.707, 0.707);
        }else if (pattern == 2) {
            moveToWrapper(pos_x, pos_y, pos_z - 0.46, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            try {
                detectAR();
            } catch (JSONException e) {
                e.printStackTrace();
            }
            api.laserControl(true);
            api.takeSnapshot();
            api.laserControl(false);
            moveToWrapper(10.6,pos_y,pos_z,0, 0, -0.707, 0.707);
            moveToWrapper(10.6,-8,4.5,0, 0, -0.707, 0.707);
        } else if (pattern == 3) {
            moveToWrapper(pos_x, pos_y, pos_z - 0.41, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            try {
                detectAR();
            } catch (JSONException e) {
                e.printStackTrace();
            }
            api.laserControl(true);
            api.takeSnapshot();
            api.laserControl(false);
            moveToWrapper(10.6,pos_y,pos_z,0, 0, -0.707, 0.707);
            moveToWrapper(10.6,-8,4.5,0, 0, -0.707, 0.707);
        } else if (pattern == 4) {
            moveToWrapper(pos_x, pos_y, pos_z - 0.45, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            try {
                detectAR();
            } catch (JSONException e) {
                e.printStackTrace();
            }
            api.laserControl(true);
            api.takeSnapshot();
            api.laserControl(false);
            moveToWrapper(10.6,pos_y,4.5,0, 0, -0.707, 0.707);
            moveToWrapper(10.6,-8,4.5,0, 0, -0.707, 0.707);
        } else if (pattern == 5) {
            double x_kiz_left = pos_x - 0.35;
            moveToWrapper(x_kiz_left, pos_y, pos_z - 0.68, 0, 0, -0.707, 0.707);
            moveToWrapper(x_kiz_left, pos_y, pos_z, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            try {
                detectAR();
            } catch (JSONException e) {
                e.printStackTrace();
            }
            api.laserControl(true);
            api.takeSnapshot();
            api.laserControl(false);
            moveToWrapper(10.6,pos_y,pos_z,0, 0, -0.707, 0.707);
            moveToWrapper(10.6,-8,4.5,0, 0, -0.707, 0.707);
        } else if (pattern == 6) {
            double x_kiz_left = pos_x - 0.35;
            moveToWrapper(x_kiz_left, pos_y, pos_z - 0.64, 0, 0, -0.707, 0.707);
            moveToWrapper(x_kiz_left, pos_y, pos_z, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            try {
                detectAR();
            } catch (JSONException e) {
                e.printStackTrace();
            }
            api.laserControl(true);
            api.takeSnapshot();
            api.laserControl(false);
            moveToWrapper(10.6,pos_y,pos_z,0, 0, -0.707, 0.707);
            moveToWrapper(10.6,-8,4.5,0, 0, -0.707, 0.707);
        } else if (pattern == 7) {
            double x_kiz_right = pos_x + 0.22;
            moveToWrapper(pos_x + 0.22, pos_y, pos_z - 0.76, 0, 0, -0.707, 0.707);
            moveToWrapper(x_kiz_right, pos_y, pos_z - 0.76, 0, 0, -0.707, 0.707);
            moveToWrapper(x_kiz_right, pos_y, pos_z, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            try {
                detectAR();
            } catch (JSONException e) {
                e.printStackTrace();
            }
            api.laserControl(true);
            api.takeSnapshot();
            api.laserControl(false);
            moveToWrapper(x_kiz_right, pos_y, pos_z, 0, 0, -0.707, 0.707);
            moveToWrapper(x_kiz_right, pos_y, pos_z - 0.76, 0, 0, -0.707, 0.707);
            moveToWrapper(10.6,pos_y,4.5,0, 0, -0.707, 0.707);
            moveToWrapper(10.6,-8,4.5,0, 0, -0.707, 0.707);
        } else if (pattern == 8) {
            moveToWrapper(pos_x, pos_y, pos_z - 0.43, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            try {
                detectAR();
            } catch (JSONException e) {
                e.printStackTrace();
            }
            api.laserControl(true);
            api.takeSnapshot();
            api.laserControl(false);
            moveToWrapper(pos_x, pos_y, pos_z-0.4, 0, 0, -0.707, 0.707);
            moveToWrapper(10.6,pos_y,4.5,0, 0, -0.707, 0.707);
            moveToWrapper(10.6,-8,4.5,0, 0, -0.707, 0.707);
        }


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

    private void moveToWrapper(double pos_x, double pos_y, double pos_z, double qua_x, double qua_y, double qua_z, double qua_w) {
        int MVLC = 0;
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y, (float) qua_z, (float) qua_w);

        Result result = api.moveTo(point, quaternion, true);

        while (!result.hasSucceeded() && MVLC < LM){
            result = api.moveTo(point, quaternion, true);
            MVLC++;
        }
    }

    private void moveToQUA(double qua_x, double qua_y, double qua_z, double qua_w) {
        int MVLC = 0;
        final Point goalpoint = new Point(0, 0, 0);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y, (float) qua_z, (float) qua_w);

        Result result = api.relativeMoveTo(goalpoint, quaternion, true);

        while (!result.hasSucceeded() && MVLC < LM){
            result = api.relativeMoveTo(goalpoint, quaternion, true);
            MVLC++;
        }
    }

    private void moveToEuler(double x_org, double y_org, double z_org, double x_des, double y_des, double z_des) {
        double dx = x_des - x_org;
        double dy = y_des - y_org;
        double dz = z_des - z_org;
        double magnitude = Math.sqrt((dx * dx) + (dy * dy) + (dz * dz));
        double x_unit = dx / magnitude;
        double y_unit = dy / magnitude;
        double z_unit = dz / magnitude;

        double[][] matrix =
                {
                        {1, 0, 0},
                        {x_unit, y_unit, z_unit}
                };

        double x = matrix[0][1] * matrix[1][2] - matrix[1][1] * matrix[0][2];
        double y = matrix[0][2] * matrix[1][0] - matrix[1][2] * matrix[0][0];
        double z = matrix[0][0] * matrix[1][1] - matrix[1][0] * matrix[0][1];
        double i = matrix[1][0] - matrix[0][0];
        double j = matrix[1][1] - matrix[0][1];
        double k = matrix[1][2] - matrix[0][2];
        double q = Math.sqrt(x * x + y * y + z * z);
        double p = Math.sqrt(i * i + j * j + k * k);
        double theta = Math.acos((2 - p * p) / 2);

        double a = Math.sin(theta / 2) * x / q;
        double b = Math.sin(theta / 2) * y / q;
        double c = Math.sin(theta / 2) * z / q;
        double w = Math.cos(theta / 2);

        double pitch = -Math.atan((2 * (a * w + b * c)) / (w * w - a * a - b * b + c * c));
        double roll = -Math.asin(2 * (a * c - b * w));
        double yaw = Math.atan((2 * (c * w + a * b)) / (w * w + a * a - b * b - c * c));
        double sx = (0.103 * Math.cos(roll + 0.279) / Math.cos(1.57080 + yaw));
        double sy = (0.103 * Math.sin(roll + 0.279) / Math.cos(pitch));

        moveToWrapper((float) x_org - (float) sx, (float) y_org, (float) z_org + (float) sy, (float) a, (float) b, (float) c, (float) w);
    }

    private void detectQR(){
        Bitmap bMap = api.getBitmapNavCam();
        Bitmap c_bMap = Bitmap.createBitmap(bMap,510,550,220,220);
        int[] size_bMap = new int[c_bMap.getWidth()*c_bMap.getHeight()];
        c_bMap.getPixels(size_bMap,0,c_bMap.getWidth(),0,0,c_bMap.getWidth(),c_bMap.getHeight());
        try{
            LuminanceSource lms = new RGBLuminanceSource(c_bMap.getWidth(),c_bMap.getHeight(),size_bMap);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(lms));
            qr = new QRCodeReader().decode(bitmap);
        }catch (Exception e){
            Log.e(QRlog, "QR_NOT_DETECTED");
        }
        assert qr != null;
    }

    private void detectAR() throws JSONException {
        int IDLC = 0;
        Mat img = api.getMatNavCam();
        Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();
        DetectorParameters detectparam = DetectorParameters.create();
        List<Mat> reject = new ArrayList<>();
        float markerLength = 0.05f;
        Mat rVecs = new Mat();
        Mat tVecs = new Mat();
        JSONObject jsonRPY = new JSONObject();
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
        final double cMT_value[] =
                {
                        567.229305, 0.0, 659.077221,
                        0.0, 574.192915, 517.007571,
                        0.0, 0.0, 1.0
                };
        final double dC_value[] = {-0.216247, 0.03875, -0.010157, 0.001969, 0.0};
        cameraMatrix.put(0, 0, cMT_value);
        distCoeffs.put(0, 0, dC_value);
        detectparam.set_minMarkerDistanceRate(0.05f);
        detectparam.set_minMarkerPerimeterRate(0.05d);
        detectparam.set_maxMarkerPerimeterRate(0.5d);
        detectparam.set_errorCorrectionRate(0.001d);
        Aruco.detectMarkers(img, dict, corners, ids, detectparam, reject);
        while (ids.rows() != 4 && IDLC < AR_LM){
            Aruco.detectMarkers(img, dict, corners, ids, detectparam, reject);
            IDLC++;
        }
        Log.i(ARlog,String.valueOf(ids.get(0, 0)[0]));
        Log.i(ARlog,String.valueOf(ids.get(1, 0)[0]));
        Log.i(ARlog,String.valueOf(ids.get(2, 0)[0]));
        Log.i(ARlog,String.valueOf(ids.get(3, 0)[0]));
        Log.i(ARlog, "AR_DETECTED");
        for (int i = 0; i < 4; ++i) {
        Aruco.estimatePoseSingleMarkers(corners, 0.05f, cameraMatrix, distCoeffs, rVecs, tVecs);
        Mat rot = new Mat();
        Mat mtxR = new Mat();
        Mat mtxQ = new Mat();
        Calib3d.Rodrigues(rVecs.row(i), rot);
        double[] eulerAngle = Calib3d.RQDecomp3x3(rot, mtxR, mtxQ);
        float pitch = (float) eulerAngle[0];
        float yaw = (float) -eulerAngle[1];
        float roll = (float) -eulerAngle[2];
        jsonRPY.put(String.valueOf(ids.get(i, 0)[0]), yaw + "," + pitch + "," + roll);
        Log.i(ARlog, String.valueOf(jsonRPY));
        }
        JSONObject sepValue = new JSONObject(String.valueOf(jsonRPY));

        String ID1 = sepValue.getString("1.0");
        String[] splitComma_1 = ID1.split(",");
        double yaw_1 = Double.parseDouble(splitComma_1[0]);
        double pitch_1 = Double.parseDouble(splitComma_1[1]);
        double roll_1 = Double.parseDouble(splitComma_1[2]);

        String ID2 = sepValue.getString("2.0");
        String[] splitComma_2 = ID1.split(",");
        double roll_2 = Double.parseDouble(splitComma_2[0]);
        double pitch_2 = Double.parseDouble(splitComma_2[1]);
        double yaw_2 = Double.parseDouble(splitComma_2[2]);

        String ID3 = sepValue.getString("3.0");
        String[] splitComma_3 = ID1.split(",");
        double roll_3 = Double.parseDouble(splitComma_3[0]);
        double pitch_3 = Double.parseDouble(splitComma_3[1]);
        double yaw_3 = Double.parseDouble(splitComma_3[2]);

        String ID4 = sepValue.getString("4.0");
        String[] splitComma_4 = ID1.split(",");
        double roll_4 = Double.parseDouble(splitComma_4[0]);
        double pitch_4 = Double.parseDouble(splitComma_4[1]);
        double yaw_4 = Double.parseDouble(splitComma_4[2]);

        if(pattern == 1){
            if(-Math.abs(pitch_4) <= -170){
                euler_x = -Math.abs(roll_4);
                euler_y = -Math.abs(pitch_4);
                euler_z = Math.abs(yaw_4);
            }
            else{
                euler_x = -Math.abs(roll_1);
                euler_y = -Math.abs(pitch_1);
                euler_z = Math.abs(yaw_1);
            }
        }
    }
}
