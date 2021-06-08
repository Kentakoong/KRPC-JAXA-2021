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

import java.util.ArrayList;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    String QRlog = "QR_STATUS";
    String ARlog = "AR_STATUS";
    int QRLC = 0;
    int ARLC = 0;
    int LM = 5;
    int AR_LM = 2;
    com.google.zxing.Result qr = null;
    String QR_str = null;
    String pattern_raw = null;
    String pos_x_raw = null;
    String pos_y_raw = null;
    String pos_z_raw = null;
    Mat ids_global;
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
        int pattern = Integer.parseInt(pattern_raw);
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
        }else if (pattern == 2) {
            moveToWrapper(pos_x, pos_y, pos_z - 0.46, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");/*
            do {
                try {
                    detectAR();
                } catch (JSONException e) {
                    Log.e(ARlog, "AR_NOT_DETECTED");
                }
                ARLC++;
            }while (ids_global.rows() != 2 && ARLC < AR_LM); */
            moveToWrapper(10.6,pos_y,pos_z,0, 0, -0.707, 0.707);
            moveToWrapper(10.6,pos_y,4.5,0, 0, -0.707, 0.707);
            moveToWrapper(10.6,-8,4.5,0, 0, -0.707, 0.707);
        } else if (pattern == 3) {
            moveToWrapper(pos_x, pos_y, pos_z - 0.41, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            do {
                try {
                    detectAR();
                } catch (JSONException e) {
                    Log.e(ARlog, "AR_NOT_DETECTED");
                }
                ARLC++;
            }while (ids_global.rows() != 2 && ARLC < AR_LM);
        } else if (pattern == 4) {
            moveToWrapper(pos_x, pos_y, pos_z - 0.45, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            do {
                try {
                    detectAR();
                } catch (JSONException e) {
                    Log.e(ARlog, "AR_NOT_DETECTED");
                }
                ARLC++;
            }while (ids_global.rows() != 2 && ARLC < AR_LM);
        } else if (pattern == 5) {
            double x_kiz_left = pos_x - 0.35;
            moveToWrapper(x_kiz_left, pos_y, pos_z - 0.68, 0, 0, -0.707, 0.707);
            moveToWrapper(x_kiz_left, pos_y, pos_z, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            do {
                try {
                    detectAR();
                } catch (JSONException e) {
                    Log.e(ARlog, "AR_NOT_DETECTED");
                }
                ARLC++;
            }while (ids_global.rows() != 2 && ARLC < AR_LM);
        } else if (pattern == 6) {
            double x_kiz_left = pos_x - 0.35;
            moveToWrapper(x_kiz_left, pos_y, pos_z - 0.64, 0, 0, -0.707, 0.707);
            moveToWrapper(x_kiz_left, pos_y, pos_z, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            do {
                try {
                    detectAR();
                } catch (JSONException e) {
                    Log.e(ARlog, "AR_NOT_DETECTED");
                }
                ARLC++;
            }while (ids_global.rows() != 2 && ARLC < AR_LM);
        } else if (pattern == 7) {
            double x_kiz_right = pos_x + 0.22;
            moveToWrapper(pos_x + 0.22, pos_y, pos_z - 0.78, 0, 0, -0.707, 0.707);
            moveToWrapper(x_kiz_right, pos_y, pos_z - 0.78, 0, 0, -0.707, 0.707);
            moveToWrapper(x_kiz_right, pos_y, pos_z, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            do {
                try {
                    detectAR();
                } catch (JSONException e) {
                    Log.e(ARlog, "AR_NOT_DETECTED");
                }
                ARLC++;
            }while (ids_global.rows() != 2 && ARLC < AR_LM);
        } else if (pattern == 8) {
            moveToWrapper(pos_x, pos_y, pos_z - 0.43, 0, 0, -0.707, 0.707);
            moveToWrapper(pos_x, pos_y, pos_z, 0, 0, -0.707, 0.707);
            Log.i("STATUS : ", "MOVED TO A_prime");
            do {
                try {
                    detectAR();
                } catch (JSONException e) {
                    Log.e(ARlog, "AR_NOT_DETECTED");
                }
                ARLC++;
            }while (ids_global.rows() != 2 && ARLC < AR_LM);
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
            LM++;
        }
    }
    private void detectQR(){
        Bitmap bMap = api.getBitmapNavCam();
        Bitmap c_bMap = Bitmap.createBitmap(bMap,520,560,200,200);
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
        int AR_dtLC = 0;
        Mat AR_ID = new Mat();
        Mat mMat = api.getMatNavCam();
        Dictionary AR_DICT = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        DetectorParameters parameters = DetectorParameters.create();
        parameters.set_minMarkerDistanceRate(0.05f);
        parameters.set_minMarkerPerimeterRate(0.05d);
        float AR_mkL = 0.05f;
        JSONArray JsonObj = new JSONArray();
        List<Mat> rj = new ArrayList<>();
        List<Mat> AR_CN = new ArrayList<>();
        Mat AR_rVec = new Mat();
        Mat AR_tVec = new Mat();
        int uds_row = 1280; int uds_col = 960;
        Mat AR_cMT = new Mat(3, 3, CvType.CV_32FC1);
        Mat AR_dC = new Mat(1, 5, CvType.CV_32FC1);
        double cMT_value[] =
                {
                        567.229305, 0.0, 659.077221,
                        0.0, 574.192915, 517.007571,
                        0.0, 0.0, 1.0
                };
        double dC_value[] = {-0.216247, 0.03875, -0.010157, 0.001969, 0.0};
        AR_cMT.put(uds_row, uds_col, cMT_value);
        AR_dC.put(uds_row, uds_col, dC_value);
        do {
            Aruco.detectMarkers(mMat, AR_DICT, AR_CN, AR_ID, parameters, rj);
            Aruco.estimatePoseSingleMarkers(AR_CN, AR_mkL, AR_cMT, AR_dC, AR_rVec, AR_tVec);
            JsonObj.put(Integer.parseInt(String.valueOf(AR_ID.get(AR_dtLC,0)[0])), String.valueOf(AR_CN.get(AR_dtLC).dump()));
            Log.i(ARlog, String.valueOf((int) AR_ID.get(AR_dtLC, 0)[0]));
            Log.i(ARlog, String.valueOf(AR_CN.get(0).get(0, AR_dtLC)[0]));
            AR_dtLC++;
        }while(AR_ID.rows() != 2 && AR_dtLC < AR_LM);
        ids_global = AR_ID;
    }
}
