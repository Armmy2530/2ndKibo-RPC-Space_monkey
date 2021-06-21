package jp.jaxa.iss.kibo.rpc.defaultapk;

import android.graphics.Bitmap;
import android.os.SystemClock;
import android.util.Log;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.api.types.PointCloud;

import static org.opencv.android.Utils.matToBitmap;

/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */

public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1() {
        int max_count = 3;
        String mode = "sim";
        api.startMission();
        moveTo(11.21, -9.8, 4.79, 0, 0, -0.707, 0.707);
        // at point A
        // qr part
        api.flashlightControlFront(0.55F);
        wait(21000);
        Mat src = api.getMatNavCam();
        double[] qr_content = QR_event(src,mode, max_count,0.76); // 0.66
        int koz_pattern = (int) qr_content[0];
        float x_ap = (float) qr_content[1];
        float y_ap = (float) qr_content[2];
        float z_ap = (float) qr_content[3];
        Log.d("QR[qr_content]", "koz:" + koz_pattern + " x:" + x_ap + " y:" + y_ap + " z:" + z_ap);
        // ar part
        double[] astrobee_positon = {x_ap,y_ap,z_ap};
        double[] target_position = AR_findtarget(mode,src,11.21f,4.79f);
        target_pointX = target_position.clone();
        float rotatequternion[] = RotatetoTarget(target_position, astrobee_positon, 0.82);
        Log.d("AR[status]:", "final target position: " + Arrays.toString(target_position));
        Log.d("AR[status]:", "final target point: " + Arrays.toString(target_pointX));
        // at point A'
        moveToA(x_ap, y_ap, z_ap, rotatequternion[0],rotatequternion[1],rotatequternion[2],rotatequternion[3], koz_pattern, (float) target_pointX[0], (float) target_pointX[1]);
        api.laserControl(true);
        api.takeSnapshot();
        api.laserControl(false);
        // at point B
        moveToB(astrobee_positon[0],astrobee_positon[1],astrobee_positon[2],koz_pattern,target_position);
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

    public double[] target_pointX = new double[3];

    public void moveToB(double ap_x, double ap_y, double ap_z,int pattern,double target_point[]) {
        float offset = 0.04F;
        double wall_position[] = {10.6,-8.8,4.5};
        double pointB_position[] = {10.6,-8.0,4.5};
        double qz1=-0.707,qw1=0.707;
        double qz2= 0.707, qw2=0.707;
        Log.d("Moveto A[status]:", "start moving to point A");
        if (pattern == 1 ) {
            moveTo(target_point[0]-0.075+offset, -9.8, target_point[1]-0.3-offset,0,0, qz1, qw1);
            moveTo(wall_position[0],wall_position[1],wall_position[2], 0,0, qz2, qw2);
            moveTo(pointB_position[0],pointB_position[1],pointB_position[2], 0,0, qz1, qw1);
        }
        else if (pattern == 8){
            moveTo(target_point[0]+offset, -9.8, target_point[1]-0.3-offset,0,0, qz1, qw1);
            moveTo(wall_position[0],wall_position[1],wall_position[2], 0,0, qz2, qw2);
            moveTo(pointB_position[0],pointB_position[1],pointB_position[2], 0,0, qz1, qw1);
        }
        else if ( pattern == 2 || pattern == 3 || pattern == 4) {
            moveTo(wall_position[0],wall_position[1],wall_position[2], 0,0, qz2, qw2);
            moveTo(pointB_position[0],pointB_position[1],pointB_position[2], 0,0, qz1, qw1);
        }
        else if (pattern == 5) {
            moveTo(target_point[0]-0.3-offset, -9.8f, target_point[1]-0.075+offset, 0, 0, qz1, qw1);
            moveTo(wall_position[0],wall_position[1],wall_position[2], 0,0, qz2, qw2);
            moveTo(pointB_position[0],pointB_position[1],pointB_position[2], 0,0, qz1, qw1);
        }
        else if (pattern == 6) {
            moveTo(target_point[0]-0.3-offset, -9.8f, target_point[1]+offset, 0, 0, qz1, qw1);
            moveTo(wall_position[0],wall_position[1],wall_position[2], 0,0, qz2, qw2);
            moveTo(pointB_position[0],pointB_position[1],pointB_position[2], 0,0, qz1, qw1);
        }
        else if (pattern == 7) {
            moveTo(target_point[0]+0.3+offset, -9.8f, target_point[1]-0.075+offset, 0, 0, qz1, qw1);
            moveTo(target_point[0]+0.3+offset, -9.8f, target_point[1]-0.3-offset, 0, 0, qz1, qw1);
            moveTo(wall_position[0],wall_position[1],wall_position[2], 0,0, qz2, qw2);
            moveTo(pointB_position[0],pointB_position[1],pointB_position[2], 0,0, qz1, qw1);
        }
        else {
            Log.d("Moveto A[status]:", "pattern is unknowed");
        }
        Log.d("Moveto A[status]:", "done");
        api.reportMissionCompletion();
    }

    public void moveToA(float px, float py, float pz, float qx, float qy, float qz, float qw, int pattern,float target_positionx,float target_positiony) {
        float offset = 0.04F;
        Log.d("Moveto A[status]:", "start moving to point A");
        if (pattern == 1 || pattern == 2 || pattern == 8) {
            moveTo(px, py, pz, qx, qy, qz, qw);
        } else if (pattern == 3 || pattern == 4) {
            moveTo(px, py, pz, qx, qy, qz, qw);
        } else if (pattern == 5) {
            Log.d("Moveto A[status]:", "step 1[5] start");
            moveTo(target_positionx-0.3-offset, -9.8f, target_positiony-0.3-offset, 0, 0, -0.707, 0.707);
            Log.d("Moveto A[status]:", "step 1[5] done");
            Log.d("Moveto A[status]:", "step 2[5] start");
            moveTo(target_positionx-0.3-offset, -9.8f, target_positiony-0.075+offset, 0, 0, -0.707, 0.707);
            Log.d("Moveto A[status]:", "step 2[5] done");
            moveTo(px, py, pz, qx, qy, qz, qw);
        } else if (pattern == 6) {
            Log.d("Moveto A[status]:", "step 1[6] start");
            moveTo(target_positionx-0.3-offset, -9.8f, target_positiony-0.3-offset, 0, 0, -0.707, 0.707);
            Log.d("Moveto A[status]:", "step 1[6] done");
            Log.d("Moveto A[status]:", "step 2[6] start");
            moveTo(target_positionx-0.3-offset, -9.8f, target_positiony+offset, 0, 0, -0.707, 0.707);
            Log.d("Moveto A[status]:", "step 2[6] done");
            moveTo(px, py, pz, qx, qy, qz, qw);
        } else if (pattern == 7) {
            Log.d("Moveto A[status]:", "step 1[7] start");
            moveTo(target_positionx+0.3+offset, -9.8f, target_positiony-0.3-offset, 0, 0, -0.707, 0.707);
            Log.d("Moveto A[status]:", "step 1[7] done");
            Log.d("Moveto A[status]:", "step 2[7] start");
            moveTo(target_positionx+0.3+offset, -9.8f, target_positiony-0.075+offset, 0, 0, -0.707, 0.707);
            Log.d("Moveto A[status]:", "step 2[7] done");
            moveTo(px, py, pz, qx, qy, qz, qw);
        } else {
            Log.d("Moveto A[status]:", "pattern is unknowed");
        }
        Log.d("Moveto A[status]:", "done");
    }

    public void moveTo(double px, double py, double pz, double qx, double qy, double qz, double qw) {
        Result result;
        int count = 0, max_count = 5;
        Point point = new Point(px, py, pz);
        Quaternion quaternion = new Quaternion((float) qx, (float) qy, (float) qz, (float) qw);
        do {
            result = api.moveTo(point, quaternion, true);
            count++;
        } while (!result.hasSucceeded() && count < max_count);
    }

    public Mat undistordAR(Mat src,String _mode, int x , int y) {
        Size s = src.size();
        int rows = (int) s.height;
        int cols = (int) s.width;
        Mat dst = new Mat(rows, cols, CvType.CV_8UC1);
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
        double cx=0,cy=0,fx=0,fy=0;
        int row = 0, col = 0;
        double offset_x = -29.077221 , offset_y = -17.007571;
        if (_mode == "sim") {
            cx = 659.077221-x+offset_x;
            cy = 517.007571-y+offset_y;
            fx = 567.229305;
            fy = 574.192915;
        }else if (_mode == "iss") {
            cx = 571.399891-x;
            cy = 504.956891-y;
            fx = 692.827528;
            fy = 691.919547;
        }
        double cameraMatrix_ar[] = {
                fx, 0.0D, cx,
                0.0D, fy, cy,
                0.0D, 0.0D, 1.0D
        };
        double distCoeffs_ar[] = {-0.216247D, 0.03875D, -0.010157D, 0.001969D, 0.0D};
        cameraMatrix.put(row, col, cameraMatrix_ar);
        distCoeffs.put(row, col, distCoeffs_ar);
        Log.d("Mode[camera]:", "undistor for ar");
        Imgproc.undistort(src, dst, cameraMatrix, distCoeffs);
        return dst;
    }

    public Mat undistord(Mat src, String _mode) {
        Size s = src.size();
        int rows = (int) s.height;
        int cols = (int) s.width;
        Mat dst = new Mat(rows, cols, CvType.CV_8UC1);
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);
        int row = 0, col = 0;
        double cameraMatrix_sim[] = {
                567.229305D, 0.0D, 659.077221D,
                0.0D, 574.192915D, 517.007571D,
                0.0D, 0.0D, 1.0D
        };
        double distCoeffs_sim[] = {-0.216247D, 0.03875D, -0.010157D, 0.001969D, 0.0D};

        double cameraMatrix_iss[] = {
                692.827528D, 0.0D, 571.399891D,
                0.0D, 691.919547D, 504.956891D,
                0.0D, 0.0D, 1.0D
        };
        double distCoeffs_iss[] = {-0.312191D, 0.073843D, -9.18E-4D, 0.00189D, 0.0D};

        if (_mode == "sim") {
            cameraMatrix.put(row, col, cameraMatrix_sim);
            distCoeffs.put(row, col, distCoeffs_sim);
            Log.d("Mode[camera]:", " sim");
        } else if (_mode == "iss") {
            cameraMatrix.put(row, col, cameraMatrix_iss);
            distCoeffs.put(row, col, distCoeffs_iss);
            Log.d("Mode[camera]:", " iss");
        }
        Imgproc.undistort(src, dst, cameraMatrix, distCoeffs);
        return dst;
    }

    public Mat cropImage(Mat src,double new_x,double new_y,double new_height,double new_width) {
        Mat dst = src.submat((int)new_y,(int)new_y+(int)new_height,(int)new_x,(int)new_x+(int)new_width);
        Size s = dst.size();
        Log.d("QR[status]:", "crop cal new_x:" + new_x + " new_y:" + new_y + " new_widht:" + new_width + " new_height:" + new_height + " mode: custom");
        Log.d("QT[status]", "DST mat size: "+s);
        return dst;
    }
    public Mat cropImage(Mat src,String _mode,double percentage) {
        double nav_max_w = 1280;// 4:3
        double nav_max_h = 960; //--+
        double total = 1228800;
        double totalx = 12;// 4*3
        double multiple = Math.sqrt((total * (percentage / 100)) / totalx);
        double new_width = (4 * multiple);
        double new_height = new_width * (0.75);
        double new_x=0,new_y=0;
        if (_mode == "center") {
            new_x = (nav_max_w / 2) - (new_width / 2);
            new_y = (nav_max_h / 2) - (new_height / 2);
        }
        else if (_mode == "centerbr") {
            new_x = nav_max_w / 2;
            new_y = nav_max_h / 2;
        } else if (_mode == "br") {//bottom right
            new_x = nav_max_w - new_width;
            new_y = nav_max_h - new_height;
        } else if (_mode == "bl") {//bottom right
            new_x = 0;
            new_y = nav_max_h - new_height;
        } else if (_mode == "tl") {//bottom right
            new_x = 0;
            new_y = 0;
        } else if (_mode == "tr") {//bottom right
            new_x = nav_max_w - new_width;
            new_y = 0;
        }
        Mat dst = src.submat((int)new_y,(int)new_y+(int)new_height,(int)new_x,(int)new_x+(int)new_width);
        Size s = dst.size();
        Log.d("QR[status]:", "crop cal new_x:" + new_x + " new_y:" + new_y + " new_widht:" + new_width + " new_height:" + new_height + " mutliple:" + multiple + " mode:" + _mode);
        Log.d("QT[status]", "DST mat size: "+s);
        return dst;
    }

    public float[] RotatetoTarget(double[] target_position, double[] astrobee_position, double disy) {
        String TAG = "RotateTo[status]:";
        Log.d(TAG, "START");
        disy = disy - 0.1302;
        double dfyaw = -(Math.PI / 2);
        double dfpitch = 0;
        double laserpointer_position[]={astrobee_position[0]+0.0572,astrobee_position[2]-0.1111};
        double disx = target_position[0]-laserpointer_position[0];
        double disz = laserpointer_position[1]-target_position[1];
        double addyaw = Math.atan(disx / disy);
        double addpitch = Math.atan(disz / disy);
        double finalyaw = addyaw + dfyaw;
        double finalpitch = addpitch + dfpitch;
        Log.d(TAG, "Delta x:" + disx + " Delta z:" + disz);
        Log.d(TAG, "Euler angle: Pitch:" + finalpitch + " Roll: 0" + " Yaw: " + finalyaw);
        Log.d(TAG, "Euler angle: Pitch:" + Math.toDegrees(finalpitch) + " Roll: 0" + " Yaw: " + Math.toDegrees(finalyaw));
        float[] finalquternion = EulertoQuaternion(finalpitch, 0, finalyaw);
        Log.d(TAG, "Quaternion:" + Arrays.toString(finalquternion));// x  y  z  w
        moveTo(astrobee_position[0], astrobee_position[1], astrobee_position[2], finalquternion[0], finalquternion[1], finalquternion[2], finalquternion[3]);
        Log.d(TAG, "Rotated successful");
        return finalquternion;
    }

    public float[] EulertoQuaternion(double pitch, double roll, double yaw) {
        double cy = Math.cos(yaw * 0.5);
        double sy = Math.sin(yaw * 0.5);
        double cp = Math.cos(pitch * 0.5);
        double sp = Math.sin(pitch * 0.5);
        double cr = Math.cos(roll * 0.5);
        double sr = Math.sin(roll * 0.5);
        double w = cr * cp * cy + sr * sp * sy;
        double x = sr * cp * cy - cr * sp * sy;
        double y = cr * sp * cy + sr * cp * sy;
        double z = cr * cp * sy - sr * sp * cy;
        return new float[]{(float)x,(float)y,(float)z, (float)w};
    }

    public double[] AR_findtarget(String _mode,Mat _src,float x_ap, float z_ap) {
        double[][] ar_info = AR_event(_src,_mode, 1);
        double pretarget_p[][] = new double[4][3];
        double target_p[] = new double[3];
        int i = 0;
        while (ar_info[4][0] == 0) {
            ar_info = AR_event(_src,_mode, 1);
        }
        switch ((int) ar_info[4][0]) {
            case 1:
                while (ar_info[i][0] == 0 && ar_info[i][1] == 0) {
                    i++;
                }
                switch (i) {
                    case 0:
                        pretarget_p[0][0] = x_ap - 0.0422 + ar_info[0][0] - 0.1125;// distance between ar and target in x
                        pretarget_p[0][1] = z_ap - 0.0826 + ar_info[0][1] + 0.0415;// in y
                        break;
                    case 1:
                        pretarget_p[0][0] = x_ap - 0.0422 + ar_info[1][0] + 0.1125;
                        pretarget_p[0][1] = z_ap - 0.0826 + ar_info[1][1] + 0.0415;
                        break;
                    case 2:
                        pretarget_p[0][0] = x_ap - 0.0422 + ar_info[2][0] + 0.1125;
                        pretarget_p[0][1] = z_ap - 0.0826 + ar_info[2][1] - 0.0415;
                        break;
                    case 3:
                        pretarget_p[0][0] = x_ap - 0.0422 + ar_info[3][0] - 0.1125;
                        pretarget_p[0][1] = z_ap - 0.0826 + ar_info[3][1] - 0.0415;
                        break;
                }
                break;
            case 2:
                for (int x = 0; x <= 1; x++) {
                    while (ar_info[i][0] == 0 && ar_info[i][1] == 0) {
                        i++;
                    }
                    switch (i) {
                        case 0:
                            pretarget_p[x][0] = x_ap - 0.0422 + ar_info[0][0] - 0.1125;// distance between ar and target in x
                            pretarget_p[x][1] = z_ap - 0.0826 + ar_info[0][1] + 0.0415;// in y
                            break;
                        case 1:
                            pretarget_p[x][0] = x_ap - 0.0422 + ar_info[1][0] + 0.1125;
                            pretarget_p[x][1] = z_ap - 0.0826 + ar_info[1][1] + 0.0415;
                            break;
                        case 2:
                            pretarget_p[x][0] = x_ap - 0.0422 + ar_info[2][0] + 0.1125;
                            pretarget_p[x][1] = z_ap - 0.0826 + ar_info[2][1] - 0.0415;
                            break;
                        case 3:
                            pretarget_p[x][0] = x_ap - 0.0422 + ar_info[3][0] - 0.1125;
                            pretarget_p[x][1] = z_ap - 0.0826 + ar_info[3][1] - 0.0415;
                            break;
                    }
                    i++;
                }
                break;
            case 3:
                for (int x = 0; x <= 2; x++) {
                    while (ar_info[i][0] == 0 && ar_info[i][1] == 0) {
                        i++;
                    }
                    switch (i) {
                        case 0:
                            pretarget_p[x][0] = x_ap - 0.0422 + ar_info[0][0] - 0.1125;// distance between ar and target in x
                            pretarget_p[x][1] = z_ap - 0.0826 + ar_info[0][1] + 0.0415;// in y
                            break;
                        case 1:
                            pretarget_p[x][0] = x_ap - 0.0422 + ar_info[1][0] + 0.1125;
                            pretarget_p[x][1] = z_ap - 0.0826 + ar_info[1][1] + 0.0415;
                            break;
                        case 2:
                            pretarget_p[x][0] = x_ap - 0.0422 + ar_info[2][0] + 0.1125;
                            pretarget_p[x][1] = z_ap - 0.0826 + ar_info[2][1] - 0.0415;
                            break;
                        case 3:
                            pretarget_p[x][0] = x_ap - 0.0422 + ar_info[3][0] - 0.1125;
                            pretarget_p[x][1] = z_ap - 0.0826 + ar_info[3][1] - 0.0415;
                            break;
                    }
                    i++;
                }
                break;
            case 4:
                for (int x = 0; x <= 3; x++) {
                    while (ar_info[i][0] == 0 && ar_info[i][1] == 0) {
                        i++;
                    }
                    switch (i) {
                        case 0:
                            pretarget_p[x][0] = x_ap - 0.0422 + ar_info[0][0] - 0.1125;// distance between ar and target in x
                            pretarget_p[x][1] = z_ap - 0.0826 + ar_info[0][1] + 0.0415;// in y
                            break;
                        case 1:
                            pretarget_p[x][0] = x_ap - 0.0422 + ar_info[1][0] + 0.1125;
                            pretarget_p[x][1] = z_ap - 0.0826 + ar_info[1][1] + 0.0415;
                            break;
                        case 2:
                            pretarget_p[x][0] = x_ap - 0.0422 + ar_info[2][0] + 0.1125;
                            pretarget_p[x][1] = z_ap - 0.0826 + ar_info[2][1] - 0.0415;
                            break;
                        case 3:
                            pretarget_p[x][0] = x_ap - 0.0422 + ar_info[3][0] - 0.1125;
                            pretarget_p[x][1] = z_ap - 0.0826 + ar_info[3][1] - 0.0415;
                            break;
                    }
                    i++;
                }
                break;
        }
        Log.d("AR[status]:", "target position: " + Arrays.deepToString(pretarget_p));
        double sumx = 0;
        double sumy = 0;
        double numberofmarker = (int) ar_info[4][0];
        for (int y = 0; y < numberofmarker; y++) {
            sumx += pretarget_p[y][0];
            sumy += pretarget_p[y][1];
        }
        target_p[0] = sumx / numberofmarker;
        target_p[1] = sumy / numberofmarker;
        Log.d("AR[status]:", "target position[avg]: " + Arrays.toString(target_p));
        return target_p;
    }
    public Mat MakeBorder(Mat src,int top,int bottom,int left,int right){
        Size s = src.size();
        int rows = (int) s.height;
        int cols = (int) s.width;
        Mat dst = new Mat(rows+top+bottom,cols+left+right,CvType.CV_32FC1);
        Core.copyMakeBorder(src,dst,top,bottom,left,right,Core.BORDER_CONSTANT,new Scalar(0,0,0));
        return dst;
    }
    public double[][] AR_event(Mat _src,String _mode_, int count_max) {
        String TAG = "AR[status]";
        long start_time = SystemClock.elapsedRealtime();
        int count = 0;
        int ar_id_raw[] = new int[4];
        double ar_center[][] = new double[5][2];
        double ar_center_pixel[][] = new double[4][2];
        int i = 0;
        double pixeltoM    = 0;
        double sumpixeltoM = 0;
        int x = 500;
        int y = 774;
        int height = 960-y;
        int width  =760-x;
        double avg_x = width/2;
        double avg_y = height/2;
        double offsetpx_x = (x+avg_x)-640;
        double offsetpx_y = (y+avg_y)-480;
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        Mat inputImage = undistordAR(MakeBorder(cropImage(_src,x,y,height,width),0,100,0,100), _mode_,x,y);
        long stop_time = SystemClock.elapsedRealtime();
        Log.d("AR[status]", "time: " + ((stop_time - start_time) / 1000));
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Log.d("AR[status]:", "start");
        while (count < count_max) {
            try {
                start_time = SystemClock.elapsedRealtime();
                Log.d("AR[status]:", "decoding");
                Aruco.detectMarkers(inputImage, dictionary, corners, markerIds);
                ar_center[4][0]=corners.size();
                String dump = markerIds.dump();
                Log.d("AR[status]:", "markerIds: " + dump);
                for (i = 0; i < corners.size(); i++) {
                    String dump2 = corners.get(i).dump();
                    Log.d(TAG, "corner: "+dump2);
                }
                for (i = 0; i < corners.size(); i++) {
                    sumpixeltoM += corners.get(i).get(0,1)[0] - corners.get(i).get(0,0)[0];
                    sumpixeltoM += corners.get(i).get(0,3)[1] - corners.get(i).get(0,0)[1];
                }
                pixeltoM = 0.05/(sumpixeltoM/(corners.size()*2));
                Log.d(TAG, "sumpixeltoM:"+sumpixeltoM);
                Log.d(TAG, "divider:"+(corners.size()*2));
                Log.d(TAG, "Pixel to M:"+pixeltoM);
                Log.d(TAG, "TEST pixel to M(0.05m):"+((corners.get(0).get(0,1)[0] - corners.get(0).get(0,0)[0])*pixeltoM));
                for (i = 0; i < corners.size(); i++) {
                    Log.d("AR[status]:", "loop:" + i);
                    ar_id_raw[i] = (int) markerIds.get(i, 0)[0];
                    Log.d("AR[status]:", "marker id sucessful");
                    switch (ar_id_raw[i]) {
                        case 1:
                            ar_center_pixel[0][0] = ((corners.get(i).get(0,0)[0]+corners.get(i).get(0,1)[0]+corners.get(i).get(0,2)[0]+corners.get(i).get(0,3)[0])/4)-avg_x+offsetpx_x;
                            ar_center_pixel[0][1] = ((corners.get(i).get(0,0)[1]+corners.get(i).get(0,1)[1]+corners.get(i).get(0,2)[1]+corners.get(i).get(0,3)[1])/4)-avg_y+offsetpx_y;
                            ar_center[0][0]= ar_center_pixel[0][0]*pixeltoM;
                            ar_center[0][1]= ar_center_pixel[0][1]*pixeltoM;
                            break;
                        case 2:
                            ar_center_pixel[1][0] = ((corners.get(i).get(0,0)[0]+corners.get(i).get(0,1)[0]+corners.get(i).get(0,2)[0]+corners.get(i).get(0,3)[0])/4)-avg_x+offsetpx_x;
                            ar_center_pixel[1][1] = ((corners.get(i).get(0,0)[1]+corners.get(i).get(0,1)[1]+corners.get(i).get(0,2)[1]+corners.get(i).get(0,3)[1])/4)-avg_y+offsetpx_y;
                            ar_center[1][0]= ar_center_pixel[1][0]*pixeltoM;
                            ar_center[1][1]= ar_center_pixel[1][1]*pixeltoM;
                            break;
                        case 3:
                            ar_center_pixel[2][0] = ((corners.get(i).get(0,0)[0]+corners.get(i).get(0,1)[0]+corners.get(i).get(0,2)[0]+corners.get(i).get(0,3)[0])/4)-avg_x+offsetpx_x;
                            ar_center_pixel[2][1] = ((corners.get(i).get(0,0)[1]+corners.get(i).get(0,1)[1]+corners.get(i).get(0,2)[1]+corners.get(i).get(0,3)[1])/4)-avg_y+offsetpx_y;
                            ar_center[2][0]= ar_center_pixel[2][0]*pixeltoM;
                            ar_center[2][1]= ar_center_pixel[2][1]*pixeltoM;
                            break;
                        case 4:
                            ar_center_pixel[3][0] = ((corners.get(i).get(0,0)[0]+corners.get(i).get(0,1)[0]+corners.get(i).get(0,2)[0]+corners.get(i).get(0,3)[0])/4)-avg_x+offsetpx_x;
                            ar_center_pixel[3][1] = ((corners.get(i).get(0,0)[1]+corners.get(i).get(0,1)[1]+corners.get(i).get(0,2)[1]+corners.get(i).get(0,3)[1])/4)-avg_y+offsetpx_y;
                            ar_center[3][0]= ar_center_pixel[3][0]*pixeltoM;
                            ar_center[3][1]= ar_center_pixel[3][1]*pixeltoM;
                            break;
                    }
                    Log.d("AR[status]:", "marker position(pixel): " + Arrays.deepToString(ar_center_pixel));
                    Log.d("AR[status]:", "marker position(meter): " + Arrays.deepToString(ar_center));
                }
                stop_time = SystemClock.elapsedRealtime();
                Log.d("AR[status]", "time[" + count + "]: " + ((stop_time - start_time) / 1000));
            } catch (Exception e) {
                Log.d("AR[status]:", " Not detected:" + e);
            }
            Log.d("AR[status]:", "found " + corners.size() + " marker");
            Log.d("AR[status]:", "final marker position: " + Arrays.deepToString(ar_center));
            count++;
        }
        return  ar_center;
    }

    public double[] QR_event(Mat _src,String _mode_, int count_max,double _scale) {
        String contents = null;
        int count = 0;
        double ap_x = 0, ap_y = 0, ap_z = 0, koz_pattern = 0;
        while (contents == null && count < count_max) {
            Log.d("QR[status]:", " start");
            long start_time = SystemClock.elapsedRealtime();
            Log.d("QR[NO]: ", "A");
            // current best tune is mk3 tune 11!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            // present tune is mk4 tune 1 !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            Mat srcmat = cropImage(_src,550,509,272,272);
            Log.d("QR[status]:", "before resize complete");
            Mat src_mat = resizeImage(srcmat, _scale);
            Log.d("QR[status]:", "resize complete");
            Size s = src_mat.size();
            int rows = (int) s.height;
            int cols = (int) s.width;
            Log.d("QR[status]:", "size mat(w*h):  " + cols + " x " + rows);
            Bitmap bMap = Bitmap.createBitmap(cols, rows, Bitmap.Config.ARGB_8888);
            matToBitmap(src_mat, bMap, false);
            Log.d("QR[status]:", "mat to bitmap complete");
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            int[] intArray = new int[bMap.getWidth() * bMap.getHeight()];
            bMap.getPixels(intArray, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());
            LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArray);
            BinaryBitmap bitmap = new BinaryBitmap(new HybridBinarizer(source));
            Log.d("QR[status]:", "bitmap complete");
            QRCodeReader reader = new QRCodeReader();
            try {
                Log.d("QR[status]:", "start decoder~~~");
                com.google.zxing.Result result = reader.decode(bitmap);
                Log.d("QR[status]:", "decode step 1");
                contents = result.getText();
                Log.d("QR[status]:", "decode step 2");
                Log.d("QR[status]:", " Detected: " + contents);
                String[] multi_contents1 = contents.split(",");
                String[] multi_contents2 = multi_contents1[0].split(":");
                String[] multi_contents3 = multi_contents1[1].split(":");
                String[] multi_contents4 = multi_contents1[2].split(":");
                String[] multi_contents5 = multi_contents1[3].split(":");
                String multi_contents6 = multi_contents5[1].replace("}", "");
                koz_pattern = Double.parseDouble(multi_contents2[1]);
                ap_x = Double.parseDouble(multi_contents3[1]);
                ap_y = Double.parseDouble(multi_contents4[1]);
                ap_z = Double.parseDouble(multi_contents6);
            } catch (Exception e) {
                Log.d("QR[status]:", " Not detected:" + e);
            }
            //////////////////////////////////////////////////////////////////////////////////////////////////////
            Log.d("QR[status]:", " stop");
            long stop_time = SystemClock.elapsedRealtime();
            Log.d("QR[count]:", " " + count);
            Log.d("QR[total_time]:", " " + (stop_time - start_time) / 1000);
            //log.d("QR[multicontents2:","pattern:"+koz_pattern);
            //log.d("QR[multicontents2:","x:"+ap_x);
            //log.d("QR[multicontents2:","y:"+ap_y);
            //log.d("QR[multicontents2:","z:"+ap_z);
            count++;
        }
        api.sendDiscoveredQR(contents);
        return new double[]{koz_pattern, ap_x, ap_y, ap_z};
    }

    public Mat resizeImage(Mat src, double scale) {
        Mat dst = new Mat();
        int nav_max_w = 1280;// 4:3
        int nav_max_h = 960; //--+
        Size srcsize = src.size();
        int rows = (int) srcsize.height;
        int cols = (int) srcsize.width;
        int width2 = (int) (cols * scale);
        int height2 = (int) (rows * scale);
        Size size = new Size(width2, height2);
        Imgproc.resize(src, dst, size);
        return dst;
    }

    public double getPointCloud(int center_range) {
        double depth = 0;
        int count = 0;

        Log.d("PointCloud[status]:", " start");
        PointCloud hazCam = api.getPointCloudHazCam();
        Point[] point = hazCam.getPointArray();
        int width = hazCam.getWidth();
        int height = hazCam.getHeight();
        int row_max = height / 2 + center_range / 2;
        int row_min = height / 2 - center_range / 2;
        int col_max = width / 2 + center_range / 2;
        int col_min = width / 2 - center_range / 2;
        Log.d("PointCloud[status]:", " stop");
        //////////////////////////////////////////////////////////////////////////////////////////////////////
        for (int row = row_min; row < row_max; row++) {
            for (int col = col_min; col < col_max; col++) {
                depth += point[(row * width) + col].getZ();
                count++;
            }
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////////


        depth /= count;
        Log.d("PointCloud[value]:", "z[" + depth + "]");
        return depth;
    }

    public void wait(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}