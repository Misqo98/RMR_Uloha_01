#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#define ENC_POSITIVE_TRESHOLD 60000
#define ENC_NEGATIVE_TRESHOLD -60000
#define ENC_MAX_VAL 65535

#include <QMainWindow>
#include <QTimer>
#include<windows.h>
#include<iostream>
//#include<arpa/inet.h>
//#include<unistd.h>
//#include<sys/socket.h>
#include<sys/types.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
#include <fstream>
#include<vector>
#include "ckobuki.h"
#include "DataSender.h"
#include "rplidar.h"
/*#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"*/

namespace Ui {
class MainWindow;
}
enum PositioningResult{
    none,
    positioning,
    done
};

typedef struct{
    double x =0.0;
    double y=0.0;
    double fi=0.0;
    double dist=0.0;
}Coordinates;



typedef struct{
    int width = 120;
    int heigh = 120;
    int midX = width/2;
    int midY = heigh/2;
   int array[120][120];
}robotMap;

typedef struct{
    int width = 40;
    int heigh = 40;
    int midX = width/2;
    int midY = heigh/2;
   int array[40][40];
}robotResizedMap;

typedef struct{
    int lenght = 4;
    int x[4]={-1,0,0,1};
    int y[4]={0,1,-1,0};
}Direction;

typedef struct{
    int lenght = 4;
    int x[4]={-1,0,0,1};
    int y[4]={0,1,-1,0};
}DirectionPath;

typedef struct{
    int x;
    int y;
    int value;
}PointIdx;

typedef struct{
    double x;
    double y;
    int value;
}PointCoor;

typedef struct{
    PointCoor right;
    PointCoor left;
}ObstacleEdges;

typedef struct{
   std::vector<std::vector<PointIdx>>  map;\


}floodMap;

typedef struct{
    bool start = 0;
    bool circularMovement = 0;
    bool rotation = 0;
    bool acceleration = 1;
}PositioningState;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    bool useCamera;
  //  cv::VideoCapture cap;

    int actIndex;
    //    cv::Mat frame[3];


    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void robotprocess();
    void laserprocess();
    void processThisLidar(LaserMeasurement &laserData);
    PointIdx coordToMapIdx(PointCoor pointRobot);
    void mapNavigation();
    PointCoor mapIdxToCoords(PointIdx pointMap);
    vector<PointCoor> mapIdxToCoordsList(vector<PointIdx> pointsMap);
    PointCoor findSafePoints(PointCoor obstacleEdge, Coordinates robotPosition, double safeDistance, bool left, double safeAngle);
    bool obstacleNearby(Coordinates robotPosition, double sideSaveDistance, double frontSaveDistance);
    void processThisRobot();
    ObstacleEdges findEdges(double robX , double robY, double robAngle);
    void avoidObstacles(PointCoor target, Coordinates actual);
    PointCoor getBestEdge(ObstacleEdges obstacle, PointCoor target, Coordinates actual, bool* left);

    HANDLE robotthreadHandle; // handle na vlakno
    DWORD robotthreadID;  // id vlakna
    static DWORD WINAPI robotUDPVlakno(void *param)
    {
        ((MainWindow*)param)->robotprocess();
        return 0;
    }
    HANDLE laserthreadHandle; // handle na vlakno
    DWORD laserthreadID;  // id vlakna
    static DWORD WINAPI laserUDPVlakno(void *param)
    {
        ((MainWindow*)param)->laserprocess();

        return 0;
    }
    //veci na broadcast laser
    struct sockaddr_in las_si_me, las_si_other,las_si_posli;

    int las_s,  las_recv_len;
    unsigned int las_slen;
    //veci na broadcast robot
    struct sockaddr_in rob_si_me, rob_si_other,rob_si_posli;

    int rob_s,  rob_recv_len;
    unsigned int rob_slen;

    void localisation();
    void writeMapToCsv(int map[120][120]);
    double mapping(double robX , double robY , double robAngle);
    double euclideanDistance(double x1, double y1, double x2, double y2);
    double directionAngle(double x1, double y1, double x2, double y2);
    double radToDeg(double radians);
    double degToRad(double degree);
    void robotRotate(double angl);
    void robotStop();
    void robotArcMove(double translation,double radius);
    std::vector<PointIdx> findNeighbour(PointIdx position, Direction neighbours, robotResizedMap *map, int foundStart[1]);
    vector<PointIdx> findPath(robotResizedMap map, PointIdx start, PointIdx goal);
    void positionning();
    bool mapFloodFill(robotResizedMap *map, PointIdx start, PointIdx goal);
    robotResizedMap resizeMapFill(robotMap map);
    robotMap readMapToArr(string path);
    bool init = true;

private:
    double actualEncLeft, actualEncRight, actualFi = 0.0;
    double newFi, xPosition, yPosition, lRight, lLeft = 0.0;
    double x, y = 0.0;
    double fiAbs = 0.0;
    double ramp = 0.0;
    bool avoidTask = false;
    bool isMaping = false;
    bool isRottating = false;
    double Kp=600,Kr=103;
    double angleErr, distErr = 0.0;
    vector<PointCoor> targetPointPath;
    robotMap map;
    PositioningResult positioningReslut = none;
    DataSender dataSend;
    Coordinates targetPosition;
    Coordinates actualPosition;
    PointCoor globalGoal;

    bool moveToCoor(PointCoor point);
    bool moveToCoors(vector<PointCoor> *points);


private slots:
    void on_pushButton_9_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_5_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_clicked();
    void getNewFrame();

    void on_pushButton_11_clicked();

    void on_pushButton_10_clicked();

    void on_pushButton_8_clicked();

    void on_pushButton_12_clicked();

    void on_pushButton_13_clicked();

    void on_pushButton_14_clicked();

private:
     JOYINFO joystickInfo;
    Ui::MainWindow *ui;
     void paintEvent(QPaintEvent *event);// Q_DECL_OVERRIDE;
     int updateLaserPicture;
     LaserMeasurement copyOfLaserData;
     std::string ipaddress;
     CKobuki robot;
     TKobukiData robotdata;
     int datacounter;
     QTimer *timer;
     PositioningState positioningState;

public slots:
     void setUiValues(DataSender dataSend);
signals:
     void uiValuesChanged(DataSender dataSend); ///toto nema telo


};

#endif // MAINWINDOW_H
