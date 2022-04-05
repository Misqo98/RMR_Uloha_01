#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPainter>
#include <math.h>
#include <fstream>

//*********************************************************
//********************** Constructor **********************
//*********************************************************
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    //tu je napevno nastavena ip. treba zmenit na to co ste si zadali do text boxu alebo nejaku inu pevnu. co bude spravna
    ipaddress="127.0.0.1";
    //cap.open("http://192.168.1.11:8000/stream.mjpg");
    ui->setupUi(this);
    datacounter=0;
    globalGoal.x = 0;
    globalGoal.y = 0;
  //  timer = new QTimer(this);
 //   connect(timer, SIGNAL(timeout()), this, SLOT(getNewFrame()));
    actIndex=-1;
    useCamera=false;
    for (int i = 0; i < 120; i++){
      for (int j = 0; j < 120; j++){
        map.array[i][j] = 0;
      }
    }
    datacounter=0;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setBrush(Qt::black);
    QPen pero;
    pero.setStyle(Qt::SolidLine);
    pero.setWidth(3);
    pero.setColor(Qt::green);
    QRect rect(20,120,700,500);
    rect= ui->frame->geometry();
    rect.translate(0,15);
    painter.drawRect(rect);
    {
        if(updateLaserPicture==1)
        {
            updateLaserPicture=0;
            painter.setPen(pero);
            for(int k=0;k<copyOfLaserData.numberOfScans/*360*/;k++)
            {
                int dist=copyOfLaserData.Data[k].scanDistance/20;
                int xp=rect.width()-(rect.width()/2+dist*2*sin((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().x();
                int yp=rect.height()-(rect.height()/2+dist*2*cos((360.0-copyOfLaserData.Data[k].scanAngle)*3.14159/180.0))+rect.topLeft().y();
                if(rect.contains(xp,yp))
                    painter.drawEllipse(QPoint(xp, yp),2,2);
            }
        }
    }
}


//********************************************************
//********************** Processing **********************
//********************************************************
void MainWindow::processThisRobot()
{
      localisation();
      if(isMaping && !positioningState.rotation && !isRottating){
      mapping(actualPosition.x, actualPosition.y, actualPosition.fi);
      }

      /*if(targetPointPath.empty()){
        mapNavigation();
      }else{
          //positioningState.start = 1;
          //moveToCoors(&targetPointPath);
      }*/
        positionning();

    if(datacounter%5)
    {
        dataSend.x = actualPosition.x;
        dataSend.y = actualPosition.y;
        dataSend.fi = actualPosition.fi;
        dataSend.angleErr = angleErr;
        dataSend.distErr = distErr;

        emit uiValuesChanged(dataSend);
    }
    datacounter++;
}

void MainWindow::processThisLidar(LaserMeasurement &laserData)
{
    memcpy( &copyOfLaserData,&laserData,sizeof(LaserMeasurement));
    //tu mozete robit s datami z lidaru.. napriklad najst prekazky, zapisat do mapy. naplanovat ako sa prekazke vyhnut.
    // ale nic vypoctovo narocne - to iste vlakno ktore cita data z lidaru
    updateLaserPicture=1;
    update();//tento prikaz prinuti prekreslit obrazovku.. zavola sa paintEvent funkcia
}
void  MainWindow::setUiValues(DataSender dataSend)
{
     ui->lineEdit_2->setText(QString::number(dataSend.x));
     ui->lineEdit_3->setText(QString::number(dataSend.y));
     ui->lineEdit_4->setText(QString::number(radToDeg(dataSend.fi)));
     ui->lineEdit_7->setText(QString::number(radToDeg(dataSend.angleErr)));
     ui->lineEdit_8->setText(QString::number(dataSend.distErr));
}

void MainWindow::on_pushButton_9_clicked() //start button
{

    //tu sa nastartuju vlakna ktore citaju data z lidaru a robota
    laserthreadHandle=CreateThread(NULL,0,laserUDPVlakno, (void *)this,0,&laserthreadID);
    robotthreadHandle=CreateThread(NULL,0, robotUDPVlakno, (void *)this,0,&robotthreadID);
    /*  laserthreadID=pthread_create(&laserthreadHandle,NULL,&laserUDPVlakno,(void *)this);
      robotthreadID=pthread_create(&robotthreadHandle,NULL,&robotUDPVlakno,(void *)this);*/
    connect(this,SIGNAL(uiValuesChanged(DataSender)),this,SLOT(setUiValues(DataSender)));

}

void MainWindow::on_pushButton_2_clicked() //forward
{
    //pohyb dopredu
    std::vector<unsigned char> mess=robot.setTranslationSpeed(500);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_3_clicked() //back
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(-250);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_6_clicked() //left
{

    std::vector<unsigned char> mess=robot.setRotationSpeed(3.14159/2);

    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_5_clicked()//right
{

    std::vector<unsigned char> mess=robot.setRotationSpeed(-3.14159/2);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::on_pushButton_4_clicked() //stop
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

void MainWindow::laserprocess()
{
    WSADATA wsaData = {0};
    int iResult = 0;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
    las_slen = sizeof(las_si_other);
    if ((las_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }

    int las_broadcastene=1;
    setsockopt(las_s,SOL_SOCKET,SO_BROADCAST,(char*)&las_broadcastene,sizeof(las_broadcastene));
    // zero out the structure
    memset((char *) &las_si_me, 0, sizeof(las_si_me));

    las_si_me.sin_family = AF_INET;
    las_si_me.sin_port = htons(52999);
    las_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    las_si_posli.sin_family = AF_INET;
    las_si_posli.sin_port = htons(5299);
    las_si_posli.sin_addr.s_addr = inet_addr(ipaddress.data());//htonl(INADDR_BROADCAST);
    bind(las_s , (struct sockaddr*)&las_si_me, sizeof(las_si_me) );
    char command=0x00;
    if (sendto(las_s, &command, sizeof(command), 0, (struct sockaddr*) &las_si_posli, las_slen) == -1)
    {

    }
    LaserMeasurement measure;
    while(1)
    {
        if ((las_recv_len = recvfrom(las_s, (char*)&measure.Data, sizeof(LaserData)*1000, 0, (struct sockaddr *) &las_si_other, (int*)&las_slen)) == -1)
        {
            continue;
        }
        measure.numberOfScans=las_recv_len/sizeof(LaserData);
        processThisLidar(measure);
    }
}

//****************************************************************
//********************** Map coordination ***********************
//***************************************************************
PointIdx MainWindow::coordToMapIdx(PointCoor pointRobot){
    PointIdx result;
    int ofset = 40/2;

    result.x = (int)(pointRobot.x*1000.0/300.0);
    result.y = (int)(pointRobot.y*1000.0/300.0);

    result.x += ofset;
    result.y += ofset;
    return result;
}

PointCoor MainWindow::mapIdxToCoords(PointIdx pointMap){
    int ofset = ofset = 40/2;
    PointCoor result;
    result.x = ((double)(pointMap.x-ofset))*300.0/1000.0 + 0.15;
    result.y = ((double)(pointMap.y-ofset))*300.0/1000.0 + 0.15;
    result.value = pointMap.value;
    return result;

}
vector<PointCoor> MainWindow::mapIdxToCoordsList(vector<PointIdx> pointsMap){
    vector<PointCoor> result;
    for(int i = 0; i<pointsMap.size();i++){
        PointIdx idx = pointsMap[i];
        PointCoor coor = mapIdxToCoords(idx);
        result.push_back(coor);
    }
    return result;
}

//****************************************************************
//********************** Map navigation **************************
//****************************************************************
void MainWindow::mapNavigation(){
    robotMap map = readMapToArr("D:\\gitHub\\RMR\\RMR_Uloha_01\\robotmap.csv");
    robotResizedMap resizedMap = resizeMapFill(map);
    vector<PointIdx> navigatePath;
    vector<PointCoor> navigatePathCoord;
    PointCoor goal = globalGoal;
    PointCoor start;
    start.x = actualPosition.x;
    start.y = actualPosition.y;

    PointIdx startIdx = coordToMapIdx(start);
    PointIdx goalIdx = coordToMapIdx(goal);
   if(mapFloodFill(&resizedMap, startIdx, goalIdx)){
        startIdx.value = resizedMap.array[startIdx.x][startIdx.y];
        goalIdx.value = resizedMap.array[goalIdx.x][goalIdx.y];
        navigatePath = findPath(resizedMap, startIdx, goalIdx);
   }

      navigatePathCoord = mapIdxToCoordsList(navigatePath);
      targetPointPath = navigatePathCoord;
}

std::vector<PointIdx> MainWindow::findNeighbour(PointIdx position, Direction neighbours, robotResizedMap *map, int foundStart[1]){
    std::vector<PointIdx> result;
    for(int i = 0; i < neighbours.lenght; i++){
        int x = position.x + neighbours.x[i];
        int y = position.y + neighbours.y[i];
        if(x >= 0 && x < 40 && y>=0 && y < 40){
            int value = map->array[x][y];
            if(value == INT_MAX){
                map->array[x][y] =position.value + 1;
                //map->array[x][y] =1000;
                *foundStart = 1;
                result.clear();
                break;
            }
            if(value == 0){
                map->array[x][y] =position.value + 1;
                PointIdx newPoint;
                newPoint.x = x;
                newPoint.y = y;
                newPoint.value = position.value +1;
                result.push_back(newPoint);
            }
        }
    }

    return result;
}
bool MainWindow::mapFloodFill(robotResizedMap *map, PointIdx start, PointIdx goal){
    Direction neighbours;
    int startFound = 0;
        bool result = false;
    if(map->array[start.x][start.y] != 1 && map->array[goal.x][goal.y] != 1 ){
        start.value = INT_MAX;
        goal.value = 2;
        map->array[start.x][start.y] = start.value;
         map->array[goal.x][goal.y] = goal.value;
        std::vector<PointIdx> actualPoint;
        std::vector<PointIdx> nextPoint;
        nextPoint.push_back(goal);

        while(!nextPoint.empty()){
            actualPoint.clear();
            actualPoint.insert(actualPoint.end(), nextPoint.begin(), nextPoint.end());
            nextPoint.clear();
            while(!actualPoint.empty()){
               PointIdx act =  actualPoint[0];
               std::vector<PointIdx> actualNeighbours = findNeighbour(act, neighbours, map, &startFound);
               if(startFound == 1){
                   nextPoint.clear();
                   actualPoint.clear();
                   result = true;
                   break;
               }

               nextPoint.insert(nextPoint.end(), actualNeighbours.begin(), actualNeighbours.end());
               actualPoint.erase(actualPoint.begin());

            }
        }
        for (int y = 39; y > 0; y--) {
            for (int x = 0; x < 40; x++)
                printf("%02d ",map->array[x][y]);
            cout << "\n";
        }
    }

    return result;
}

vector<PointIdx> MainWindow::findPath(robotResizedMap map, PointIdx start, PointIdx goal){

    DirectionPath selectPath;
    bool init = false;
    int newMinDirection = 0;
    int minPointValue,minPointDirection;
    vector<PointIdx> neighbourPoints;
    vector<PointIdx> pathPoints;
    PointIdx actualPosition = start;
    PointIdx newPosition;
    PointIdx pathPosition;
    //pathPoints.push_back(start);

    minPointValue = INT_MAX;
    while(actualPosition.value != goal.value){
        for(int i=0;i<selectPath.lenght;i++){
            newPosition.x = actualPosition.x + selectPath.x[i];
            newPosition.y = actualPosition.y + selectPath.y[i];
            newPosition.value = map.array[newPosition.x][newPosition.y];
           if(newPosition.value > 1 && newPosition.value < minPointValue){
               if(!neighbourPoints.empty()){
                   neighbourPoints.erase(neighbourPoints.begin());
                }
               neighbourPoints.push_back(newPosition);
               minPointValue = neighbourPoints.begin()->value;
               minPointDirection = i;
           }
       }
       pathPosition = actualPosition;
       actualPosition = neighbourPoints.front();
       if(newMinDirection != minPointDirection && init){
           pathPoints.push_back(pathPosition);
           newMinDirection = minPointDirection;

       }
       init = true;
       if(actualPosition.value == goal.value)
           pathPoints.push_back(actualPosition);
    }
    for (std::size_t y = 0; y < pathPoints.size(); y++) {
        printf("%d : x=%d, y=%d ", (int)y, pathPoints[y].x, pathPoints[y].y);
    }
    printf("\n");
   return pathPoints;
}


//*************************************************************************
//********************** Map read, write, and resize **********************
//*************************************************************************
void MainWindow::writeMapToCsv(int map[120][120]){

    ofstream outfile;
    outfile.open("D:\\gitHub\\RMR\\RMR_Uloha_01\\robotmap.csv");

    if (outfile.is_open())
              {
                  for (int y = 119; y >= 0; y--)
                  {
                      for (int x = 0; x < 120; x++)
                      {
                          outfile << map[0+x][0+y] << ";";
                      }
                      outfile << endl;
                  }
                  printf("Map save complete");
      }
      else cout << "Unable to open file";

        outfile.close();
}

robotMap MainWindow::readMapToArr(string path){
    std::ifstream f;
    robotMap result;
    f.open (path);
    if (! f.is_open()) {
            std::cerr << "error: file open failed '" << path << "'.\n";
    }

    std::string line, val;
    std::vector<std::vector<int>> array;
    for (int i = 0; i < 120; i++){
      for (int j = 0; j < 120; j++){
        result.array[i][j] = 0;
      }
    }
    while (std::getline (f, line)) {
        std::vector<int> v;
        std::stringstream s (line);
        while (getline (s, val, ';'))
            v.push_back (std::stoi (val));
        array.push_back (v);
    }
    std::reverse(array.begin(), array.end());
    for (std::size_t y = 0; y < 120; y++) {
        for (std::size_t x = 0; x < 120; x++)
            result.array[x][y] = array[y][x];
            //std::cout << array[y][x] << " ";
        //std::cout << "\n";
    }


    return result;
}

robotResizedMap MainWindow::resizeMapFill(robotMap map){
    robotResizedMap result;
    for (int i = 0; i < 40; i++){
      for (int j = 0; j < 40; j++){
        result.array[i][j] = 0;
      }
    }

    for(int y=0; y<map.heigh; y++){
        for(int x=0; x<map.width; x++){
            int resultX = x/3;
            int resultY = y/3;
            if(result.array[resultX][resultY] ||  map.array[x][y]){
                result.array[resultX][resultY] = 1;
            }else{
                result.array[resultX][resultY] = 0;
            }
            //result.array[resultX][resultY] |=  map.array[x][y];
        }
    }
    ofstream outfile;
    outfile.open("D:\\gitHub\\RMR\\RMR_Uloha_01\\robotmap_resized.csv");

    if (outfile.is_open())
              {
                  for (int y = 39; y > 0; y--)
                  {
                      for (int x = 0; x < 40; x++)
                      {
                          outfile << result.array[0+x][0+y] << ";";
                      }
                      outfile << endl;
                  }

      }
      else cout << "Unable to open file";

        outfile.close();
    return result;
}
//*****************************************************
//********************** Mapping **********************
//*****************************************************
double MainWindow::mapping(double robX , double robY , double robAngle)
{
    int mapX=0,mapY=0;
    for(int i=0; i<copyOfLaserData.numberOfScans; i++)
    {
        double obstacleAngle=(360-copyOfLaserData.Data[i].scanAngle)*(PI/180);
        double robAngle2PI = robAngle >= 0 ? robAngle : robAngle + 2*PI;

        if (copyOfLaserData.Data[i].scanDistance > 150 && copyOfLaserData.Data[i].scanDistance < 3000){//mm
            mapX=(((robX)*1000 + copyOfLaserData.Data[i].scanDistance * cos(robAngle2PI+obstacleAngle))/100);
            mapY=(((robY)*1000 + copyOfLaserData.Data[i].scanDistance * sin(robAngle2PI+obstacleAngle))/100);

            map.array[(int)mapX+map.midX][(int)mapY+map.midY]=1;
        }
    }
    return 1.0;


}

//***************************************************************
//********************** Robot positioning **********************
//***************************************************************
void MainWindow::robotArcMove(double translation,double radius) //stop
{
    std::vector<unsigned char> mess=robot.setArcSpeed(translation,radius);
     if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
        {

     }
}

void MainWindow::robotStop() //stop
{
    std::vector<unsigned char> mess=robot.setTranslationSpeed(0);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}
void MainWindow::robotRotate(double angl)
{

    std::vector<unsigned char> mess=robot.setRotationSpeed(angl);

    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
}

double MainWindow::euclideanDistance(double x1, double y1, double x2, double y2){
   return (double)sqrt(pow(x2-x1,2)+pow(y2-y1,2));
}
double MainWindow::directionAngle(double x1, double y1, double x2, double y2){
    // Funkcia atan vracia hodnoty v rozsahu -pi-pi
    double result = atan2(y2-y1, x2-x1);
    return result;

}

double MainWindow::radToDeg(double radians){
   return radians * 180/PI;
}

double MainWindow::degToRad(double degree){
   return degree * PI/180;
}

bool MainWindow::moveToCoor(PointCoor point){

    if(positioningState.start == 0){
        targetPosition.x = point.x;
        targetPosition.y = point.y;
        positioningState.start = 1;
    }
    double dist = euclideanDistance(actualPosition.x, actualPosition.y, targetPosition.x, targetPosition.y);
    return dist < 0.1;
}

bool MainWindow::moveToCoors(vector<PointCoor> *points){
    if(!points->empty()){
        bool respond  = moveToCoor((*points)[0]);
        if(respond){
            points->erase(points->begin());
        }
    }
    return points->empty();
}

void MainWindow::positionning(){
    if(!targetPointPath.empty()){
        targetPosition.x = targetPointPath[0].x;
        targetPosition.y = targetPointPath[0].y;
    }
    targetPosition.fi = directionAngle(actualPosition.x, actualPosition.y, targetPosition.x, targetPosition.y);
    targetPosition.dist = euclideanDistance(actualPosition.x, actualPosition.y, targetPosition.x, targetPosition.y);

    double angleDiff = 0;
    angleDiff = targetPosition.fi-actualPosition.fi;
    if(angleDiff < -PI){
        angleDiff += 2*PI;
    }else if (angleDiff>PI){
        angleDiff-=2*PI;
    }
    angleErr = angleDiff;
    distErr = targetPosition.dist;
    if (positioningState.start)
    {
        printf("Start\n");
        if (abs(angleDiff) > PI/4 ) {
            positioningState.rotation =1;
            positioningState.circularMovement=0;
        }
        else{
            positioningState.rotation=0;
            positioningState.circularMovement=1;
        }

        if (targetPosition.dist < 0.1) {
            targetPointPath.erase(targetPointPath.begin());
            //positioningState.start=0;
            positioningState.start = targetPointPath.empty() ? 0 : 1;
            robotStop();
            positioningState.acceleration=1;
            ramp=0;

        }
    }

    if(positioningState.start && positioningState.rotation)
    {
           if (angleDiff > 0 ){
               robotRotate(PI/2);
           }
           else{
               robotRotate(-PI/2);
           }
    }

    if(positioningState.start && positioningState.circularMovement)
     {

         double trans = Kp*targetPosition.dist;
         double radius = 0;
         if(angleDiff == 0){
             radius = 5000;
         }else{
            radius = Kr/(angleDiff);
         }

         if (trans>400)
             trans=400;
         if (positioningState.acceleration)
             trans=trans*ramp;

         if (trans<15)
             trans=15;
        if(radius < 8){
            isRottating = true;
        }else{
            isRottating = false;
        }
         robotArcMove(trans,radius);
         if (ramp>=1.0)
             positioningState.acceleration=0;
         else
             ramp=ramp+0.01;

     }
}
//****************************************************************
//********************** Robot Localisation **********************
//****************************************************************
void MainWindow::localisation(){

    double newEncLeft = robotdata.EncoderLeft;
    double newEncRight = robotdata.EncoderRight;
    double diameter = robot.getB();
    double tickToMeter = robot.getTickToMeter();


    /*if(init){

    }*/

    if(actualEncLeft - newEncLeft > ENC_POSITIVE_TRESHOLD){
      lLeft = tickToMeter* (newEncLeft - actualEncLeft + ENC_MAX_VAL);
    }else if (actualEncLeft - newEncLeft < ENC_NEGATIVE_TRESHOLD){
        lLeft = tickToMeter* (newEncLeft - actualEncLeft - ENC_MAX_VAL);
    }else
        lLeft = tickToMeter* (newEncLeft - actualEncLeft);

    if(actualEncRight - newEncRight > ENC_POSITIVE_TRESHOLD){
      lRight = tickToMeter* (newEncRight - actualEncRight + ENC_MAX_VAL);
    }else if (actualEncRight - newEncRight < ENC_NEGATIVE_TRESHOLD){
        lRight = tickToMeter* (newEncRight - actualEncRight - ENC_MAX_VAL);
    }else
        lRight = tickToMeter * (newEncRight - actualEncRight);

    double dAlpha = (lRight - lLeft)*(1.0/diameter);
    newFi = actualFi + dAlpha;

    if (newFi < - PI){
            newFi += 2*PI;
        }
     else if (newFi > PI){
            newFi -= 2*PI;
        }

    actualPosition.fi = newFi;
    if(lRight == lLeft){
        actualPosition.x = actualPosition.x + lRight*cos(actualFi);
        actualPosition.y = actualPosition.y + lRight*sin(actualFi);
    }else{
        actualPosition.x = actualPosition.x + ((diameter*(lRight + lLeft)/(2.0*(lRight-lLeft)))*(sin(newFi)-sin(actualFi)));
        actualPosition.y = actualPosition.y - ((diameter*(lRight + lLeft)/(2.0*(lRight-lLeft)))*(cos(newFi)-cos(actualFi)));
    }
    actualFi = newFi;
}

//***********************************************************
//********************** Robot process **********************
//***********************************************************
void MainWindow::robotprocess()
{
    WSADATA wsaData = {0};
    int iResult = 0;

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);

    if ((rob_s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
    {

    }
    char rob_broadcastene=1;
    setsockopt(rob_s,SOL_SOCKET,SO_BROADCAST,&rob_broadcastene,sizeof(rob_broadcastene));
    // zero out the structure
    memset((char *) &rob_si_me, 0, sizeof(rob_si_me));

    rob_si_me.sin_family = AF_INET;
    rob_si_me.sin_port = htons(53000);
    rob_si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    rob_si_posli.sin_family = AF_INET;
    rob_si_posli.sin_port = htons(5300);
    rob_si_posli.sin_addr.s_addr =inet_addr(ipaddress.data());//inet_addr("10.0.0.1");// htonl(INADDR_BROADCAST);
    rob_slen = sizeof(rob_si_me);
    bind(rob_s , (struct sockaddr*)&rob_si_me, sizeof(rob_si_me) );

    std::vector<unsigned char> mess=robot.setDefaultPID();
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    Sleep(100);
    mess=robot.setSound(440,1000);
    if (sendto(rob_s, (char*)mess.data(), sizeof(char)*mess.size(), 0, (struct sockaddr*) &rob_si_posli, rob_slen) == -1)
    {

    }
    unsigned char buff[50000];
    while(1)
    {
        memset(buff,0,50000*sizeof(char));
        if ((rob_recv_len = recvfrom(rob_s, (char*)&buff, sizeof(char)*50000, 0, (struct sockaddr *) &rob_si_other,(int*) &rob_slen)) == -1)
        {

            continue;
        }

        actualEncLeft = robotdata.EncoderLeft;
        actualEncRight = robotdata.EncoderRight;

        int returnval=robot.fillData(robotdata,(unsigned char*)buff);//ziskame data
        if(returnval==0)
        {
            if(init){
                if(actualEncLeft == 0){
                    actualEncLeft = robotdata.EncoderLeft;
                }
                if(actualEncRight == 0){
                    actualEncRight = robotdata.EncoderRight;
                }
                if(actualFi == 0){
                    actualFi = robotdata.GyroAngle/100.0 * PI/180.0;
                }
                init = false;
            }
            processThisRobot();

        }


    }
}

//********************** Slots **********************

void MainWindow::on_pushButton_clicked()
{
    if(useCamera==true)
    {
        useCamera=false;
        timer->stop();
        ui->pushButton->setText("use camera");
    }
    else
    {
        useCamera=true;
        timer->start(30);
        ui->pushButton->setText("use laser");
    }
}



void MainWindow::on_pushButton_11_clicked()
{
    globalGoal.x = (ui->lineEdit_5->text().toDouble());
    globalGoal.y = (ui->lineEdit_6->text().toDouble());
    mapNavigation();
    positioningState.start = 1; // Zacni polohovat
}

void MainWindow::on_pushButton_10_clicked()
{
    writeMapToCsv(map.array);
}


void MainWindow::on_pushButton_8_clicked()
{

}

void MainWindow::on_pushButton_12_clicked()
{
    if(isMaping==true)
    {
        isMaping=false;

        ui->pushButton_12->setText("Start mapping");
    }
    else
    {
        isMaping=true;

        ui->pushButton_12->setText("Stop mapping");
    }
}

void MainWindow::on_pushButton_13_clicked()
{
    readMapToArr("D:\\gitHub\\RMR\\RMR_Uloha_01\\robotmap.csv");
}
void MainWindow::getNewFrame()
{

}
