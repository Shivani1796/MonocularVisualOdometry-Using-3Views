#ifndef CONFIGDATA_H
#define CONFIGDATA_H
#include <string>
#include <iostream>

using namespace std;

class ConfigData
{
public:
    double cameraSensorWidth;
    double cameraSensorHeight;
    double iCx;
    double iCy;
    double dCx;
    double dCy;
    double ratio;
    double imageWidth;
    double imageHeight;
    double parameter0;
    double parameter1;
    double parameter2;
    double parameter3;
    double parameter4;
    double parameter5;
    double calibrationRatio;
    string cameraName;

    ConfigData();
    void setCameraSensorWidth(double cameraSensorWidth);
    void setCameraSensorHeight(double cameraSensorHieght);
    void setIcx(double iCx);
    void setIcy(double iCy);
    void setDcx(double dCx);
    void setDcy(double dCy);
    void setRatio(double ratio);
    void setImageWidth(double imageWidth);
    void setImageHeight(double imageHeight);
    void setParameter0(double parameter0);
    void setParameter1(double parameter1);
    void setParameter2(double parameter2);
    void setParameter3(double parameter3);
    void setParameter4(double parameter4);
    void setParameter5(double parameter5);
    void setCalibrationRatio(double calibrationRatio);
    void setCameraName(string cameraName);
    double getCameraSensorWidth();
    double getCameraSensorHeight();
    double getIcx();
    double getIcy();
    ConfigData *getcd();
    double getDcx();
    double getDcy();
    double getRatio();
    double getImageWidth();
    double getImageHeight();
    double getParameter0();
    double getParameter1();
    double getParameter2();
    double getParameter3();
    double getParameter4();
    double getParameter5();
    double getCalibrationRatio();
    string getCameraName();

};

#endif // CONFIGDATA_H
