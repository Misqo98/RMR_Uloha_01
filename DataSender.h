#ifndef DATASENDER_H
#define DATASENDER_H

#include <QObject>
#include <QMetaType>
class DataSender
{


public:
    DataSender(){}
    ~DataSender(){}
 //   Signal(const Signal &){};

    double x;
    double y;
    double fi;

};
Q_DECLARE_METATYPE(DataSender)


#endif // DATASENDER_H
