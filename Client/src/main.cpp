#include "ISensorImplementation.h"
#include <iostream>
#include"KalmanTracker.h"
#include"IFusionInterface.h"



int main()
 {
  ISensorInterface *sensorInterface;
  IFusionInterface *fusionInterface;
  
  InterfaceImplementation interface;
  KalmanTracker  kalmanFilter;
  
 
  fusionInterface  = &kalmanFilter;
  sensorInterface = &interface;
  
  
  SensorObjectList objectList;
    if(sensorInterface->connectToSensor()){
    std::cout<<"connection established with sensor /n"<<std::endl;
    
  } 
  
  
   while(sensorInterface->getNextObjectList(objectList)){
    //std::cout<<objectList.numOfValidObjects <<std::endl;
      
       SensorObject so1 = objectList.objectList[0];
       SensorObject so2 = objectList.objectList[1];
  
     //std::cout << so1.x << " " << so1.y << " " << so1.vx << " " << so1.vy << std::endl;
     //std::cout << so2.x << " " << so2.y << " " << so2.vx << " " << so2.vy << std::endl;  

     fusionInterface->doUpdate(objectList);

    if( sensorInterface->confirmObjectsReceived()){
     std::cout<<"acknowledgment sent"<<std::endl;

    }
  
  }
  sensorInterface->closeConnection();

  return 0 ;
}

