#include "ISensorInterface.h"
#include <iostream>
#include"KalmanTracker.h"
#include"IFusionInterface.h"



int main()
 {
  // This is the entry point for your solution.
  // Connect to the sensor and track all objects.
  // Have fun!!!

    


  InterfaceImplementation Is ;
  int i =100;
  SensorObjectList objectList;
  KalmanTracker  kf;
  if(Is.connectToSensor()){
    std::cout<<"connection established with sensor /n"<<std::endl;
    
  } 
  
  while(i>0){
  if(Is.getNextObjectList(objectList)){
    std::cout<<"recived object"<<std::endl;
  }   
 
 
    //std::cout<<objectList.numOfValidObjects <<std::endl;
      
      std::cout<<i <<std::endl;
      SensorObject so1 = objectList.objectList[0];
       SensorObject so2 = objectList.objectList[1];




  
     //std::cout << so1.x << " " << so1.y << " " << so1.vx << " " << so1.vy << std::endl;
     //std::cout << so2.x << " " << so2.y << " " << so2.vx << " " << so2.vy << std::endl;  

    

     kf.doUpdate(objectList);

    if( Is.confirmObjectsReceived()){
     std::cout<<"acknowledgment sent"<<std::endl;

    }
   i--;
  }

  return 0 ;
}

