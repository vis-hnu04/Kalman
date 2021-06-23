#ifndef ISENSOR_IMPLEMENTATION_H_
#define ISENSOR_IMPLEMENTATION_H_ ISENSOR_IMPLEMENTATION_H_

#include"ISensorInterface.h"

class InterfaceImplementation :public ISensorInterface{

  public:

    InterfaceImplementation() :_sock(0){
       memset(&_serverAddress, '0' ,sizeof(_serverAddress));
    } 
   
    bool connectToSensor(const int port = SENSOR_SERVER_PORT,
                          const char *ip = SENSOR_SERVER_IP_ADDRESS){  


      _serverAddress.sin_family = AF_INET;
      _serverAddress.sin_port = htons(port);                           
      if ((_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
      {
        perror("socket failed");
        exit(EXIT_FAILURE);
        return 0;
        }

      if(inet_pton(AF_INET, ip, &_serverAddress.sin_addr)<=0) 
      {
        printf("\nInvalid address/ Address not supported \n");
        return 0;
        }

      if (connect(_sock, (struct sockaddr *)&_serverAddress, sizeof(_serverAddress)) < 0)
      {
        printf("\nConnection Failed \n");
        return 0;
        }
      return true;
          
      }

    bool getNextObjectList(SensorObjectList &objectList) {  

      if( recv( _sock , (char*) &objectList,sizeof(SensorObjectList),0) <0){
        printf("\n Reciving Failed");
        return 0;
      }

      return 1;    

      }

    bool confirmObjectsReceived(){

      if(send(_sock,OBJECTS_RECEIVED_MSG, OBJECTS_RECEIVED_MSG_SIZE,0 )<0){
        printf("\n Sending Failed\n");
        return 0;
      }

      return 1; 
    }


    bool closeConnection(){

    if (close(_sock) < 0){
       printf("\nFailed to Close the communication \n");
       return 0;
    }
      return 1;
    }

  private:
    int _sock;
    struct sockaddr_in _serverAddress; 

    } ;
#endif