#ifndef ISENSOR_INTERFACE_H_
#define ISENSOR_INTERFACE_H_ ISENSOR_INTERFACE_H_
#include "Definitions.h"
#include <netinet/in.h>
#include <sys/socket.h>
#include<stdio.h>
#include<stdlib.h>
#include<arpa/inet.h>
#include <unistd.h>
#include<string.h>

/*
 *This interface must be implemented in order to
 *establish a connection to the vehicle sensor.
 */
class ISensorInterface {
public:
  /// Define a port and an ip address to connect to the vehicle sensoe
  virtual bool connectToSensor(const int port = SENSOR_SERVER_PORT,
                               const char *ip = SENSOR_SERVER_IP_ADDRESS) = 0;
  /// Get the current object list from the sensor
  virtual bool getNextObjectList(SensorObjectList &objectList) = 0;

   /// Send OBJECTS_RECEIVED_MSG to the server AFTER processing the
  /// object list in the fusion
  virtual bool confirmObjectsReceived() = 0;
  /// Disconnect from sensor
  virtual bool closeConnection() = 0;
};


class InterfaceImplementation :public ISensorInterface{

  public:

    InterfaceImplementation() :_sock(0){
       memset(&_serveraddress, '0' ,sizeof(_serveraddress));
    } 
   
    bool connectToSensor(const int port = SENSOR_SERVER_PORT,
                          const char *ip = SENSOR_SERVER_IP_ADDRESS){  


      _serveraddress.sin_family = AF_INET;
      _serveraddress.sin_port = htons(port);                           
      if ((_sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
      {
        perror("socket failed");
        exit(EXIT_FAILURE);
        return 0;
        }

      if(inet_pton(AF_INET, ip, &_serveraddress.sin_addr)<=0) 
      {
        printf("\nInvalid address/ Address not supported \n");
        return 0;
        }

      if (connect(_sock, (struct sockaddr *)&_serveraddress, sizeof(_serveraddress)) < 0)
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
    struct sockaddr_in _serveraddress; 

    } ;

#endif
