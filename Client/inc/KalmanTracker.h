#ifndef KALMANTRACKER_H_
#define KALMANTRACKER_H_ KALMANTRACKER_H_

#include"IFusionInterface.h"
#include<eigen3/Eigen/Dense>
#include "JSONFileLogger.h"
#include"HungarianAlgorithm.h"

class KalmanTracker : public IFusionInterface
{  
  public:
    
  KalmanTracker();

  ~KalmanTracker();

  void read_state(const Object &object);
   

  void read_covariance(const Object & object);

  void write_object(Object & object);

 
  void predict(const uint64_t timestamp) override ;
   
  void createNewObject(const SensorObject &sensorObject);
  


  bool associate(const SensorObject &sensorObject,
                         uint8_t &associatedObjectIndex);
    
 
  void update(const SensorObject &sensorObject,
                      const uint8_t associatedObjectIndex) ;
     
      



  void doUpdate(const SensorObjectList &sensorObjectList) ;

  void logwriter(const SensorObjectList &sensorObjectList) ;


private:
     VectorXd x_;

    // state covariance matrix
    MatrixXd P_;

    // state transistion matrix
    MatrixXd F_;

    // process covariance matrix
    MatrixXd Q_;

    // measurement matrix
    MatrixXd H_;  

    // measurement covariance matrix
    MatrixXd R_;

    MatrixXd Ht;

    bool is_initialized_;
    
    float threshold_gating;

    std::vector<float> _costMatrix;

    std::vector<std::pair<int,int>> associationArray;


};




#endif
