#ifndef KALMANTRACKER_H_
#define KALMANTRACKER_H_ KALMANTRACKER_H_

#include"IFusionInterface.h"
#include"matrix.h"
#include "JSONFileLogger.h"
#include"HungarianAlgorithm.h"

class KalmanTracker : public IFusionInterface
{  
  public:
    
  KalmanTracker();

  ~KalmanTracker();
  
  // to read the state of a given object within the
  // object.
  //@param[in] Object
  void getState(const Object &object);
   
  // to read the covariance of a given track within the
  // object.
  //@param[in] Object
  void getCovariance(const Object & object);

  //Used to write the predicition and updation to the 
  //given  object.
  //@param[in] Object
  void writeObject(Object & object);

  // to read the detections from  the sensor object
  // object.
  //@param[in] SensorObject
  //@param[out] Matrix of size 4*1 with sensor readings 
  Matrix getMeasurement(const SensorObject &);

 
  void predict(const uint64_t timestamp) ;
   
  void createNewObject(const SensorObject &sensorObject);
  


  bool associate(const SensorObject &sensorObject,
                         uint8_t &associatedObjectIndex);
    
 
  void update(const SensorObject &sensorObject,
                      const uint8_t associatedObjectIndex) ;
     
      

  // use to find the minimum value in the assignment matrix for .
  //the assosiation of the track with the detection. Associations are written to the _associationArray
  //@param[in] cost for different associations
  void findOptimalAssignment(std::vector<std::vector<float>> &associationMatrix);
  
  void doUpdate(const SensorObjectList &sensorObjectList) ;
  
  // use to write the results to the json file.
  //@param[in] SensorObjectList
  void logwriter(const SensorObjectList &sensorObjectList) ;


private:
     Matrix x_;

    // state covariance matrix
    Matrix P_;

    // state transistion matrix
    Matrix F_;

    // process covariance matrix
    Matrix Q_;

    // measurement matrix
    Matrix H_;  

    // measurement covariance matrix
    Matrix R_;
   
   /** transpose of H_*/
    Matrix Ht_;
   
   // flag is set to false for the  first timestamp
    bool _isInitialized;
    
    //threshold for the mahalanobis distance 
    float _gatingThreshold;
    
    //to store the distances for each observation.
    std::vector<float> _costMatrix;
   
    // array to store the mapping of detections with tracks.
    std::vector<std::pair<int,int>> _associationArray;

    


};




#endif
