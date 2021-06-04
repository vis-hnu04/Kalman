#include<eigen3/Eigen/Dense>
#include "JSONFileLogger.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;


bool KalmanTracker::associate(const SensorObject &sensorObject,
                         uint8_t &associatedObjectIndex) {
    VectorXd x_meas(4);
    
    x_meas   <<   sensorObject.x,
                  sensorObject.y,
                  sensorObject.vx,
                  sensorObject.vy;

    int count =0;              
    std::vector<float> cost;
    uint8_t tempIndex = associatedObjectIndex;
    for(int i{0}; i < _objectList.numOfValidObjects ; i++){ 

      
       read_state(_objectList.objects[i]);
       read_covariance(_objectList.objects[i]);
       MatrixXd S = H_ * P_ * H_.transpose() + R_;
       MatrixXd Si = S.inverse();

      auto mahalanaboisDistance = sqrt((x_ - x_meas).transpose() * Si * (x_- x_meas)) + log(S.determinant());


      if(mahalanaboisDistance > threshold_distance && cycle_count > 1){
          continue;
        }
      cost.push_back(mahalanaboisDistance);
      count++;
      associatedObjectIndex = i;
      std::cout<<" track index " << i<<std::endl;
      std::cout<<"distance " << mahalanaboisDistance<<std::endl;  
                         }
      if(count > 1 || cycle_count == 1){ 
        auto itr = std::min_element(cost.begin(),cost.end());
        associatedObjectIndex = itr - cost.begin();
        if(associatedObjectIndex == tempIndex){
          cost.erase(cost.begin()+ tempIndex);
          auto itr = std::min_element(cost.begin(),cost.end());
          associatedObjectIndex = itr - cost.begin();
          return true;
        }
      }            
      return count > 0 ? true: false;                    

         }






class KalmanFilter  
{

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

    bool is_initialized_;


    //pair<int,int> association;
    ObjectList _objectList;    // object database
  uint16_t _currentObjectId; // use this one to create new object id's



  public:


  KalmanFilter()
  {
      is_initialized_ = false;

      // create a 4D state vector, we don't know yet the values of the x state
      x_ = VectorXd(4);

      // state covariance matrix P
      P_ = MatrixXd(4, 4);
      P_ <<     5, 0, 0, 0,
                0, 5, 0, 0,
                0, 0, 5, 0,
                0, 0, 0, 5;


      // measurement covariance
      R_ = MatrixXd(4,4);
      R_ <<     Rxx, 0,  0,   0,
                0, Ryy , 0,   0,
                0, 0 ,Rvxvx,  0,
                0, 0 , 0,  Rvyvy;
      
      // process covaraince matrix  
      Q_ = MatrixXd(4,4);
      Q_ <<     Qxx, 0,  0,   0,
                0, Qyy , 0,   0,
                0, 0 ,Qvxvx,  0,
                0, 0 , 0,  Qvyvy;


      // measurement matrix 
      H_ = MatrixXd(4, 4);
      H_ <<     1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

      // the initial transition matrix F_
      F_ = MatrixXd(4, 4);
      F_ <<     1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  } 

 
  void predict(const uint64_t timestamp) {//

      for (int i{0}; i< _objectList.numOfValidObjects; i++ ){
      x_   <<             _objectList.objects[i].x,
                          _objectList.objects[i].y,
                          _objectList.objects[i].vx,
                          _objectList.objects[i].vy;
      
      float dt = (timestamp - _objectList.timestamp)/1000000.0;


      P_ <<           _objectList.objects[i].Pxx, 0 , _objectList.objects[i].Pxvx, 0 , 
                          0,   _objectList.objects[i].Pyy, 0 , _objectList.objects[i].Pyvy,
                          _objectList.objects[i].Pxvx, 0 , _objectList.objects[i].Pvxvx, 0,
                          0, _objectList.objects[i].Pyvy, 0 , _objectList.objects[i].Pvyvy;

                    
      
     
      F_(0, 2) = dt;
      F_(1, 3) = dt;
     
      x_ = F_ * x_;

      MatrixXd Ft = F_.transpose();
      P_ = F_ * P_ * Ft + Q_;


      
      _objectList.objects[i].x = x_(0);
      _objectList.objects[i].y = x_(1);
      _objectList.objects[i].vx = x_(2);
      _objectList.objects[i].vy = x_(3);
      _objectList.objects[i].Pxx = P_(0,0);
      _objectList.objects[i].Pyy = P_(1,1);
      _objectList.objects[i].Pvxvx = P_(2,2);
      _objectList.objects[i].Pvyvy = P_(3,3);
      _objectList.objects[i].Pxvx = P_(0,2);
      _objectList.objects[i].Pyvy = P_(1,3);
      
        
        std::cout<< "state vector after prediction "<< x_(0) <<" ,"<<x_(1)<<" , "<<x_(2) <<" ,"<<x_(3) <<std::endl;
   
      // std::cout<< "P matrix after prediction "<<P_<<std::endl;
 
      
      
      }

      

  }
  
void createNewObject(const SensorObject &sensorObject) {
     
      _objectList.objects[_currentObjectId].x =  sensorObject.x;

      _objectList.objects[_currentObjectId].y =  sensorObject.y;
      _objectList.objects[_currentObjectId].vx = sensorObject.vx;
      _objectList.objects[_currentObjectId].vy = sensorObject.vy;  
      _objectList.objects[_currentObjectId].Pxx = P_(0,0);
      _objectList.objects[_currentObjectId].Pyy = P_(1,1);
      _objectList.objects[_currentObjectId].Pvxvx = P_(2,2);
      _objectList.objects[_currentObjectId].Pvyvy = P_(3,3);
      _objectList.objects[_currentObjectId].Pxvx = P_(0,2);
      _objectList.objects[_currentObjectId].Pyvy = P_(1,3);
          
      _objectList.objects[_currentObjectId].objectId =  _currentObjectId;
     _currentObjectId++;
  }

  bool associate(const SensorObject &sensorObject,
                         uint8_t &associatedObjectIndex) {
    VectorXd x_meas(4);
    x_meas   <<   sensorObject.x,
                  sensorObject.y,
                  sensorObject.vx,
                  sensorObject.vy;

    VectorXd tempx_(4); 
    MatrixXd tempP_ (4,4);

                  
    std::vector<float> cost;
    for(int i{0}; i < _objectList.numOfValidObjects ; i++){ 

      tempx_   <<         _objectList.objects[i].x,
                          _objectList.objects[i].y,
                          _objectList.objects[i].vx,
                          _objectList.objects[i].vy;
      
      tempP_ <<           _objectList.objects[i].Pxx, 0 , _objectList.objects[i].Pxvx, 0 , 
                          0,   _objectList.objects[i].Pyy, 0 , _objectList.objects[i].Pyvy,
                          _objectList.objects[i].Pxvx, 0 , _objectList.objects[i].Pvxvx, 0,
                          0, _objectList.objects[i].Pyvy, 0 , _objectList.objects[i].Pvyvy;
      

       MatrixXd S = H_ * tempP_ * H_.transpose() + R_;
       MatrixXd Si = S.inverse();

      auto mahalanaboisDistance = sqrt((tempx_ - x_meas).transpose() * Si * (tempx_- x_meas)) + log(S.determinant());
      cost.push_back(mahalanaboisDistance);
      std::cout<<" track index " << i<<std::endl;
      std::cout<<"distance " << mahalanaboisDistance<<std::endl;  
                         }
      auto itr = std::min_element(cost.begin(),cost.end());
      associatedObjectIndex = itr - cost.begin();
      return 1;
                         
    }
 
  void update(const SensorObject &sensorObject,
                      const uint8_t associatedObjectIndex) 
     {            VectorXd z(4)  ;
                   z    <<      sensorObject.x,
                                sensorObject.y,
                                sensorObject.vx,
                                sensorObject.vy; 

                  x_   << _objectList.objects[associatedObjectIndex].x,
                          _objectList.objects[associatedObjectIndex].y,
                          _objectList.objects[associatedObjectIndex].vx,
                          _objectList.objects[associatedObjectIndex].vy;
                   

                  P_ <<   _objectList.objects[associatedObjectIndex].Pxx, 0 , _objectList.objects[associatedObjectIndex].Pxvx, 0 , 
                          0,   _objectList.objects[associatedObjectIndex].Pyy, 0 , _objectList.objects[associatedObjectIndex].Pyvy,
                          _objectList.objects[associatedObjectIndex].Pxvx, 0 , _objectList.objects[associatedObjectIndex].Pvxvx, 0,
                          0, _objectList.objects[associatedObjectIndex].Pyvy, 0 , _objectList.objects[associatedObjectIndex].Pvyvy;
      
                  
      
                  VectorXd z_pred = H_ * x_;
                  VectorXd y = z - z_pred;
                  MatrixXd Ht = H_.transpose();
                  MatrixXd S = H_ * P_ * Ht + R_;
                  MatrixXd Si = S.inverse();
                  MatrixXd PHt = P_ * Ht;
                  MatrixXd K = PHt * Si;

                  //new estimate
                  x_ = x_ + (K * y);
                  
                  long x_size = x_.size();
                  MatrixXd I = MatrixXd::Identity(x_size, x_size);
                  P_ = (I - K * H_) * P_;

                   _objectList.objects[associatedObjectIndex].x = x_(0);
                   _objectList.objects[associatedObjectIndex].y = x_(1) ;
                   _objectList.objects[associatedObjectIndex].vx = x_(2);
                   _objectList.objects[associatedObjectIndex].vy = x_(3);      
                   _objectList.objects[associatedObjectIndex].Pxx = P_(0,0);
                   _objectList.objects[associatedObjectIndex].Pyy = P_(1,1);
                   _objectList.objects[associatedObjectIndex].Pvxvx = P_(2,2);
                   _objectList.objects[associatedObjectIndex].Pvyvy = P_(3,3);
                   _objectList.objects[associatedObjectIndex].Pxvx = P_(0,2);
                   _objectList.objects[associatedObjectIndex].Pyvy = P_(1,3);
                                      
                   
                   std::cout<< "state vector after correction "<< x_(0) <<" ,"<<x_(1)<<" , "<<x_(2) <<" ,"<<x_(3) <<std::endl;
                  // std::cout<< "P matrix after correction "<<P_<<std::endl;
       
                      
                      }       

      



  void doUpdate(const SensorObjectList &sensorObjectList)  {
     
      if (!is_initialized_) {

      _objectList.numOfValidObjects = sensorObjectList.numOfValidObjects;

      _objectList.timestamp     =  sensorObjectList.timestamp;

      for(int i = 0; i< sensorObjectList.numOfValidObjects;i++ ){

        createNewObject(sensorObjectList.objectList[i]);
      
      }   
    
    //previous_timestamp_ = sensorObjectList.timestamp;
    is_initialized_ = true;
    return;
      }
  
    predict(sensorObjectList.timestamp);
                  // MatrixXd S = H_ * P_ * Ht + R_;
                  // MatrixXd Si = S.inverse();
    
   uint8_t associationIndexArray[_objectList.numOfValidObjects];
  for(int i = 0; i< sensorObjectList.numOfValidObjects;i++ )
  { 
    uint8_t index ;
    if(associate(sensorObjectList.objectList[i],index))
    {
        associationIndexArray[i] = index; 
    }
  }

  for(int i = 0;i < sensorObjectList.numOfValidObjects;i++){

      update(sensorObjectList.objectList[i],associationIndexArray[i]);

    } 
   _objectList.timestamp = sensorObjectList.timestamp;
    
}

};





#ifndef KALMANTRACKER_H_
#define KALMANTRACKER_H_ KALMANTRACKER_H_

#include"IFusionInterface.h"
#include<eigen3/Eigen/Dense>
#include "JSONFileLogger.h"
using Eigen::MatrixXd;
using Eigen::VectorXd;



class KalmanTracker : public IFusionInterface
{  
  public:
    
  KalmanTracker()
  {
      is_initialized_ = false;
      threshold_distance = 3.0;
      cycle_count = -1;
      
      // create a 4D state vector, we don't know yet the values of the x state
      x_ = VectorXd(4);

      // state covariance matrix P
      P_ = MatrixXd(4, 4);
      P_ <<     5, 0, 0, 0,
                0, 5, 0, 0,
                0, 0, 5, 0,
                0, 0, 0, 5;


      // measurement covariance
      R_ = MatrixXd(4,4);
      R_ <<     Rxx, 0,  0,   0,
                0, Ryy , 0,   0,
                0, 0 ,Rvxvx,  0,
                0, 0 , 0,  Rvyvy;
      
      // process covaraince matrix  
      Q_ = MatrixXd(4,4);
      Q_ <<     Qxx, 0,  0,   0,
                0, Qyy , 0,   0,
                0, 0 ,Qvxvx,  0,
                0, 0 , 0,  Qvyvy;


      // measurement matrix 
      H_ = MatrixXd(4, 4);
      H_ <<     1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;

      // the initial transition matrix F_
      F_ = MatrixXd(4, 4);
      F_ <<     1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;

  } 
  


  void read_state(const Object &object){
    x_ <<                 object.x,
                          object.y,
                          object.vx,
                          object.vy;
   
    }


  void read_covariance(const Object & object){
   
    P_ <<               object.Pxx,      0    ,         object.Pxvx  ,      0 , 
                              0        ,      object.Pyy    ,    0                  , object.Pyvy,
                          object.Pxvx ,      0              ,   object.Pvxvx    , 0       ,
                             0    ,            object.Pyvy,         0 ,           object.Pvyvy;
    
  }

  void write_object(Object & object){
   
      object.x = x_(0);
      object.y = x_(1);
      object.vx = x_(2);
      object.vy = x_(3);
      object.Pxx = P_(0,0);
      object.Pyy = P_(1,1);
      object.Pvxvx = P_(2,2);
      object.Pvyvy = P_(3,3);
      object.Pxvx = P_(0,2);
      object.Pyvy = P_(1,3);
   
  }
 
  void predict(const uint64_t timestamp) {//

        for (int i{0}; i< _objectList.numOfValidObjects; i++ ){

          read_state(_objectList.objects[i]);

          read_covariance(_objectList.objects[i]);
          float dt = (timestamp - _objectList.timestamp)/1000000.0;


          F_(0, 2) = dt;
          F_(1, 3) = dt;

          x_ = F_ * x_;

          MatrixXd Ft = F_.transpose();
          P_ = F_ * P_ * Ft + Q_;
          write_object(_objectList.objects[i]);
          std::cout<< "state vector after prediction "<< x_(0) <<" ,"<<x_(1)<<" , "<<x_(2) <<" ,"<<x_(3) <<std::endl;

          // std::cout<< "P matrix after prediction "<<P_<<std::endl;
        }

        }
  
void createNewObject(const SensorObject &sensorObject) {
     
      _objectList.objects[_currentObjectId].x =  sensorObject.x;
      _objectList.objects[_currentObjectId].y =  sensorObject.y;
      _objectList.objects[_currentObjectId].vx = sensorObject.vx;
      _objectList.objects[_currentObjectId].vy = sensorObject.vy;  
      _objectList.objects[_currentObjectId].Pxx = P_(0,0);
      _objectList.objects[_currentObjectId].Pyy = P_(1,1);
      _objectList.objects[_currentObjectId].Pvxvx = P_(2,2);
      _objectList.objects[_currentObjectId].Pvyvy = P_(3,3);
      _objectList.objects[_currentObjectId].Pxvx = P_(0,2);
      _objectList.objects[_currentObjectId].Pyvy = P_(1,3);
          
      _objectList.objects[_currentObjectId].objectId =  _currentObjectId;
     _currentObjectId++;
  }



  bool associate(const SensorObject &sensorObject,
                         uint8_t &associatedObjectIndex) {
    VectorXd x_meas(4);
    bool ischeck;
    x_meas   <<   sensorObject.x,
                  sensorObject.y,
                  sensorObject.vx,
                  sensorObject.vy;

    int count =0;              
    std::vector<float> cost;
    uint8_t tempIndex = associatedObjectIndex;
    for(int i{0}; i < _objectList.numOfValidObjects ; i++){ 

      
       read_state(_objectList.objects[i]);
       read_covariance(_objectList.objects[i]);
       MatrixXd S = H_ * P_ * H_.transpose() + R_;
       MatrixXd Si = S.inverse();

      auto mahalanaboisDistance = sqrt((x_ - x_meas).transpose() * Si * (x_- x_meas)) + log(S.determinant());


      if(mahalanaboisDistance > threshold_distance && cycle_count > 1){
          continue;
        }
      cost.push_back(mahalanaboisDistance);
      count++;
      associatedObjectIndex = i;
      std::cout<<" track index " << i<<std::endl;
      std::cout<<"distance " << mahalanaboisDistance<<std::endl;  
                         }
      if(count > 1 || cycle_count == 1){ 
        auto itr = std::min_element(cost.begin(),cost.end());
        associatedObjectIndex = itr - cost.begin();
        if(associatedObjectIndex == tempIndex){
          cost.erase(cost.begin()+ tempIndex);
          auto itr = std::min_element(cost.begin(),cost.end());
          associatedObjectIndex = itr - cost.begin();
          return true;
        }
      }            
      return count > 0 ? true: false;                    

         }                  
                     
    
 
  void update(const SensorObject &sensorObject,
                      const uint8_t associatedObjectIndex) 
     {            VectorXd z(4)  ;
                   z    <<      sensorObject.x,
                                sensorObject.y,
                                sensorObject.vx,
                                sensorObject.vy; 

                read_state(_objectList.objects[associatedObjectIndex]);
                read_covariance(_objectList.objects[associatedObjectIndex]);   

                  
      
                  VectorXd z_pred = H_ * x_;
                  VectorXd y = z - z_pred;
                  MatrixXd Ht = H_.transpose();
                  MatrixXd S = H_ * P_ * Ht + R_;
                  MatrixXd Si = S.inverse();
                  MatrixXd PHt = P_ * Ht;
                  MatrixXd K = PHt * Si;

                  //new estimate
                  x_ = x_ + (K * y);
                  
                  long x_size = x_.size();
                  MatrixXd I = MatrixXd::Identity(x_size, x_size);
                  P_ = (I - K * H_) * P_;

                   
                   write_object(_objectList.objects[associatedObjectIndex]);
                   
                   std::cout<< "state vector after correction "<< x_(0) <<" ,"<<x_(1)<<" , "<<x_(2) <<" ,"<<x_(3) <<std::endl;
                  // std::cout<< "P matrix after correction "<<P_<<std::endl;
           
                      }       

      



  void doUpdate(const SensorObjectList &sensorObjectList)  {
      
      cycle_count++;
      if (!is_initialized_) {

      _objectList.numOfValidObjects = sensorObjectList.numOfValidObjects;

      _objectList.timestamp     =  sensorObjectList.timestamp;

      for(int i = 0; i< sensorObjectList.numOfValidObjects;i++ ){

        createNewObject(sensorObjectList.objectList[i]);
      
      }   
    
    
        is_initialized_ = true;
         return;
         }
  
   predict(sensorObjectList.timestamp);
                  
   uint8_t associationIndexArray[_objectList.numOfValidObjects];
   uint8_t index = 100;
   
  for(int i = 0; i< sensorObjectList.numOfValidObjects;i++ )
  { 
    
    if(associate(sensorObjectList.objectList[i],index))
    {
        associationIndexArray[i] = index; 
    }
    else
        {
         createNewObject(sensorObjectList.objectList[i]);
       }
    }
  

  for(int i = 0;i < sensorObjectList.numOfValidObjects;i++){

      update(sensorObjectList.objectList[i],associationIndexArray[i]);

    } 
   _objectList.timestamp = sensorObjectList.timestamp;

}


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

    bool is_initialized_;

    // JSONFileLogger outwriter;


    //pair<int,int> association;
    ObjectList _objectList;    // object database
    uint16_t _currentObjectId; // use this one to create new object id's
    
    float threshold_distance;
    int cycle_count;
    
    float previousmincost;


};

#endif
bool KalmanTracker::associate(const SensorObject &sensorObject,
                         uint8_t &associatedObjectIndex) {
    VectorXd x_meas(4);
    
    x_meas   <<   sensorObject.x,
                  sensorObject.y,
                  sensorObject.vx,
                  sensorObject.vy;

    int count =0;              
    std::vector<float> cost;
    uint8_t tempIndex = associatedObjectIndex;
    for(int i{0}; i < _objectList.numOfValidObjects ; i++){ 

      
       read_state(_objectList.objects[i]);
       read_covariance(_objectList.objects[i]);
       MatrixXd S = H_ * P_ * H_.transpose() + R_;
       MatrixXd Si = S.inverse();

      auto mahalanaboisDistance = sqrt((x_ - x_meas).transpose() * Si * (x_- x_meas)) + log(S.determinant());


      if(mahalanaboisDistance > threshold_gating && cycle_count > 1){
          continue;
        }
      cost.push_back(mahalanaboisDistance);
      count++;
      associatedObjectIndex = i;
      std::cout<<" track index " << i<<std::endl;
      std::cout<<"distance " << mahalanaboisDistance<<std::endl;  
                         }
      if(count > 1 || cycle_count == 1){ 
        auto itr = std::min_element(cost.begin(),cost.end());
        associatedObjectIndex = itr - cost.begin();
        if(associatedObjectIndex == tempIndex){
          cost.erase(cost.begin()+ tempIndex);
          auto itr = std::min_element(cost.begin(),cost.end());
          associatedObjectIndex = itr - cost.begin();
          return true;
        }
      }            
      return count > 0 ? true: false;                    

         }

      
