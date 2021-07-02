#include"KalmanTracker.h"
#include<iostream>

KalmanTracker::KalmanTracker(){
      is_initialized_ = false;
      threshold_gating = 10.0;
      _logger.init();  

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

      Ht = MatrixXd(4, 4);
      Ht = H_.transpose();



      // the initial transition matrix F_
      F_ = MatrixXd(4, 4);
      F_ <<     1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;


}

KalmanTracker::~KalmanTracker(){
  _logger.close();
}

void KalmanTracker::read_state(const Object & object)
{
      x_ <<               object.x,
                          object.y,
                          object.vx,
                          object.vy;
   
}

void KalmanTracker::read_covariance(const Object &object)
{
   
    P_ <<               object.Pxx ,  0 ,  object.Pxvx  ,   0 , 
                              0     ,  object.Pyy  ,  0   , object.Pyvy,
                          object.Pxvx ,  0      ,   object.Pvxvx  , 0  ,
                             0  ,  object.Pyvy ,    0 ,  object.Pvyvy;
    
}



void KalmanTracker::write_object(Object & object){
   
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

void KalmanTracker::predict(const uint64_t timestamp) 
{
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
        }
 }


 void KalmanTracker::findMinimum(std::vector<std::vector<float>> &costMatrix){
   double infinity = std::numeric_limits<double>:: max();
   float minimum = infinity; 
   int column;
   int row;
   size_t num_observations = costMatrix.size();
   while(num_observations){
      for (int index =0; index <  costMatrix.size();index++){    
          auto itr = std::min_element(costMatrix[index].begin(), costMatrix[index].end());
          if(*itr < minimum){
            
            minimum = *itr;
            column =  itr - costMatrix[index].begin();
            row = index;
        }
   }
      for (int i {0};i< costMatrix.size();i++){
         costMatrix[i][column]=infinity;
         costMatrix[row][i] = infinity;
     }
      associationArray.push_back({row,column});
      minimum = infinity;
      num_observations--;
   }
 }



 void KalmanTracker::createNewObject(const SensorObject &sensorObject) {
     
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


bool KalmanTracker::associate(const SensorObject &sensorObject,
                         uint8_t &associatedObjectIndex) {
    VectorXd x_meas(4);
    
    x_meas   <<   sensorObject.x,
                  sensorObject.y,
                  sensorObject.vx,
                  sensorObject.vy;


    for(int i{0}; i < _objectList.numOfValidObjects ; i++){ 

       read_state(_objectList.objects[i]);
       read_covariance(_objectList.objects[i]);
       MatrixXd S = H_ * P_ * H_.transpose() + R_;
       MatrixXd Si = S.inverse();
 
      auto mahalanaboisDistance = sqrt((x_ - x_meas).transpose() * Si * (x_- x_meas)) + log(S.determinant());
      std::cout<<"track id: "<< i <<" distance :"<<mahalanaboisDistance<<std::endl;
      _costMatrix.push_back(mahalanaboisDistance);

        }

      return std::any_of(_costMatrix.begin(),_costMatrix.end(),[&](int i) {
                          return i < threshold_gating; }
                             ) ;                  
      }  


 void KalmanTracker::update(const SensorObject &sensorObject,
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
                   
                   MatrixXd S = H_ * P_ * H_.transpose() + R_;
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




void KalmanTracker::logwriter(const SensorObjectList &sensorObjectList) 
{

          _logger.openNewFusionCycleArray();
          _logger.addElement(sensorObjectList.timestamp,TIMESTAMP);
          _logger.addSensorObjectList(sensorObjectList);
          _logger.addObjectList(_objectList,PREDICTION);
          _logger.addAssociationIndices(associationArray);
          _logger.addObjectList(_objectList,UPDATED);
          _logger.closeFusionCycleArray();
           associationArray.clear();
  

}                           
                     
void KalmanTracker::doUpdate(const SensorObjectList &sensorObjectList) 
 {    
       uint8_t associationIndexArray[_objectList.numOfValidObjects];
   
      std::vector<std::vector<float>> assignmentMatrix;

      uint8_t index;


       if (!is_initialized_) {
         
          _objectList.numOfValidObjects = sensorObjectList.numOfValidObjects;
          _objectList.timestamp     =  sensorObjectList.timestamp;
          
          for(int i = 0; i< sensorObjectList.numOfValidObjects;i++ ){
           
            createNewObject(sensorObjectList.objectList[i]);
            std::pair<int,int> ObjectTrackPair(i,-1);
            associationArray.push_back(ObjectTrackPair);
 
           }   
          is_initialized_ = true;
          logwriter(sensorObjectList);

          return;
         }
  
     predict(sensorObjectList.timestamp);

     for(int i = 0; i< sensorObjectList.numOfValidObjects;i++ )
    { 
    
         if(associate(sensorObjectList.objectList[i],index))
       {
             assignmentMatrix.push_back(_costMatrix);
            _costMatrix.clear();
           }
         else
          {
            createNewObject(sensorObjectList.objectList[i]);
            associationArray.push_back({i,-1});

             }
    }
  
     findMinimum(assignmentMatrix);
    // HungarianAlgorithm ha(assignmentMatrix);       

 //   associationArray = ha.init();                              //changed
  

      for (auto values: associationArray){
        associationIndexArray[values.first] = values.second;
        std::cout<<values.first<< " observation mapped to " <<values.second<<std::endl; 

      }

   
      for(int i = 0;i < sensorObjectList.numOfValidObjects;i++){

            update(sensorObjectList.objectList[i],associationIndexArray[i]);

       }
       logwriter(sensorObjectList);  
      _objectList.timestamp = sensorObjectList.timestamp;

}

