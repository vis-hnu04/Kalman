#include"KalmanTracker.h"
#include<iostream>

    KalmanTracker::KalmanTracker(){
        _isInitialized = false;
        _gatingThreshold = 10.0;
        _logger.init();  

        // create a 4D state vector
        x_ = Matrix(4,1,1);

        // state covariance matrix P
        P_ = Matrix(4, 4,0);
        P_.createDiagonalMatrix(5);

        // measurement covariance
        std::vector<vector<float>>measurememnt {{Rxx,0,0,0}, {0,Ryy,0,0},{0,0,Rvxvx,0},{0,0,0,Rvyvy}};
        std::vector<vector<float>>process {{Qxx,0,0,0}, {0,Qyy,0,0},{0,0,Qvxvx,0},{0,0,0,Qvyvy}};

        R_ = Matrix(measurememnt);

        // process covaraince matrix  
        Q_ = Matrix(process);

        // measurement matrix 
        H_ = Matrix (4, 4,0);
        H_.createIdentityMatrix();

        Ht_ = Matrix (4, 4,0);
        Ht_ = H_.transpose();



        // the initial transition matrix F_
        F_ = Matrix (4, 4,0);
        F_.createIdentityMatrix(); 

    }

    KalmanTracker::~KalmanTracker(){
       _logger.close();
    }

    void KalmanTracker::getState(const Object & object)
    {
        x_(0,0)   =           object.x;
        x_(1,0)   =           object.y;
        x_(2,0)   =           object.vx;
        x_(3,0)   =           object.vy;

    }

    void KalmanTracker::getCovariance(const Object &object)
    {

        P_(0,0)   =    object.Pxx;           
        P_(0,2)   =    object.Pxvx; 
        P_(1,1)   =    object.Pyy;                        
        P_(1,3)   =    object.Pyvy;           
        P_(2,0)   =    object.Pxvx; 
        P_(2,2)   =    object.Pvxvx; 
        P_(3,1)   =    object.Pyvy; 
        P_(3,3)   =    object.Pvyvy;                    
                          
    }

    Matrix KalmanTracker::getMeasurement(const SensorObject &sensorObject ){
        Matrix detection(4,1,0);
        detection(0,0) =    sensorObject.x,
        detection(1,0) =    sensorObject.y,
        detection(2,0) =    sensorObject.vx,
        detection(3,0) =    sensorObject.vy;

        return detection; 
    }


    void KalmanTracker::writeObject(Object & object){

        object.x = x_(0,0);
        object.y = x_(1,0);
        object.vx = x_(2,0);
        object.vy = x_(3,0);

        object.Pxx = P_(0,0);
        object.Pyy = P_(1,1);
        object.Pvxvx = P_(2,2);
        object.Pvyvy = P_(3,3);
        object.Pxvx = P_(0,2);
        object.Pyvy = P_(1,3);

    }

    void KalmanTracker::logwriter(const SensorObjectList &sensorObjectList) 
    {

        _logger.openNewFusionCycleArray();
        _logger.addElement(sensorObjectList.timestamp,TIMESTAMP);
        _logger.addSensorObjectList(sensorObjectList);
        _logger.addObjectList(_objectList,PREDICTION);
        _logger.addAssociationIndices(_associationArray);
        _logger.addObjectList(_objectList,UPDATED);
        _logger.closeFusionCycleArray();
        _associationArray.clear();


    }                           

    void KalmanTracker::createNewObject(const SensorObject &sensorObject) {

        _objectList.objects[_currentObjectId].x     =  sensorObject.x;
        _objectList.objects[_currentObjectId].y     =  sensorObject.y;
        _objectList.objects[_currentObjectId].vx    = sensorObject.vx;
        _objectList.objects[_currentObjectId].vy    = sensorObject.vy;  
        _objectList.objects[_currentObjectId].Pxx   = P_(0,0);
        _objectList.objects[_currentObjectId].Pyy   = P_(1,1);
        _objectList.objects[_currentObjectId].Pvxvx = P_(2,2);
        _objectList.objects[_currentObjectId].Pvyvy = P_(3,3);
        _objectList.objects[_currentObjectId].Pxvx  = P_(0,2);
        _objectList.objects[_currentObjectId].Pyvy  = P_(1,3);

        _objectList.objects[_currentObjectId].objectId =  _currentObjectId;
        _currentObjectId++;
    }


    void KalmanTracker::predict(const uint64_t timestamp) 
    {
        for (int i{0}; i< _objectList.numOfValidObjects; i++ ){

        getState(_objectList.objects[i]);
        getCovariance(_objectList.objects[i]);

        float dt = (timestamp - _objectList.timestamp)/1000000.0;

        F_(0, 2) = dt;
        F_(1, 3) = dt;

        x_ = F_ * x_;
        Matrix Ft = F_.transpose();
        P_ = F_ * P_ * Ft + Q_;

        writeObject(_objectList.objects[i]);
        std::cout<< "state vector after prediction "<< x_(0,0) <<" ,"<<x_(1,0)<<" , "<<x_(2,0) <<" ,"<<x_(3,0) <<std::endl;
    }
    }


    void KalmanTracker::findOptimalAssignment(std::vector<std::vector<float>> &costMatrix){
        double infinity = std::numeric_limits<double>:: max();
        float minimum = infinity; 
        int column = -1;
        int row = -1;
        size_t numObservations = costMatrix.size();

        while(numObservations){
         for (int index =0; index <  costMatrix.size();index++){    
            auto itr = std::min_element(costMatrix[index].begin(), costMatrix[index].end());

            if(*itr < minimum & *itr < _gatingThreshold){

              minimum = *itr;
              column =  itr - costMatrix[index].begin();
              row = index;
            } // if
          }// for index
         
          if(row != -1){ 
           for (int i {0};i< costMatrix.size();i++){

              costMatrix[i][column]=infinity;
              costMatrix[row][i] = infinity;
             }
          _associationArray.push_back({row,column});

        }

          else{
          _associationArray.push_back({numObservations, -1});
        
          }

          minimum = infinity;
          numObservations--;
        }
    }


    bool KalmanTracker::associate(const SensorObject &sensorObject,
        uint8_t &associatedObjectIndex) {
        Matrix x_meas(4,1,0);

        x_meas = getMeasurement(sensorObject);

        for(int i{0}; i < _objectList.numOfValidObjects ; i++){ 

          getState(_objectList.objects[i]);
          getCovariance(_objectList.objects[i]);


          Matrix S =  H_*P_* Ht_ + R_;
          Matrix Si = S.inverse();

          Matrix residual = (x_ - x_meas);
          Matrix residual_t =residual.transpose();

          Matrix product = residual_t*Si* residual;
          auto mahalanaboisDistance = sqrt(product(0,0)) + log(S.det());
          _costMatrix.push_back(mahalanaboisDistance);

        }

        return std::any_of(_costMatrix.begin(),_costMatrix.end(),[&](int i) {
            return i < _gatingThreshold; }
                ) ;                  
        }  


    void KalmanTracker::update(const SensorObject &sensorObject,
      const uint8_t associatedObjectIndex) 
    {            
        Matrix z(4,1,0);

        z = getMeasurement(sensorObject);


        getState(_objectList.objects[associatedObjectIndex]);
        getCovariance(_objectList.objects[associatedObjectIndex]);   

        Matrix z_pred = H_ * x_;
        Matrix y = z - z_pred;


        Matrix S =  H_*P_* Ht_ ;//+ R_;
        Matrix Si = S.inverse();

        Matrix k = P_ * Ht_*Si;
        x_ = (k*y) + x_;

        long x_size = x_.getRows();
        Matrix I(x_size,x_size,0) ;
        I.createIdentityMatrix();
        Matrix kH_ = k* H_;
        P_ = (I - kH_) * P_;


        writeObject(_objectList.objects[associatedObjectIndex]);

        std::cout<< "state vector after correction "<< x_(0,0) <<" ,"<<x_(1,0)<<" , "<<x_(2,0) <<" ,"<<x_(3,0) <<std::endl;
        // std::cout<< "P matrix after correction "<<P_<<std::endl;

        } 


    void KalmanTracker::doUpdate(const SensorObjectList &sensorObjectList) 
    {    
        uint8_t sensorObjectAssociationId[sensorObjectList.numOfValidObjects];

        std::vector<std::vector<float>> assignmentMatrix;

        if (!_isInitialized) {

          _objectList.numOfValidObjects = sensorObjectList.numOfValidObjects;
          _objectList.timestamp     =  sensorObjectList.timestamp;

        for(int i = 0; i< sensorObjectList.numOfValidObjects;i++ ){

          createNewObject(sensorObjectList.objectList[i]);
          std::pair<int,int> ObjectTrackPair(i,-1);
          _associationArray.push_back(ObjectTrackPair);

        }   
        _isInitialized = true;
        logwriter(sensorObjectList);
        return;
        }

        predict(sensorObjectList.timestamp);

        for(int i = 0; i< sensorObjectList.numOfValidObjects;i++ )
        { 

          if(associate(sensorObjectList.objectList[i], sensorObjectAssociationId[i]))
          {
            assignmentMatrix.push_back(_costMatrix);
           _costMatrix.clear();
          }
          else
          { 
          createNewObject(sensorObjectList.objectList[i]);
          _associationArray.push_back({i,-1});

         }
        }
        findOptimalAssignment(assignmentMatrix);
        // HungarianAlgorithm ha(assignmentMatrix);       

        // _associationArray = ha.solve();                              //changed
        for (auto values: _associationArray){
          sensorObjectAssociationId[values.first] = values.second;
          std::cout<<values.first<< " observation mapped to " <<values.second<<std::endl; 

        }

        for(int i = 0;i < sensorObjectList.numOfValidObjects;i++){

          if (sensorObjectAssociationId[i] == 255){
          continue;
          }     
          update(sensorObjectList.objectList[i],sensorObjectAssociationId[i]);

        }
        logwriter(sensorObjectList);  
        _objectList.timestamp = sensorObjectList.timestamp;

    }

