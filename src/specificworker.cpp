/*
 *    Copyright (C) 2016 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
  innermodel = new InnerModel("/home/salabeta/robocomp/files/innermodel/simpleworld.xml");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker(){
	
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params){

	timer.start(Period);	
	return true;
}

void SpecificWorker::compute()
{
  try
  {
  
    RoboCompLaser::TLaserData ldata = laser_proxy->getLaserData();  //read laser data    
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);     
    innermodel->updateTransformValues("base", bState.x, 0, bState.z, 0, bState.alpha, 0);
        
  
    if(t.isActive()){
      qDebug()<<"hola";
      goTo();       
    }
  }
  catch(const Ice::Exception &ex) { std::cout  << ex << std::endl;}
}  

void SpecificWorker::goTo(){
 
    qDebug()<<"GOTO";
   
    QVec vec = innermodel->transform("base", t.getPose(), "world");
   
    float angulo = atan2(vec.x(), vec.z());
   
    float dist = vec.norm2();
   
    if(dist < 100){
      differentialrobot_proxy->stopBase();
      qDebug()<<"FINNNN";
    }else{
      float adv = dist;
      float rotacion = angulo;
      
      if(fabs(angulo) > 0.05)	
	adv = 0;     
       
      differentialrobot_proxy->setSpeedBase(adv, rotacion);
   }          
}

void SpecificWorker::setPick(const Pick &myPick){
 
    qDebug()<<"Recibido myPick";
    qDebug()<<myPick.x<<myPick.z;
    t.copy(myPick.x,myPick.z);
    t.setActive(true);
    objetivo=false;
}

void SpecificWorker::bug()

{

}

bool SpecificWorker::obstacle()

{

}

bool SpecificWorker::targetAtSight()

{

}
