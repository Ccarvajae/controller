/*
 *
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
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx){
  innermodel = new InnerModel("/home/charlie-u/robocomp/files/innermodel/simpleworld.xml");
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
    RoboCompDifferentialRobot::TBaseState bState;
    differentialrobot_proxy->getBaseState(bState);

    RoboCompLaser::TLaserData laserData = laser_proxy->getLaserData();

    innermodel->updateTransformValues("base", bState.x, 0, bState.z ,0, bState.alpha, 0);    
    
    switch(state){ //--------------------->no entra (problema state)   
      case State::INIT:
	if(t.isActive())	  
	  state = State::GOTO;
      break;
      case State::GOTO:
	gotoTarjet(laserData);
      break;
      case State::BUG:
	bug(laserData);
      break;
      case State::END:
	end();
      break;      
   }
}
  
void SpecificWorker::gotoTarjet(const TLaserData &laserData){
  
    if(obstacle(laserData) == true){
      state = State::BUG;
      return;
    }
    QVec rt = innermodel->transform("base", t.getPose(), "world");
    float dist = rt.norm2();
    float ang  = atan2(rt.x(), rt.z());
    if(dist < 100){
      state = State::INIT;
      t.setActive(true);
      return;
    }
    float adv = dist;
    float rotacion=ang;
    if(fabs(rotacion) > 0.05)
      adv = 0;
 }

void SpecificWorker::setPick(const Pick &myPick){
    qDebug()<<"Recibido myPick";
    qDebug()<<myPick.x<<myPick.z;
    t.copy(myPick.x,myPick.z);
    t.setActive(true);
    objetivo=false;
}


void SpecificWorker::bug(const TLaserData &laserData){
    
  int vr;
  int va;
  int vv;
  int v;
  
  if(targetAtSight(laserData)){
    state=State::GOTO;
    return;
  }
  
  float d=laserData[10].dist; 
  if(d>150)
     vr=-0,2; 
  
  v=200;
  
  if(d<150)
     vr=0.2;
  
  if(fabs(vv)>0,1) //vv??---->copiado en la pizarra
    v=0;
  
  differentialrobot_proxy->setSpeedBase(va,vr); //va?---->copiado en la pizarra
}

void SpecificWorker::end(){
  t.setActive(false);
}

bool SpecificWorker::obstacle(const TLaserData &laserData){
  
     const float threshold = 400; //millimeters
     float rot = 0.6;  //rads per second
    try{
        std::sort( laserData.begin()+8, laserData.end()-8, [](RoboCompLaser::TData a, RoboCompLaser::TData b){ return     a.dist < b.dist; }) ;  //sort laser data from small to large distances using a lambda function.
	
	if( laserData[8].dist < threshold){	  
	  std::cout << laserData.front().dist << std::endl;
	  differentialrobot_proxy->setSpeedBase(5, rot);
	  usleep(rand()%(1500000-100000 + 1) + 100000);  //random wait between 1.5s and 0.1sec
        }else{
	  state = State::GOTO;
        }
    }catch(const Ice::Exception &ex){
          std::cout << ex << std::endl;
    }   
}

bool SpecificWorker::targetAtSight(const TLaserData &laserData){
  
  QPolygon polygon;
  for(auto l: laserData){
    QVec lr = innermodel->laserTo("world", "laser", l.dist, l.angle);
    
    polygon << QPointF(lr.x(), lr.z());
  }
  
  QVec t3 = t.getPose();
  return  polygon.contains( QPointF(t3.x(), t3.z()));
}