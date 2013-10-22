/* -*- Mode:C++; c-basic-offset: 2; tab-width:2, indent-tabs-width:t -*- 
 * Copyright (C) 2005 State University of New York, at Binghamton
 * All rights reserved.
 *
 * NOTICE: This software is provided "as is", without any warranty,
 * including any implied warranty for merchantability or fitness for a
 * particular purpose.  Under no circumstances shall SUNY Binghamton
 * or its faculty, staff, students or agents be liable for any use of,
 * misuse of, or inability to use this software, including incidental
 * and consequential damages.

 * License is hereby given to use, modify, and redistribute this
 * software, in whole or in part, for any commercial or non-commercial
 * purpose, provided that the user agrees to the terms of this
 * copyright notice, including disclaimer of warranty, and provided
 * that this copyright notice, including disclaimer of warranty, is
 * preserved in the source code and documentation of anything derived
 * from this software.  Any redistributor of this software or anything
 * derived from this software assumes responsibility for ensuring that
 * any parties to whom such a redistribution is made are fully aware of
 * the terms of this license and disclaimer.
 *
 * Author: Ke Liu, CS Dept., State University of New York, at Binghamton 
 * October, 2005
 *
 * GPSR code for NS2 version 2.26 or later
 * Note: this implementation of GPSR is different from its original 
 *       version wich implemented by Brad Karp, Harvard Univ. 1999
 *       It is not guaranteed precise implementation of the GPSR design
 */

/* gpsr.h : The head file for the GPSR routing agent, defining the 
 *          routing agent, methods (behaviors) of the routing 
 *          
 * Note: the routing table (local neighborhood) information is kept 
 *       in another class gpsr_neighbor which is defined in 
 *       gpsr_neighbor{.h, .cc}. So the planarizing and next hop deciding
 *       is made there, not in this file
 *
 */

#ifndef GPSR_ROUTING_H_
#define GPSR_ROUTING_H_

#include "config.h"
#include "agent.h"
#include "ip.h"
#include "address.h"
//#include "scheduler.h"
#include "timer-handler.h"
#include "mobilenode.h"
#include "tools/random.h"
#include "packet.h"
#include "trace.h"
#include "classifier-port.h"
#include "cmu-trace.h"

#include "gpsr_packet.h"
#include "gpsr_newneighbor.h"
#include "gpsr_newsinklist.h"
#include "math.h"

#include "gpsr_matrixclac.h"

#define MAXNODE_ 1000
#define MAX_DIS 100000
#define MAX_ARRAY 100

class GPSRAgent;

// Timer:RtsTimer  

class GPSRrtsTimer : public TimerHandler {
public:
  GPSRrtsTimer(GPSRAgent *a,int keyID,u_int8_t keyType) : TimerHandler() {a_=a;timerID = keyID;disOrAng = MAX_DIS;type_ = keyType;}
  void update(double keyDis,nsaddr_t keyID){
    if (keyDis < disOrAng){
      disOrAng = keyDis;
      nextHop = keyID;
    }
  }
  double getDisOrAng(){return disOrAng;}
  nsaddr_t getNextHop(){return nextHop;}
  
protected:
  virtual void expire(Event *e);
  GPSRAgent *a_;
  double disOrAng;
	u_int8_t type_;
  nsaddr_t nextHop;
};


class GPSRctsTimer : public TimerHandler {
public:
	GPSRctsTimer(GPSRAgent *a,Packet* keyP) : TimerHandler() {a_=a;p = keyP;}
protected:
	virtual void expire(Event *e);
	Packet* p;
	GPSRAgent *a_;
};
//-------------------------------------------------------------


// Agent main part

class GPSRAgent : public Agent {
private:

  friend class GPSRrtsTimer;
	friend class GPSRctsTimer;


  GPSRrtsTimer* timerArray[MAX_ARRAY];
	int arrayHead,arrayTail;
	int idPointer;
	

  MobileNode *node_;             //the attached mobile node
  PortClassifier *port_dmux_;    //for the higher layer app de-multiplexing
	double waitTime;
  
  nsaddr_t my_id_;               //node id (address), which is NOT necessary
  double my_x_;                  //node location info, fatal info
  double my_y_;                  //     obtained from the attached node*/

  int planar_type_; //1=GG planarize, 0=RNG planarize
  void GetLocation(double*, double*); //called at initial phase
  RNG randSend_;
  
  

  /**
   * The below variables and functions are used for 
   * localization algorithms
   */
  double localized_x_; 
  double localized_y_;
  void dvhop();

protected:
  Trace *tracetarget;              //for routing agent special trace
  void trace(char *fmt,...);       //   Not necessary 

  /*  void hellotout();                //called by timer::expire(Event*)
  void querytout();
	void matrixtout();
	void energytout();
	void sinktout();*/
	int queryFinish;
public:
  GPSRAgent();

  int command(int, const char*const*);
  void recv(Packet*, Handler*);         //inherited virtual function

};

#endif
