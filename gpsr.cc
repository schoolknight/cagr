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

/* gpsr.cc : the definition of the gpsr routing agent class
 *           
 */
#include "gpsr.h"




int hdr_gpsr::offset_;

static class GPSRHeaderClass : public PacketHeaderClass{
public:
  GPSRHeaderClass() : PacketHeaderClass("PacketHeader/gpsr",
					 sizeof(hdr_all_gpsr)){
    bind_offset(&hdr_gpsr::offset_);
  }
}class_gpsrhdr;





static class GPSRAgentClass : public TclClass {
public:
  GPSRAgentClass() : TclClass("Agent/gpsr"){}
  TclObject *create(int, const char*const*){
    return (new GPSRAgent());
  }
}class_gpsr;

void
GPSRHelloTimer::expire(Event *e){
  a_->hellotout();
}

void
GPSRQueryTimer::expire(Event *e){
  a_->querytout();
}

void
GPSRMatrixTimer::expire(Event *e){
	a_->matrixtout();
}

void
GPSREnergyTimer::expire(Event *e){
	a_->energytout();
}

void
GPSRSinkTimer::expire(Event *e){
	a_->sinktout();
}

void
GPSRAgent::hellotout(){
  hellomsg();
  hello_timer_.resched(hello_period_);
}

void
GPSRAgent::setsink(){
	sink_list_->newSink(my_id_,yNew);
}
void
GPSRAgent::sinkon(){
	//printf("sinkon  %d \n",my_id_);
    sinktout();
}

void
GPSRAgent::sinkoff(){
	sink_timer_.resched(INFINITE_DELAY);
}

void
GPSRAgent::querytout(){
	if (queryFinish < 4){
		query();
		query_counter_++;
		query_timer_.resched(query_period_);
	}
}


void
GPSRAgent::matrixtout(){
	for(int i = 0;i < 4; i++){
		if (sAnchor[i] >= 0 || anchor[i] == my_id_){
			matrixmsg(anchor[i],sAnchor[i]);
			matrix_counter_ ++;
		}
	}
	matrix_timer_.resched(matrix_period_);

}
void
GPSRAgent::energytout(){
	for(int i = 0;i < 4;i ++)
		if (anchor[i] > 0){
			energymsg(anchor[i],anchorEnergy[i]);
			energy_counter_ ++;
		}
	energy_timer_.resched(energy_period_);
}

void
GPSRAgent::sinktout(){
	//printf("sinktout \n");
	struct NewListEntry* tmpE = sink_list_->begin();
	while(tmpE != NULL){
		sinkmsg(tmpE->id_,tmpE->y_);
		sink_counter_ ++;
		tmpE = sink_list_->getNext();
	
	}
	sink_timer_.resched(sink_period_);
}
	




/*
void
GPSRAgent::getLoc(){
  GetLocation(&my_x_, &my_y_);
}
*/
void
GPSRAgent::GetLocation(double *x, double *y){
  double pos_x_, pos_y_, pos_z_;
  node_->getLoc(&pos_x_, &pos_y_, &pos_z_);
  *x=pos_x_;
  *y=pos_y_;

}


GPSRAgent::GPSRAgent() : Agent(PT_CAGR),	 hello_timer_(this), query_timer_(this),matrix_timer_(this),energy_timer_(this),sink_timer_(this),
                         my_id_(-1),  waitTime(INFINITE_DELAY),maxRange(INFINITE_DELAY)
						
{
  GetLocation(&my_x_,&my_y_);
  arrayHead = 0;
  arrayTail = 0;
  idPointer = 0;
  
  for(int i=0; i<5; i++)
    randSend_.reset_next_substream();
  
}

void
GPSRAgent::sinkRecv(Packet *p){
  FILE *fp = fopen(SINK_TRACE_FILE, "a+");
  struct hdr_cmn *cmh = HDR_CMN(p);
  struct hdr_ip *iph = HDR_IP(p);
  //  struct hdr_gpsr_data *gdh = HDR_GPSR_DATA(p);

  fprintf(fp, "%2.f\t%d\t%d\n", GPSR_CURRENT,
	  iph->saddr(), cmh->num_forwards());
  fclose(fp);
}




GPSRrtsTimer**
GPSRAgent::newTimer(){
	 arrayTail = (arrayTail + 1) % MAX_ARRAY;
	 if (arrayTail == arrayHead){
		  printf("array fall\n");
		  return NULL;
	 }
	 return &timerArray[arrayTail];
}



double
GPSRAgent::clacDis(double x1,double x2,double y1,double y2){
  return sprt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));

}


void
GPSRAgent::recvRTSD(Packet* keyP){
  if (my_id_ < 0) return;
  struct hdr_cmn *keyCmh = HDR_CMN(keyP);
  struct hdr_ip *keyIph = HDR_IP(keyP);
  struct hdr_gpsr_rtsd *keyGrdh = HDR_GSPR_RTSD(keyP);

  double my_dis = clacDis(my_x_,my_y_,grdh->tar_x,grdh->tar_y);
  if (my_dis > keyGrdh->sour_dis)
    return;

  Packet* p = allocpkt();
 
  struct hdr_cmn *cmh = HDR_CMN(p);
  struct hdr_ip *iph = HDR_IP(p);
  struct hdr_gpsr_cts *gch = HDR_GPSR_CTS(p);

  cmh->next_hop_ = keyIph->saddr();
  cmh->last_hop_ = my_id_;
  cmh->addr_type_ = NS_AF_INET;
  cmh->ptype() = PT_GPSR;
  cmh->size() = IP_HDR_LEN + gch->size();
  
  iph->daddr() = keyIph->saddr();
  iph->saddr() = my_id_;
  iph->sport() = RT_PORT;
  iph->dport() = RT_PORT;
  iph->ttl_ = 1;
  
  cts->timerID = keyGrdh->timerID;
  cts->res = my_dis;
  cts->type_ = GPSRTYPE_CTS;

  GPSRctsTimer* tmpCts = new GPSRctsTimer(this,p);
  tmpCts->resched((1.0-(keyGrdh->sour_dis-my_dis)/maxRange)*waitTime);

}

void
GPSRAgent::recvRTSA(Packet* keyP){
  if (my_id_ < 0) return;
  struct hdr_cmn *keyCmh = HDR_CMN(keyP);
  struct hdr_ip *keyIph = HDR_IP(keyP);
  struct hdr_gpsr_rtsa *keyGrah = HDR_GSPR_RTSA(keyP);

  
  Packet* p = allocpkt();
 
  struct hdr_cmn *cmh = HDR_CMN(p);
  struct hdr_ip *iph = HDR_IP(p);
  struct hdr_gpsr_cts *gch = HDR_GPSR_CTS(p);

  cmh->next_hop_ = keyIph->saddr();
  cmh->last_hop_ = my_id_;
  cmh->addr_type_ = NS_AF_INET;
  cmh->ptype() = PT_GPSR;
  cmh->size() = IP_HDR_LEN + gch->size();
  
  iph->daddr() = keyIph->saddr();
  iph->saddr() = my_id_;
  iph->sport() = RT_PORT;
  iph->dport() = RT_PORT;
  iph->ttl_ = 1;
  
  cts->timerID = keyGrdh->timerID;
  cts->res = my_dis;
  cts->type_ = GPSRTYPE_CTS;

  GPSRctsTimer* tmpCts = new GPSRctsTimer(this,p);
  tmpCts->resched((1.0-(keyGrdh->sour_dis-my_dis)/maxRange)*waitTime);

}


void
GPSRAgent::sendRTSD(double tar_x,double tar_y){
	if (my_id_ < 0 ) return;
	Packet *p = allocpkt();
	struct hdr_cmn *cmh = HDR_CMN(p);
	struct hdr_ip *iph = HDR_IP(p);
	struct hdr_gpsr_rtsd *grdh = HDR_GPSR_RTSD(p);

	cmh->next_hop_ = IP_BROADCAST;
	cmh->last_hop_ = my_id_;
	cmh->addr_type_ = NS_AF_INET;
	cmh->ptype() = PT_GPSR;
	cmh->size() = IP_HDR_LEN + ghh->size();
  
	iph->daddr() = IP_BROADCAST;
	iph->saddr() = my_id_;
	iph->sport() = RT_PORT;
	iph->dport() = RT_PORT;
	iph->ttl_ = IP_DEF_TTL;

	grdh->type_ = GPSRTYPE_RTSD;
        grdh->timerID = idPointer + (arrayTail - arrayHead + MAX_ARRAY) % MAX_ARRAY; 
        grdh->sour_dis = clacDis(tar_x,tar_y,my_x_,my_y_);
	grdh->tar_x = tar_x;
        grdh->tar_y = tar_y;
        send(p,0);
}

void
GPSRAgent::sendRTSA(double tar_x,double tar_y){
	if (my_id_ < 0 ) return;
	Packet *p = allocpkt();
	struct hdr_cmn *cmh = HDR_CMN(p);
	struct hdr_ip *iph = HDR_IP(p);
	struct hdr_gpsr_rtsd *grah = HDR_GPSR_RTSA(p);

	cmh->next_hop_ = IP_BROADCAST;
	cmh->last_hop_ = my_id_;
	cmh->addr_type_ = NS_AF_INET;
	cmh->ptype() = PT_GPSR;
	cmh->size() = IP_HDR_LEN + ghh->size();
  
	iph->daddr() = IP_BROADCAST;
	iph->saddr() = my_id_;
	iph->sport() = RT_PORT;
	iph->dport() = RT_PORT;
	iph->ttl_ = IP_DEF_TTL;

	grah->type_ = GPSRTYPE_RTSA;
        grah->timerID = idPointer + (arrayTail - arrayHead + MAX_ARRAY) % MAX_ARRAY; 
        grah->sour_x = my_x_;
        grah->sour_y = my_y_;
	grah->tar_x = tar_x;
        grah->tar_y = tar_y;
        sead(p,0);
}



void 
GPSRAgent::forwardData(Packet *p){
  struct hdr_cmn *cmh = HDR_CMN(p);
  struct hdr_ip *iph = HDR_IP(p);

  printf("forword..... %d \n",my_id_);
  if(cmh->direction() == hdr_cmn::UP && ((nsaddr_t)iph->daddr() == IP_BROADCAST || iph->daddr() == my_id_)){
	  sinkRecv(p);
	  // printf("receive\n");
	  port_dmux_->recv(p, 0);
	  return;
  }else{
    struct hdr_gpsr_data *gdh=HDR_GPSR_DATA(p);
    GPSRrtsTimer** tmpTimer = newTimer();
    *tmpTimer = new GSPRrtsTimer(this,my_id_,gdh->mode_);
    if (tmpTimer != NULL){
      if(gdh->mode_ == GPSR_MODE_GF)
        sendRTSD(gdh->tar_x,gdh->tar_y);
      else
        sendRTSA(gdh->tar_x,gdh->tar_y);
      *tmpTimer->resched(waitTime);
  
    }
  }
}



void
GPSRAgent::recv(Packet *p, Handler *h){
  struct hdr_cmn *cmh = HDR_CMN(p);
  struct hdr_ip *iph = HDR_IP(p);
  // printf("receive %d \n",my_id_);
  if(iph->saddr() == my_id_){//a packet generated by myself
	  if(cmh->num_forwards() == 0){
		  // printf("1\n");
		  printf("receive send %d  %d\n",my_id_,iph->daddr());
		  struct hdr_gpsr_data *gdh = HDR_GPSR_DATA(p);
		  cmh->size() += IP_HDR_LEN + gdh->size();

		  gdh->type_ = GPSRTYPE_DATA;
		  gdh->mode_ = GPSR_MODE_GF;
	   
		  double keyY[3];
		  if (sink_list_->getLocID(iph->daddr(),keyY)){
			  gdh->dsty_0 = keyY[0];
			  gdh->dsty_1 = keyY[1];
			  gdh->dsty_2 = keyY[2];
			  //  printf("get sink\n");
		  }else{
			  drop(p,"NoSink");
			  return;
		  }
		  gdh->prex_0 = xNew[0];
		  gdh->prex_1 = xNew[1];
		  gdh->prex_2 = xNew[2];
		  
		  gdh->ts_ = (float)GPSR_CURRENT;
	  }
	  else if(cmh->num_forwards() > 0){ //routing loop
		  //  printf("throw");
		  if(cmh->ptype() != PT_CAGR)
			  drop(p, DROP_RTR_ROUTE_LOOP);
		  else Packet::free(p);
		  return;
	  }
  }
  if(cmh->ptype() == PT_CAGR){
	  //  printf("2\n");
	  struct hdr_gpsr *gh = HDR_GPSR(p);
	  switch(gh->type_){
	  case GPSRTYPE_HELLO:
		  recvHello(p);
		  break;
	  case GPSRTYPE_QUERY:
		  recvQuery(p);
		  break;
	  case GPSRTYPE_ENERGY:
		  recvEnergy(p);
		  break;
	  case GPSRTYPE_MATRIX:
		  recvMatrix(p);
		  break;
	  case GPSRTYPE_SINK:
		  recvSink(p);
		  break;
	  default:
		  printf("Error with gf packet type.\n");
		  exit(1);
	  }
  }else {
	  // printf("3\n");
	  iph->ttl_--;
	  if(iph->ttl_ == 0){
		  //printf("drop \n");
		  drop(p, DROP_RTR_TTL);
		  return;
	  }
	  forwardData(p);
  }
}



void 
GPSRAgent::trace(char *fmt, ...){
  va_list ap;
  if(!tracetarget)
    return;
  va_start(ap, fmt);
  vsprintf(tracetarget->pt_->buffer(), fmt, ap);
  tracetarget->pt_->dump();
  va_end(ap);
}

int
GPSRAgent::command(int argc, const char*const* argv){
	//	printf("command %d %s\n",my_id_,argv[1]);
   if(argc==2){
	   /*if(strcasecmp(argv[1], "getloc")==0){
      getLoc();
      return TCL_OK;
	  }*/
    if(strcasecmp(argv[1], "sinkon")==0){
      sinkon();
      return TCL_OK;
    }
	 if(strcasecmp(argv[1], "sinkoff")==0){
      sinkoff();
      return TCL_OK;
    }

    if(strcasecmp(argv[1], "helloon")==0){
      hellotout();
      return TCL_OK;
    }
    
    if(strcasecmp(argv[1], "hellooff")==0){
      hellooff();
      return TCL_OK;
    }

    if(strcasecmp(argv[1], "queryon")==0){
		queryon();
		return TCL_OK;
    }
	
    if(strcasecmp(argv[1], "queryoff")==0){
		queryoff();
		return TCL_OK;
    }
	if(strcasecmp(argv[1], "matrixon")==0){
		matrixInit();
		matrixon();
		return TCL_OK;
    }
	if(strcasecmp(argv[1], "matrixoff")==0){
		matrixoff();
		return TCL_OK;
    }
	
	if(strcasecmp(argv[1], "setsink")==0){
		setsink();
		return TCL_OK;
    }

    /*if(strcasecmp(argv[1], "neighborlist")==0){
      nblist_->dump();
      return TCL_OK;
	  }*/
    /*if(strcasecmp(argv[1], "sinklist")==0){
      sink_list_->dump();
      return TCL_OK;
	  }*/
	if(strcasecmp(argv[1], "printans") == 0){
		printAns();
		return TCL_OK;
	}
	if(strcasecmp(argv[1], "energyon") == 0){
		energyon();
		return TCL_OK;
	}
	if(strcasecmp(argv[1], "energyoff") == 0){
		energyoff();
		return TCL_OK;
	}
	
  }


   if(argc==3){/*
    if(strcasecmp(argv[1], "startSink")==0){
      startSink(atof(argv[2]));
      return TCL_OK;
	  }*/

    if(strcasecmp(argv[1], "addr")==0){
      my_id_ = Address::instance().str2addr(argv[2]);
      return TCL_OK;
    } 

    TclObject *obj;
    if ((obj = TclObject::lookup (argv[2])) == 0){
      fprintf (stderr, "%s: %s lookup of %s failed\n", __FILE__, argv[1],
	       argv[2]);
      return (TCL_ERROR);
    }
    if (strcasecmp (argv[1], "node") == 0) {
      node_ = (MobileNode*) obj;
      return (TCL_OK);
    }
    else if (strcasecmp (argv[1], "port-dmux") == 0) {
      port_dmux_ = (PortClassifier*) obj; //(NsObject *) obj;
      return (TCL_OK);
    } else if(strcasecmp(argv[1], "tracetarget")==0){
      tracetarget = (Trace *)obj;
      return TCL_OK;
    }

  }// if argc == 3

  return (Agent::command(argc, argv));
}

void
	GPSRAgent::printAns(){
	printf("%d  %f\n:",my_id_,initEnergy);
	for(int i =0;i < 4;i ++)
		printf("%d ",anchor[i]);
	printf("\n");
	for(int i = 0;i < 4;i ++)
		printf("%f ",sAnchor[i]);
	printf("\n");
	for(int i = 0;i < 4;i ++)
		printf("%f ",tAnchor[i]);
	printf("\n");
	printf("MatrixD: \n");
	for(int i = 0;i < 4;i ++){
		for(int j = 0;j < 4; j++)
			printf("%f  ",matrixD[i][j]);
		printf("\n");
	}
	printf("finalX: \n");
	for(int i = 0; i< 12;i ++)
		printf("%f  ",matrixClac->finalX[i]);
	printf("\n");

	
	printf("xNew: \n");
	for(int i = 0; i< 3;i ++)
		printf("%f  ",xNew[i]);
	printf("\n");
	printf("yNew: \n");
	for(int i = 0; i< 3;i ++)
		printf("%f  ",yNew[i]);
	printf("\n");
	printf("sink node: \n");
	sink_list_->dump();
	printf("\n");
	nblist_->dump();
		
	
}
