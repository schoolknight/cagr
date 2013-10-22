
/* -*- Mode:C++; c-basic-offset: 2; tab-width:2; indent-tabs-width:t -*- 
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

/* gpsr_packet.h : the definition the gpsr routing protocol packet header
 *
 * hdr_hello : hello message header, broadcast to one hop neighbors
 * hdr_query : query message header, flood the data sink info into networks 
 * hdr_data  : not a really data packet header, it is just used to 
 *             be attached to any data packet created by higher layer apps
 *             The reason to use this type of header is decided by the 
 *             design of the GPSR: each data packet has to carry the 
 *             location info of its data sink, maintain the routing protocol
 *             stateless.
 */

#ifndef GPSR_PACKET_H_
#define GPSR_PACKET_H_

#include "packet.h"
#include <math.h>

#define SINK_TRACE_FILE "sink_trace.tr"
#define NB_TRACE_FILE "gpsrnb_trace.tr"

#define GPSR_CURRENT Scheduler::instance().clock()
#define INFINITE_DELAY 5000000000000.0

#define GPSRTYPE_DATA   0x01   //the CBR data msg
#define GPSRTYPE_RTSD   0x02
#define GPSRTYPE_RTSA   0x04
#define GPSRTYPE_CTS    0x08



#define GPSR_MODE_GF    0x01   //greedy forwarding mode
#define GPSR_MODE_PERI  0x02   //perimeter routing mode

#define HDR_GPSR(p)   ((struct hdr_gpsr*)hdr_gpsr::access(p))
#define HDR_GPSR_DATA(p) ((struct hdr_gpsr_data*)hdr_gpsr::access(p))
#define HDR_GPSR_RTSD(p) ((struct hdr_gpsr_rtsd*)hdr_gpsr::access(p))
#define HDR_GPSR_RTSA(p) ((struct hdr_gpsr_rtsa*)hdr_gpsr::access(p))


struct hdr_gpsr {
  u_int8_t type_;
  
  static int offset_;
  inline static int& offset() {return offset_;}
  inline static struct hdr_gpsr* access(const Packet *p){
    return (struct hdr_gpsr*) p->access(offset_);
  }
};


struct hdr_gpsr_data {
	u_int8_t type_;  
	u_int8_t mode_;  //Greedy forwarding or Perimeter Routing

	double tar_x;  //target position
	double tar_y;
	
	double per_dis; //peri distance

	
	float ts_;      //the originating time stamp
  inline int size(){
    int sz =
      2*sizeof(u_int8_t) +
      3*sizeof(double) + sizeof(float);
    return sz;
  }
};

struct hdr_gpsr_rtsd{
	u_int8_t type_;
	int timerID;	
	double sour_dis;
	double tar_x;
	double tar_y;
	inline int size(){
		int sz = sizeof(int) +sizeof(u_int8_t)+3*sizeof(double);
		return sz;
	}
};

struct hdr_gpsr_resa{
	u_int8_t type_;
	int timerID;	
	double sour_x;
	double sour_y;
	double tar_x;
	double tar_y;
	inline int size(){
		int sz = sizeof(int) + sizeof(u_int8_t)+4*sizeof(double);
		return sz;
	}
};

struct hdr_gpsr_cts{
	u_int8_t type_;
	
	int timerID;
	double res;
	inline int size(){
		int sz = sizeof(int) + sizeof(u_int8_t)+sizeof(double);
		return sz;
	}
	
};


union hdr_all_gpsr {
	hdr_gpsr       gh;
	hdr_gpsr_data  gdh;
	hdr_gpsr_rtsd  grdh;
	hdr_gpsr_resa  grah;
};



#endif
