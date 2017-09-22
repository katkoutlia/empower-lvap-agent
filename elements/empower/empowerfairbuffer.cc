/*
 * empowerfairbuffer.{cc,hh}
 *
 * Roberto Riggio
 * Copyright (c) 2017, CREATE-NET
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   - Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   - Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   - Neither the name of the CREATE-NET nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <click/config.h>
#include "empowerfairbuffer.hh"
#include <click/confparse.hh>
#include <click/error.hh>
#include <click/straccum.hh>
#include <click/standard/scheduleinfo.hh>
#include <click/args.hh>
#include <clicknet/ether.h>
#include <clicknet/wifi.h>
#include <clicknet/llc.h>
#include <elements/wifi/bitrate.hh>
#include <elements/wifi/minstrel.hh>
CLICK_DECLS

enum {
	H_DROPS, H_BYTEDROPS, H_CAPACITY, H_LIST_QUEUES, H_DEBUG
};

EmpowerFairBuffer::EmpowerFairBuffer() {
	_rate_control = 0;
	_drops = 0;
	_bdrops = 0;
	_sleepiness = 0;
	_quantum = 3000;
	_capacity = 1000;
	_qcnt = 0;
	_iteration = 0;
	_go_to_sleep = false;
	_debug = false;
}

EmpowerFairBuffer::~EmpowerFairBuffer() {
	// de-allocate hypervisor table
	HTIter itr = _hyper_table.begin();
	while (itr != _hyper_table.end()) {
		release_queue(itr.key());
		itr++;
	}
	_hyper_table.clear();
}


void *
EmpowerFairBuffer::cast(const char *n) {
	if (strcmp(n, "EmpowerFairBuffer") == 0)
		return (EmpowerFairBuffer *) this;
	else if (strcmp(n, Notifier::EMPTY_NOTIFIER) == 0)
		return static_cast<Notifier *>(&_empty_note);
	else
		return SimpleQueue::cast(n);
}

int
EmpowerFairBuffer::configure(Vector<String>& conf, ErrorHandler* errh)
{

  int res = Args(conf, this, errh)
			.read_m("RCS", ElementCastArg("Minstrel"), _rate_control)
			.read("CAPACITY", _capacity)
			.read("DEBUG", _debug)
			.complete();


  _empty_note.initialize(Notifier::EMPTY_NOTIFIER, router());
  return res;

}

int EmpowerFairBuffer::initialize(ErrorHandler *) {
	return 0;
}

uint32_t EmpowerFairBuffer::compute_deficit(EtherAddress dst, const Packet* p) {				//I need to calculate the packet transmission time

	uint32_t transmission_time;
	uint32_t transmission_rate;
	//EtherAddress station;


	MinstrelDstInfo *nfo = _rate_control->neighbors()->findp(dst);

	if (!nfo){
		if (_debug){
			click_chatter("no rate info available");
		}
		return 0;
	}

	transmission_rate =  nfo->rates[nfo->max_tp_rate];
	transmission_time = calc_usecs_wifi_packet_tries(p->length(), transmission_rate, 0, 1);

	return transmission_time;
}

void EmpowerFairBuffer::push(int, Packet* p) {

	if (p->length() < sizeof(struct click_wifi)) {
		click_chatter("%{element} :: %s :: packet too small: %d vs %d",
					  this,
					  __func__,
					  p->length(),
					  sizeof(struct click_ether));
		p->kill();
		return;
	}

	struct click_wifi *w = (struct click_wifi *) p->data();
	EtherAddress bssid = EtherAddress(w->i_addr2);


	// get queue for bssid
	EmpowerPacketBuffer *q = _hyper_table.get(bssid);

	// push packet on queue or fail
	if (q->push(p)) {
		// if station is not in the active list then add it at the end
		if (find(_active_list.begin(), _active_list.end(), bssid) == _active_list.end()) {
			_active_list.push_back(bssid);
			q->_deficit = q->_qquantum;
		}
		q->_p_cnt++;
		//click_chatter("***********************************************************");
		//click_chatter("PUSH PACKET for %s --- AL[0]: %s --- packet counter: %d", bssid.unparse().c_str(), _active_list[0].unparse().c_str(), q->_p_cnt);
		//click_chatter("***********************************************************");

		_empty_note.wake();
		_sleepiness = 0;


	} else {
		// queue overflow, destroy the packet
		if (_drops == 0) {
			click_chatter("%{element} :: %s :: overflow", this, __func__);
		}
		//click_chatter("PACKET DROPPED for %s", bssid.unparse().c_str());
		_bdrops += p->length();
		_drops++;
		p->kill();
	}

}


Packet *
EmpowerFairBuffer::pull(int) {

	if (_hyper_table.empty()) {
		_empty_note.sleep();
		_sleepiness = 0;
		return 0;
	}

	if (_active_list.empty()) {
		if (++_sleepiness == SLEEPINESS_TRIGGER) {
			if (_debug) {
				click_chatter("%{element} :: %s :: active queue is empty, going to sleep",
							  this,
							  __func__);
			}
			_empty_note.sleep();
		}
		return 0;
	}

	HTIter active = _hyper_table.find(_active_list[0]);
	EmpowerPacketBuffer* queue = active.value();

	//click_chatter("*******************************  ENTER PULL AL[0]: %s ******************************************", _active_list[0].unparse().c_str());

	const Packet *p = queue->top();

	if (!p) {

		//click_chatter("********  ENTER PULL BUT NO PACKET --- Remove from AL  *********");
		queue->_deficit = 0;
		_active_list.pop_front();
		return 0;
	}

	uint32_t deficit = compute_deficit(queue->_dst, p);

	//click_chatter("There is a Packet for %s --- DC: %d  --- tp: %d", _active_list[0].unparse().c_str(), queue->_deficit, deficit);


	if (deficit <= queue->_deficit) {
		queue->_deficit -= compute_deficit(queue->_dst, p);

		queue->_p_cnt--;
		//click_chatter("tp < DC ---- SEND PACKET for AL[0]: %s --- NEW DC: %d --- NEW packet counter: %d", _active_list[0].unparse().c_str(), queue->_deficit, queue->_p_cnt);

		if (queue->_size <= 1)					//If there are NO more packets in the queue
		{
			//click_chatter("********  NO MORE PACKETS --- reset DC and remove from AL  *********");
			queue->_deficit = 0;				//reset the DC
			_active_list.pop_front();			//remove from active list
		}

		queue->_ttime += (float) deficit/1000;
		_iteration++;
		if (_iteration%500000){
			//click_chatter("***********************************************************");
			click_chatter("BSSID: %s --- Tenant %d --- Transmission Time %f", queue->_dst.unparse().c_str(), queue->_tenant, queue->_ttime);
			//click_chatter("***********************************************************");
		}

		return queue->pull();
	}

	EtherAddress next = _active_list.front();
	_active_list.push_back(next);
	_active_list.pop_front();
	queue->_deficit += queue->_qquantum;

	//click_chatter("********  DEFICIT NOT ENOUGH (sleep) --- must go to the end --- NEW AL[0]: %s --- DC: %d  --- tp: %d *********", _active_list[0].unparse().c_str(), queue->_deficit, deficit);

	//_empty_note.sleep();
	//_sleepiness = 0;

	return 0;


}

String EmpowerFairBuffer::list_queues() {
	StringAccum result;
	HTIter itr = _hyper_table.begin();
	result << "Key,Capacity,Packets,Bytes\n";
	while (itr != _hyper_table.end()) {
		result << (itr.key()).unparse() << "," << itr.value()->_capacity << "," << itr.value()->_size << "," << itr.value()->_bsize << "\n";
		itr++;
	}
	return (result.take_string());
}

void EmpowerFairBuffer::request_queue(EtherAddress bssid, EtherAddress dst) {
	if (_debug) {
		click_chatter("%{element} :: %s :: request new queue for bssid %s dst %s",
					  this,
					  __func__,
					  bssid.unparse().c_str(),
					  dst.unparse().c_str());
	}
	EmpowerPacketBuffer* q = new EmpowerPacketBuffer(_capacity, bssid, dst);
	if (_hyper_table.empty()) {
		_empty_note.wake();
		_sleepiness = 0;
	}
	_hyper_table.set(bssid, q);
	q->_dst = dst;
	_qcnt++;
	//q->_weight = group;
	if (_qcnt == 1){
		q->_tenant = 1;
		q->_weight = 40;
	}
	else if (_qcnt == 2){
		q->_tenant = 2;
		q->_weight = 60;
	}
	q->_qquantum = (_quantum * q->_weight)/100;
	click_chatter("sta: %s  ---- weight: %d --- qquantum: %d", dst.unparse().c_str(), q->_weight, q->_qquantum);
	//q->_qquantum = _quantum;
}

void EmpowerFairBuffer::release_queue(EtherAddress bssid) {
	if (_debug) {
		click_chatter("%{element} :: %s :: releasing queue %s",
				      this,
					  __func__,
					  bssid.unparse().c_str());
	}
	HTIter itr = _hyper_table.find(bssid);
	// destroy queued packets
	Packet* p = 0;
	do {
		p = itr.value()->pull();
		// flush
		if (p) {
			p->kill();
		}
	} while (p);
	// erase queue
	_hyper_table.erase(itr);
	// set new next
	if (!_hyper_table.empty()) {
		if (itr == _hyper_table.end()) {
			itr = _hyper_table.begin();
		}
	} else {
		_empty_note.sleep();
		_sleepiness = 0;
	}
}

void EmpowerFairBuffer::add_handlers() {
	add_read_handler("debug", read_handler, (void *) H_DEBUG);
	add_read_handler("drops", read_handler, (void*) H_DROPS);
	add_read_handler("byte_drops", read_handler, (void*) H_BYTEDROPS);
	add_read_handler("capacity", read_handler, (void*) H_CAPACITY);
	add_read_handler("list_queues", read_handler, (void*) H_LIST_QUEUES);
	add_write_handler("capacity", write_handler, (void*) H_CAPACITY);
	add_write_handler("debug", write_handler, (void *) H_DEBUG);
}

String EmpowerFairBuffer::read_handler(Element *e, void *thunk) {
	EmpowerFairBuffer *c = (EmpowerFairBuffer *) e;
	switch ((intptr_t) thunk) {
	case H_DROPS:
		return (String(c->drops()) + "\n");
	case H_DEBUG:
		return String(c->_debug) + "\n";
	case H_BYTEDROPS:
		return (String(c->bdrops()) + "\n");
	case H_CAPACITY:
		return (String(c->capacity()) + "\n");
	case H_LIST_QUEUES:
		return (c->list_queues());
	default:
		return "<error>\n";
	}
}

int EmpowerFairBuffer::write_handler(const String &in_s, Element *e, void *vparam, ErrorHandler *errh) {
	EmpowerFairBuffer *d = (EmpowerFairBuffer *) e;
	String s = cp_uncomment(in_s);
	switch ((intptr_t) vparam) {
	case H_DEBUG: {
		bool _debug;
		if (!cp_bool(s, &_debug))
			return errh->error("debug parameter must be boolean");
		d->_debug = _debug;
		break;
	}
	case H_CAPACITY: {
		uint32_t capacity;
		if (!cp_unsigned(s, &capacity))
			return errh->error("parameter must be a positive integer");
		d->_capacity = capacity;
		break;
	}
	}
	return 0;
}

CLICK_ENDDECLS
ELEMENT_REQUIRES(NotifierQueue)
EXPORT_ELEMENT(EmpowerFairBuffer)
