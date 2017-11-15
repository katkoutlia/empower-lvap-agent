/*
 * empowerfairbuffer.{cc,hh}
 *
 * Katerina Koutlia, Roberto Riggio
 * Copyright (c) 2017, UPC, CREATE-NET
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
#include "empowerlvapmanager.hh"
CLICK_DECLS

enum {
	H_DROPS, H_BYTEDROPS, H_CAPACITY, H_LIST_QUEUES, H_DEBUG, H_TTIME, H_WEIGHTS
};

EmpowerFairBuffer::EmpowerFairBuffer() {
	_rate_control = 0;
	_el = 0;
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
	/*HTIter itr = _hyper_table.begin();
	while (itr != _hyper_table.end()) {
		release_queue(itr.key());
		itr++;
	}
	_hyper_table.clear();*/
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
			.read_m("EL", ElementCastArg("EmpowerLVAPManager"), _el)
			.read("CAPACITY", _capacity)
			.read("DEBUG", _debug)
			.complete();


  _empty_note.initialize(Notifier::EMPTY_NOTIFIER, router());
  return res;

}

int EmpowerFairBuffer::initialize(ErrorHandler *) {
	return 0;
}

uint32_t EmpowerFairBuffer::compute_deficit(EtherAddress sta, const Packet* p) {				//I need to calculate the packet transmission time

	uint32_t transmission_time;
	uint32_t transmission_rate;

	MinstrelDstInfo *nfo = _rate_control->neighbors()->findp(sta);

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

	EmpowerQueueState *d = _lvap_table.get(bssid);

	// get queue for ssid
	EmpowerPacketBuffer *q = _hyper_table.get(d->_ssid);

	// push packet on queue or fail
	if (q->push(p)) {
		// if station is not in the active list then add it at the end
		if (find(_active_list.begin(), _active_list.end(), d->_ssid) == _active_list.end()) {
			_active_list.push_back(d->_ssid);
			q->_deficit = q->_qquantum;
		}

		q->LVAP_Active_List.push_back(bssid);
		q->_p_cnt++;
		//click_chatter("PUSH PACKET for bssid: %s --- ssid: %s --- AL[0]: %s --- packet counter: %d", bssid.unparse().c_str(), d->_ssid.c_str(), _active_list[0].c_str(), q->_p_cnt);

		_empty_note.wake();
		_sleepiness = 0;


	} else {
		// queue overflow, destroy the packet
		if (_drops == 0) {
			click_chatter("%{element} :: %s :: overflow", this, __func__);
		}
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

	LVAPIter active_sta = _lvap_table.find(queue->LVAP_Active_List[0]);
	EmpowerQueueState* d = active_sta.value();
	EtherAddress sta = d->_sta;

	const Packet *p = queue->top();

	if (!p) {

		queue->_deficit = 0;
		_active_list.pop_front();
		return 0;
	}

	uint32_t deficit = compute_deficit(sta, p);

	if (deficit <= queue->_deficit) {

		queue->_p_cnt--;
		queue->LVAP_Active_List.pop_front();
		queue->_deficit -= compute_deficit(sta, p);

		//click_chatter("tp < DC ---- SEND PACKET for AL[0]: %s --- NEW DC: %d --- NEW packet counter: %d", _active_list[0].c_str(), queue->_deficit, queue->_p_cnt);

		if (queue->_size <= 1)					//If there are NO more packets in the queue
		{
			//click_chatter("********  NO MORE PACKETS --- reset DC and remove from AL  *********");
			queue->_deficit = 0;				//reset the DC
			_active_list.pop_front();			//remove tenant from active list
		}

		//queue->_ttime += (float) deficit/1000;
		queue->_ttime += deficit;

		return queue->pull();
	}

	String next = _active_list.front();
	_active_list.push_back(next);
	_active_list.pop_front();
	queue->_deficit += queue->_qquantum;

	//click_chatter("********  DEFICIT NOT ENOUGH (sleep) --- must go to the end --- NEW AL[0]: %s --- DC: %d  --- tp: %d *********", _active_list[0].unparse().c_str(), queue->_deficit, deficit);

	return 0;

}

String EmpowerFairBuffer::list_queues() {
	StringAccum result;
	HTIter itr = _hyper_table.begin();
	result << "Key,Capacity,Packets,Bytes\n";
	while (itr != _hyper_table.end()) {
		result << (itr.key()).c_str() << "," << itr.value()->_capacity << "," << itr.value()->_size << "," << itr.value()->_bsize << "\n";
		itr++;
	}
	return (result.take_string());
}

String EmpowerFairBuffer::list_ttimes() {
	StringAccum result;
	HTIter itr = _hyper_table.begin();
	result << "Tenant Time\n";
	while (itr != _hyper_table.end()) {
		result << (itr.key()).c_str() << ": " << itr.value()->_ttime << "\n";
		itr++;
	}
	return (result.take_string());
}

String EmpowerFairBuffer::list_weights() {
	StringAccum result;
	HTIter itr = _hyper_table.begin();
	result << "Tenant Weights\n";
	while (itr != _hyper_table.end()) {
		result << (itr.key()).c_str() << ": " << itr.value()->_weight << "\n";
		itr++;
	}
	return (result.take_string());
}

void EmpowerFairBuffer::create_lvap_info(EtherAddress bssid, EtherAddress sta, String ssid){

	_bssid_sta_table.set(sta, bssid);

	EmpowerQueueState* d = new EmpowerQueueState(bssid, sta, ssid);
	_lvap_table.set(bssid,d);
	d->_ssid = ssid;
	d->_sta = sta;
}

void EmpowerFairBuffer::request_queue(String ssid) {
	if (_debug) {
		click_chatter("%{element} :: %s :: request new queue for bssid %s",
					  this,
					  __func__,
					  ssid.c_str());
	}

	EmpowerPacketBuffer* q = new EmpowerPacketBuffer(_capacity, ssid);
	if (_hyper_table.empty()) {
		_empty_note.wake();
		_sleepiness = 0;
	}

	/*The tenant queue is created for first time so I have to assign weights*/
	/*To do so, I need to find how many tenants (queues) I have*/
	/*I have a list of the tenants that are added*/
	/*If this tenant appears for first time, it will not be in the list - Just to be sure I check it*/
	if (find(_tenant_list.begin(), _tenant_list.end(), ssid) == _tenant_list.end()) {

		_tenant_list.push_back(ssid);	//So I add him in the list
	}

	_hyper_table.set(ssid, q);

	//check how many tenants we have --> take size of _tenant_list
	int no_of_tenants = _tenant_list.size();

	switch(no_of_tenants){
		case 1:
			q->_weight = 100;
			q->_qquantum = (_quantum * q->_weight)/100;
			q->_tenant = 1;
			//click_chatter("tenant: %s  ---- weight: %d --- qquantum: %d", ssid.c_str(), q->_weight, q->_qquantum);
			break;
		case 2:
			q->_weight = 70;
			q->_qquantum = (_quantum * q->_weight)/100;
			q->_tenant = 2;
			//click_chatter("tenant: %s  ---- weight: %d --- qquantum: %d", ssid.c_str(), q->_weight, q->_qquantum);
			HTIter previous = _hyper_table.find(_tenant_list[0]);
			EmpowerPacketBuffer* q2 = previous.value();
			q2->_weight = 30;
			q2->_qquantum = (_quantum * q2->_weight)/100;
			//click_chatter("tenant: %s  ---- weight: %d --- qquantum: %d", q2->_ssid.c_str(), q2->_weight, q2->_qquantum);
			break;
	}
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
	add_read_handler("ttimes", read_handler, (void *) H_TTIME);
	add_read_handler("weights", read_handler, (void *) H_WEIGHTS);
	add_write_handler("capacity", write_handler, (void*) H_CAPACITY);
	add_write_handler("ttimes", write_handler, (void*) H_TTIME);
	add_write_handler("weights", write_handler, (void*) H_WEIGHTS);
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
	case H_TTIME:
		return (c->list_ttimes());
	case H_WEIGHTS:
			return (c->list_weights());
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
	case H_TTIME: {
		Vector<String> tokens;
		cp_spacevec(s, tokens);

		int time;

		if (tokens.size() < 1)
			return errh->error("insert at least one parameter");

		int index = 0;

		for (HTIter it_re = d->_hyper_table.begin(); it_re.live(); it_re++) {
			//it_re.value()->_ttime = (float) IntArg().parse(tokens[index], time);
			IntArg().parse(tokens[index], it_re.value()->_ttime);
			index++;
		}
		break;
	}
	case H_WEIGHTS: {
		Vector<String> tokens;
		cp_spacevec(s, tokens);

		int weight;

		if (tokens.size() < 1)
			return errh->error("insert at least one parameter");

		int index = 0;

		for (HTIter it_re = d->_hyper_table.begin(); it_re.live(); it_re++) {
			//it_re.value()->_weight = IntArg().parse(tokens[index], weight);
			IntArg().parse(tokens[index], it_re.value()->_weight);
			index++;
		}
		break;
	}
	}
	return 0;
}

CLICK_ENDDECLS
ELEMENT_REQUIRES(NotifierQueue)
EXPORT_ELEMENT(EmpowerFairBuffer)
