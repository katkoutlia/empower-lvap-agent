#ifndef EMPOWERFAIRBUFFER_HH
#define EMPOWERFAIRBUFFER_HH
#include <click/element.hh>
#include <click/notifier.hh>
#include <click/hashtable.hh>
#include <click/etheraddress.hh>
#include <clicknet/ether.h>
#include <elements/standard/simplequeue.hh>
#include <elements/wifi/minstrel.hh>
CLICK_DECLS

/*
 * =c
 * EmpowerLVAPBuffer([ETHTYPE, CAPACITY, QUANTUM, MAX_BURST, MIN_BURST, DELAY, LT, ARP])
 * =s wifi
 * Implements the DRR scheduling policy
 * =io
 * one output, one input
 * =d
 * This element expects WiFi frames. Incoming MAC frames are classified according to the
 * destination address (the LVAP). Each flows is fed to a different queue holding up to
 * CAPACITY frames. The default for CAPACITY is 1000. Buffers are polled according with
 * a Deficit Round Robin (DRR) policy. Queues must be created by the EmpowerLVAPManager
 * Elements
 */

class EmpowerPacketBuffer {

  public:
	uint32_t _capacity;

	String _ssid;

	ReadWriteLock _queue_lock;
	Packet** _q;

	uint16_t _deficit;
	uint16_t _qquantum;
	uint32_t _size;
	uint32_t _bsize;
	uint32_t _drops;
	uint32_t _head;
	uint32_t _tail;
	uint16_t _p_cnt;
	uint8_t _weight;
	uint8_t _tenant;
	Vector<EtherAddress> LVAP_Active_List;
	float _ttime;

	EmpowerPacketBuffer(uint32_t capacity, String ssid) :
				_capacity(capacity), _ssid(ssid) {
		_q = new Packet*[_capacity];
		_size = 0;
		_bsize = 0;
		_drops = 0;
		_head = 0;
		_tail = 0;
		_deficit = 0;
		_p_cnt = 0;
		_weight = 50;
		_qquantum = 0;
		_ttime = 0.0;
		_tenant = 0;
    }

    ~EmpowerPacketBuffer() {
      _queue_lock.acquire_write();
		for (uint32_t i = 0; i < _capacity; i++) {
			if (_q[i]) {
				_q[i]->kill();
			}
		}
		delete[] _q;
		_queue_lock.release_write();

    }

    Packet* pull() {
    	Packet* p = 0;
		_queue_lock.acquire_write();
		if (_head != _tail) {
			_head = (_head + 1) % _capacity;
			p = _q[_head];
			_q[_head] = 0;
			_size--;
			_bsize -= p->length();
		}
		_queue_lock.release_write();
		return (p);
    }

    bool push(Packet* p) {
    	bool result = false;
		_queue_lock.acquire_write();
		if ((_tail + 1) % _capacity != _head) {
			_tail++;
			_tail %= _capacity;
			_q[_tail] = p;
			_size++;
			_bsize += p->length();
			result = true;
		} else {
			_drops++;
		}
		_queue_lock.release_write();
		return result;
    }

    const Packet* top() {
    	Packet* p = 0;
		_queue_lock.acquire_write();
		if (_head != _tail) {
			p = _q[(_head + 1) % _capacity];
		}
		_queue_lock.release_write();
		return p;
    }

};

class EmpowerQueueState {

	public:
		EtherAddress _bssid;
		EtherAddress _sta;
		String _ssid;

		EmpowerQueueState(EtherAddress bssid, EtherAddress sta, String ssid) :
			_bssid(bssid), _sta(sta), _ssid(ssid) {
	    }
};

class EmpowerFairBuffer : public SimpleQueue { public:

	EmpowerFairBuffer();
    ~EmpowerFairBuffer();

    const char* class_name() const		{ return "EmpowerFairBuffer"; }
    const char *port_count() const		{ return PORTS_1_1; }
    const char* processing() const		{ return PUSH_TO_PULL; }
    void *cast(const char *);

    int configure(Vector<String> &conf, ErrorHandler *);
    int initialize(ErrorHandler *);

    void push(int port, Packet *);
    Packet *pull(int port);

    void add_handlers();

    uint32_t drops() { return(_drops); } // dropped packets
    uint32_t bdrops() { return(_bdrops); } // bytes dropped

    uint32_t quantum() { return(_quantum); }
    uint32_t capacity() { return(_capacity); }

    String list_queues();
    void request_queue(String);
    void release_queue(EtherAddress);
    void create_lvap_info(EtherAddress, EtherAddress, String);

  protected:

    enum { SLEEPINESS_TRIGGER = 9};

    class Minstrel *_rate_control;
    class EmpowerLVAPManager *_el;

    int _sleepiness;
    ActiveNotifier _empty_note;

    typedef HashTable<String, EmpowerPacketBuffer*> HypervisorTable;
    typedef HypervisorTable::iterator HTIter;

    typedef HashTable<EtherAddress, EmpowerQueueState*> LVAP_table;
    typedef LVAP_table::iterator LVAPIter;

    typedef HashTable<EtherAddress, EtherAddress> BssidStaTable;
    typedef BssidStaTable::iterator BssidStaIter;

    Vector<String> _active_list;
    Vector<String> _tenant_list;
    HypervisorTable _hyper_table;
    LVAP_table _lvap_table;
	BssidStaTable _bssid_sta_table;

    uint32_t _quantum;
    uint32_t _capacity;
    uint8_t _qcnt;
    uint32_t _iteration;

    uint32_t _drops; // packets dropped because of full queue
    uint32_t _bdrops; // bytes dropped

    bool _go_to_sleep;
	bool _debug;

	//uint32_t compute_deficit(const Packet *);
	uint32_t compute_deficit(EtherAddress, const Packet *);

    static int write_handler(const String &, Element *, void *, ErrorHandler *);
    static String read_handler(Element *, void *);

};

CLICK_ENDDECLS
#endif


