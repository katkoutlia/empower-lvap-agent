#ifndef ICMP6ERROR_HH
#define ICMP6ERROR_HH

/*
 * =c
 * ICMP6Error(IPADDR, TYPE, CODE)
 * =s
 * V<IPv6>
 * =d
 * need to rewrite the following comments.
 * Generate an ICMP error packet, with specified TYPE and CODE,
 * in response to an incoming IP packet. The output is an IP/ICMP packet.
 * The ICMP packet's IP source address is set to IPADDR.
 * The error packet will include (as payload)
 * the original packet's IP header and the first 8 byte of the packet's
 * IP payload. ICMPError sets the packet destination IP and
 * fix_ip_src annotations.
 *
 * The intent is that elements that give rise to errors, like DecIPTTL,
 * should have two outputs, one of which is connected to an ICMPError.
 * Perhaps the ICMPError()s should be followed by a rate limiting
 * element.
 *
 * ICMPError never generates a packet in response to an ICMP error packet,
 * a fragment, or a link broadcast.
 *
 * The output of ICMPError should be connected to the routing lookup
 * machinery, much as if the ICMP errors came from a hardware interface.
 *
 * If TYPE is 12 and CODE is 0 (Parameter Problem), ICMPError
 * takes the error pointer from the packet's param_off annotation.
 * The IPGWOptions element sets the annotation.
 *
 * If TYPE is 5, produces an ICMP redirect message. The gateway
 * address is taken from the destination annotation. Usually a
 * Paint/CheckPaint element pair hands the packet to a redirect ICMPError.
 * RFC1812 says only code 1 (host redirect) should be used.
 *
 * =e
 * This configuration fragment produces ICMP Time Exceeded error
 * messages in response to TTL expirations, but limits the
 * rate at which such messages can be sent to 10 per second:
 *
 *   dt : DecIPTTL();
 *   dt[1] -> ICMPError(18.26.4.24, 11, 0) -> m :: PacketMeter(10) -> ...
 *   m[1] -> Discard;
 *
 * =n
 *
 * ICMPError can't decide if the packet's source or destination address is an
 * IP directed broadcast address; it is supposed to ignore packets with such
 * addresses.
 *
 * =a DecIPTTL, FixIPSrc, IPGWOptions */

#include "element.hh"
#include "ip6address.hh"

class ICMP6Error : public Element {
public:
  ICMP6Error();
  ~ICMP6Error();
  
  const char *class_name() const		{ return "ICMP6Error"; }
  const char *processing() const		{ return AGNOSTIC; }
  ICMP6Error *clone() const			{ return new ICMP6Error; }
  int configure(const Vector<String> &, ErrorHandler *);
  int initialize(ErrorHandler *errh);

  Packet *simple_action(Packet *);
  
private:

  IP6Address _src_ip;
  int _type;
  int _code;

  bool is_error_type(int);
  //bool unicast(struct in6my_addr);
  //bool valid_source(struct in6my_addr);
  //bool has_route_opt(const click_ip6 *ip);
  
};

#endif
