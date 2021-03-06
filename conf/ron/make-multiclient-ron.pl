#!/usr/bin/perl

sub TOHEX {
    $a = $_[0];
    #print $a, "\t";
    
    $a =~ /(\d*).(\d*).(\d*).(\d*)/;
    
    #print $1, " ", $2, " ", $3, " ", $4, "\t";
    $tohex = sprintf ("%02x%02x%02x%02x", $1, $2, $3, $4);
    
}

# --------- MAIN ----------
if (scalar(@ARGV) < 5){
    print("Not enough arguments\n");
    print("usage: make-multiclient-ron.pl device meIP meHW gwIP serverIP0 [serverIP1 ...]\n\n");
    exit(-1);
}

$device    = shift(@ARGV);
$meIP      = shift(@ARGV);
$meHW      = shift(@ARGV);
$gwIP      = shift(@ARGV);
@servers   = @ARGV;
$n         = scalar(@servers);
#print "[", $neigh, "]\n";

($sec,$min,$hour,$mday,$mon,$year,$wday,$yday,$isdst) = localtime();

print "// This configuration was generated by make-multiclient-ron.pl\n";
print "// $mon/$mday/$year  $hour:$min:$sec\n";
print "//\n";
print "// make-multiclient-ron.pl ",$device, " ",$meIP, " ", $meHW, " ", $gwIP, " ";
for($i=0; $i<$n; $i++) {
    print $servers[$i], " ";
}
print "\n//\n";
print "// device :\t", $device, "\n";
print "// This IP:\t", $meIP, "\n";
print "// This HW:\t", $meHW, "\n";
print "// GW IP:  \t", $gwIP, "\n";
for($i=0; $i<$n; $i++) {
    print "// Server", $i + 2, ":\t", $servers[$i], "\n";
}

print "\n";

#-------------------------------------------------------------------------------

print "require(ron);\n";
print "ChangeUID(32767, 100);\n";
print "\n";

print "elementclass SYNACKPrinter\n";
print "\t{ipc :: IPClassifier (tcp opt syn and tcp opt ack, -);\n";
print "\t\tinput -> ipc;\n";
print "\t\tipc[0] -> PrintTime(SYNACK) -> output;\n";
print "\t\tipc[1] -> output;\n";
print "\t};\n";
print "\n";

print "elementclass SYNPrinter\n";
print "\t{ipc :: IPClassifier (tcp opt syn, -);\n";
print "\t\tinput -> ipc;\n";
print "\t\tipc[0] -> PrintTime(SYN) -> output;\n";
print "\t\tipc[1] -> output;\n";
print "\t};\n";
print "\n";

print "iprw :: IPRewriter(pattern - - - - 0 1,\n";
print "\t\tpattern - - - - 2 3,\n";
print "\t\tTCP_TIMEOUT 1200,\n";
print "\t\tREAP_TCP 300);\n";
print "\n";

#print "rt :: LookupIPRouteRON(", $n + 1, ");\n";
print "bigswitch :: Switch(0) [0] -> Discard;\n";


print "neighborclass :: IPClassifier(\n";
for($i=0; $i<$n; $i++) {
    print "\tsrc ", $servers[$i], ", \n";
}
print "-);\n";

print "toktap :: EtherEncap(0x0800, 1:1:1:1:1:1, 2:2:2:2:2:2)\n";
print "\t-> KernelTap(1.0.0.1/255.255.255.0)\n";
print "\t-> Discard;\n";
print "synacks :: SYNACKPrinter[0]\n";
print "\t-> toktap;\n";
print "\n";

for($i=1; $i<$n+2; $i++) {
    print "switch$i :: Switch(0) -> Discard\n";
    print "switch$i [1] -> synacks\n";
}
print "\n";

print "setgw :: SetIPAddress(", $gwIP, ");\n";
print "arpq :: ARPQuerier(", $meIP, ", ", $meHW, ")\n";
print "from0 :: FromDevice(", $device, ");\n";
print "\n";

print "from0\t-> c :: Classifier(12/0806 20/0002, -)\n";
print "\t-> [1]arpq;\n";
print "c[1] -> Discard;\n";
print "\n";

print "sOut :: Switch(1);\n";
print "sOut[0]  -> setgw;\n";
print "sIn :: Switch(1);\n";
print "sIn[0] ->  toktap;\n";
print "\n";

print "ControlSocket(UNIX, /tmp/clicksocket);\n";
print "\n";

print "// ------------- Divert Sockets ---------------\n";
print "// Outgoing TCP Packets\n";
print "DivertSocket(4000)\n";
print "//\t-> Print(OUT_TCP)\n";
print "\t-> MarkIPHeader\n";
print "\t-> SetIPChecksum\n";
print "\t-> sOut[1]\n";
print "\t-> CheckIPHeader\n";
print "\t-> GetIPAddress(16)\n";
print "\t-> SYNPrinter\n";
print "\t-> [0]iprw;\n";
print "\n";

print "// Incoming TCP Packets\n";
print "DivertSocket(4001)\n";
print "\t-> sIn[1]\n";
print "\t-> CheckIPHeader\n";
print "//\t-> Print(IN__TCP)\n";
print "\t-> GetIPAddress(16)\n";
print "\t-> [1]iprw;\n";
print "\n";

print "// Incoming UDP Encapsulated Packets\n";
print "DivertSocket(4002)\n";
print "\t-> CheckIPHeader\n";
print "//\t-> Print(IN-ENCAP-RAW)\n";
print "\t-> GetIPAddress(16)\n";
print "\t-> neighborclass;\n";
print "// ------------- Divert Sockets ---------------\n";
print "\n";

for($i=0; $i<$n; $i++) {
    my $funk = $i + 2;
    print "neighborclass[", $i, "]\n";
    print "\t-> StripIPHeader\n";
    print "\t-> Strip(8)\n";
    print "\t-> CheckIPHeader\n";
    print "//\t-> IPPrint(IN-ENCAP-STR)\n";
    print "\t-> IPReassembler\n";
    print "\t-> CheckIPHeader\n";
    print "\t-> GetIPAddress(16)\n";
    print "\t-> switch$funk;\n";
    print "\n";
}

print "neighborclass[", $n, "]\n";
print "\t-> toktap;\n";
print "\n";

print "iprw[0] -> [0]bigswitch;\n";
print "iprw[1] -> switch1;\n";
print "iprw[2] //-> Print(OUTSIDE-IN,40)\n";
print "\t-> toktap;\n";
print "iprw[3] //-> Print(OUTSIDE-OUT,40)\n";
print "\t-> setgw;\n";
print "\n";

print "bigswitch[1]\n";
print "\t//-> IPPrint(OUT_RT_PORT1)\n";
print "\t-> setgw;\n";
print "//\t-> Discard;\n";
print "\n";

for($i=0; $i<$n; $i++) {
    print "bigswitch[", $i+2, "]\t-> IPFragmenter(1400, false)\n";
    print "\t-> UDPIPEncap($meIP, 4000, $servers[$i], 4000)\n";
    print "//\t-> IPPrint(OUT_RT_PORT", $i+2, ")\n";
    print "\t-> setgw;\n";
    print "\n";
}

print "setgw\t-> arpq\n";
print "//\t-> Print(outeth0,40)\n";
print "\t-> QueueNotify(100)\n";
print "\t-> ToDeviceNotify(", $device, ");\n";




