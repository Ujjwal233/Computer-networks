import subprocess

def set_ovs_proto(switch, proto):
    command = f'sudo ovs-vsctl set Bridge {switch} protocols={proto}'
    subprocess.run(command, shell=True)

set_ovs_proto('s1', 'OpenFlow13')
set_ovs_proto('s2', 'OpenFlow13')
from ryu.base import app_manager
from ryu.controller import ofp_event
from ryu.controller.handler import CONFIG_DISPATCHER, MAIN_DISPATCHER, set_ev_cls
from ryu.ofproto import ofproto_v1_0
from ryu.ofproto import ofproto_v1_3
from ryu.lib.packet import packet
from ryu.lib.packet import ethernet

class HubController(app_manager.RyuApp):
    OFP_VERSIONS = [ ofproto_v1_3.OFP_VERSION]

    def __init__(self, *args, **kwargs):
        super(HubController, self).__init__(*args, **kwargs)

    @set_ev_cls(ofp_event.EventOFPSwitchFeatures, CONFIG_DISPATCHER)
    def switch_features_handler(self, ev):
        datapath = ev.msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser 
        #--------------cant set god know why----------------   
        # Set the miss_send_len to ensure switch buffers packets and sends a limited amount to the controller
        # req = parser.OFPSetConfig(datapath, ofproto.OFPC_FRAG_NORMAL, 2)
        # datapath.send_msg(req)
        #-------------default flow istallation is not optional---------
        # Install default flow: forward unmatched packets to controller
        match = parser.OFPMatch()
        actions = [parser.OFPActionOutput(ofproto.OFPP_CONTROLLER, ofproto.OFPCML_NO_BUFFER)]
        self.add_flow(datapath, 0, match, actions)
        
        #------------imp for stp---------------------
        # match = parser.OFPMatch(eth_dst='ff:ff:ff:ff:ff:ff')
        # actions = [parser.OFPActionOutput(ofproto.OFPP_FLOOD)]
        # self.add_flow(datapath, 1, match, actions)


    def add_flow(self, datapath, priority, match, actions):
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]
        mod = parser.OFPFlowMod(datapath=datapath, priority=priority, match=match, instructions=inst)
        datapath.send_msg(mod)

    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def packet_in_handler(self, ev):
        
        msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        in_port = msg.match['in_port']
        
        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocol(ethernet.ethernet)
        if eth.ethertype == 0x88cc : #filter lldp
            return
        actions = [parser.OFPActionOutput(ofproto.OFPP_FLOOD)]
        out = parser.OFPPacketOut(datapath=datapath, buffer_id=msg.buffer_id, in_port=in_port, actions=actions,data=msg.data)
        datapath.send_msg(out)
