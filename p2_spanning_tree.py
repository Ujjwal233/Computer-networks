import time
from collections import defaultdict
from ryu.base import app_manager
from ryu.controller import ofp_event
from ryu.controller.handler import MAIN_DISPATCHER, CONFIG_DISPATCHER, set_ev_cls
from ryu.ofproto import ofproto_v1_3
from ryu.lib.packet import packet, ethernet
from ryu.topology.api import get_switch, get_link
from ryu.topology import event

class LearningSwitch(app_manager.RyuApp):
    OFP_VERSIONS = [ofproto_v1_3.OFP_VERSION]

    def __init__(self, *args, **kwargs):
        super(LearningSwitch, self).__init__(*args, **kwargs)
        self.mac_to_port = {}
        self.switch_ports = defaultdict(list)  # Ports per switch
        self.links = defaultdict(list)  # Links between switches
        self.host_ports = defaultdict(list)  # Ports where hosts are connected
        self.stp_ports = defaultdict(list)  # Ports as part of spanning tree

    @set_ev_cls(ofp_event.EventOFPSwitchFeatures, CONFIG_DISPATCHER)
    def switch_features_handler(self, ev):
        # Install a default rule to send all unmatched packets to the controller
        datapath = ev.msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        match = parser.OFPMatch()
        actions = [parser.OFPActionOutput(ofproto.OFPP_CONTROLLER, ofproto.OFPCML_NO_BUFFER)]
        self.add_flow(datapath, 0, match, actions)

    def add_flow(self, datapath, priority, match, actions):
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]
        mod = parser.OFPFlowMod(
            datapath=datapath,
            priority=priority,
            match=match,
            instructions=inst)
        datapath.send_msg(mod)

    @set_ev_cls(event.EventSwitchEnter)
    def get_topology_data(self, ev):
        # Step 1: Wait for topology discovery to complete
        # print('Waiting for topology discovery...')
        
        time.sleep(2)  # Give time for LLDP packets to propagate

        # Fetch switches and links
        switch_list = get_switch(self, None)
        link_list = get_link(self, None)

        # Step 2: Populate switch ports and identify host ports
        for switch in switch_list:
            # print(switch)
            dpid = switch.dp.id
            ports = [p.port_no for p in switch.ports]
            self.switch_ports[dpid] = ports

        for link in link_list:
            # print(link)
            self.links[link.src.dpid].append((link.dst.dpid, link.src.port_no,link.dst.port_no))
        
        # Subtract switch-to-switch ports to find host-facing ports
        for dpid, ports in self.switch_ports.items():
            switch_to_switch_ports = {link[1] for link in self.links[dpid]}
            host_ports = set(ports) - switch_to_switch_ports
            self.host_ports[dpid] = list(host_ports)

        # Step 3: Calculate spanning tree to prevent loops
        self.stp_ports = self.calculate_spanning_tree()

        # Step 4: Merge host-facing and spanning tree ports for each switch
        for dpid in self.host_ports:
            # print(self.host_ports[dpid],'before',dpid)
            self.host_ports[dpid].extend(self.stp_ports[dpid])
            # print(self.host_ports[dpid],'after',dpid)

    def calculate_spanning_tree(self):
        """
        Calculate a spanning tree to prevent loops in the network using DFS.
        """
        visited = set()
        spanning_tree = defaultdict(list)

        def dfs(dpid):
            visited.add(dpid)
            for neighbor, port_src,port_dst in self.links[dpid]:
                if neighbor not in visited:
                    spanning_tree[dpid].append(port_src)
                    spanning_tree[neighbor].append(port_dst)
                    dfs(neighbor)

        if self.links:
            root_switch = list(self.links.keys())[0]
            dfs(root_switch)

        return spanning_tree

    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def packet_in_handler(self, ev):
        msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        in_port = msg.match['in_port']

        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocol(ethernet.ethernet)
        dst = eth.dst
        src = eth.src
        dpid = datapath.id

        self.mac_to_port.setdefault(dpid, {})
        self.mac_to_port[dpid][src] = in_port

        if dst in self.mac_to_port[dpid]:
            # Known destination MAC: forward to the appropriate port
            out_port = self.mac_to_port[dpid][dst]
            actions = [parser.OFPActionOutput(out_port)]
        else:
            # Unknown destination or broadcast: forward to all available ports (except in_port)
            available_ports = set(self.host_ports[dpid]) - {in_port}
            # print("forwarding to these ports",available_ports)
            actions = [parser.OFPActionOutput(port) for port in available_ports]

        if dst in self.mac_to_port[dpid] or dst == 'ff:ff:ff:ff:ff:ff':
            match = parser.OFPMatch(in_port=in_port, eth_dst=dst, eth_src=src)
            self.add_flow(datapath, 1, match, actions)

        out = parser.OFPPacketOut(
            datapath=datapath,
            buffer_id=msg.buffer_id,
            in_port=in_port,
            actions=actions,
            data=msg.data)
        datapath.send_msg(out)
