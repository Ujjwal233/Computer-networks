import time
import heapq  # We'll use this for Dijkstra's algorithm
from ryu.lib.packet import packet, ethernet, lldp
from collections import defaultdict
from ryu.base import app_manager
from ryu.controller import ofp_event
from ryu.controller.handler import MAIN_DISPATCHER, CONFIG_DISPATCHER, set_ev_cls
from ryu.ofproto import ofproto_v1_3
from ryu.lib.packet import packet, ethernet, ether_types
from ryu.topology.api import get_switch, get_link
from ryu.topology import event
from ryu.lib import mac
import struct

class ShortestPathSwitch(app_manager.RyuApp):
    OFP_VERSIONS = [ofproto_v1_3.OFP_VERSION]

    def __init__(self, *args, **kwargs):
        super(ShortestPathSwitch, self).__init__(*args, **kwargs)  # Fixed __init__ typo
        self.mac_to_port = {}
        self.switch_ports = defaultdict(list)  # Ports per switch
        self.links = defaultdict(list)  # Links between switches with delays
        self.link_delays = defaultdict(dict)  # Delays between switches (as weights)
        self.host_ports = defaultdict(list)  # Ports where hosts are connected
        self.sp_paths = {}  # Stores the shortest paths
        self.lldp_responses = {}  # Store LLDP responses (not needed)
        self.mac_to_switch = {}
        self.iter =0 #iterations of topology discovery
        self.num_links = 0 #number of links
        self.start_times = {} #start_time for each of the links
        self.count=0
        self.link_list=[]
        self.switch_list=[]

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
            instructions=inst
        )
        datapath.send_msg(mod)

    @set_ev_cls(event.EventSwitchEnter)
    def get_topology_data(self, ev):
        self.iter+=1
        # print('Waiting for topology discovery...')
        time.sleep(2)  # Allow time for LLDP propagation

        # Fetch switches and links
        self.switch_list = get_switch(self, None)
        self.link_list = get_link(self, None)

        self.num_links = len(self.link_list)

        # Populate switch ports and links with delays
        if self.iter==len(self.switch_list):
            for switch in self.switch_list:
                # print(switch)
                dpid = switch.dp.id
                ports = [p.port_no for p in switch.ports]
                self.switch_ports[dpid] = ports
            
            for link in self.link_list:
                # start_time = time.time()
                self.send_custom_lldp(link)
            # time.sleep(5)
    def trigger(self):
        for link in self.link_list:
            # print(link)
            src = link.src.dpid
            dst = link.dst.dpid
            delay = self.link_delays[(src,dst)]  # Get the delay for this link
            self.links[src].append((dst, link.src.port_no, delay,link.dst.port_no))
            # self.links[dst].append((src, link.dst.port_no, delay))  # Bi-directional

        # Subtract switch-to-switch ports to find host-facing ports
        for dpid, ports in self.switch_ports.items():
            switch_to_switch_ports = {link[1] for link in self.links[dpid]}
            host_ports = set(ports) - switch_to_switch_ports
            self.host_ports[dpid] = list(host_ports)

        self.stp_ports = self.calculate_spanning_tree()

        for dpid in self.host_ports:
            self.host_ports[dpid].extend(self.stp_ports[dpid])
        # Calculate shortest paths
        self.sp_paths = self.calculate_shortest_paths()

        # print(f"Topology ready. Switches: {self.switch_ports}, Links: {self.links}")#, Shortest Paths: {self.sp_paths}")

    def calculate_spanning_tree(self):
        """
        Calculate a spanning tree to prevent loops in the network using DFS.
        """
        visited = set()
        spanning_tree = defaultdict(list)

        def dfs(dpid):
            visited.add(dpid)
            for neighbor, port_src,_,port_dst in self.links[dpid]:
                if neighbor not in visited:
                    spanning_tree[dpid].append(port_src)
                    spanning_tree[neighbor].append(port_dst)
                    dfs(neighbor)

        if self.links:
            root_switch = list(self.links.keys())[0]
            dfs(root_switch)

        return spanning_tree
    
    def send_custom_lldp(self, link):
        """Send a custom LLDP message from the source of the link to the destination."""
        src_dpid = link.src.dpid
        dst_dpid = link.dst.dpid
        src_port_no = link.src.port_no

        # Find the datapath corresponding to the source DPID
        datapath = self.get_datapath(src_dpid)
        if not datapath:
            self.logger.error(f"No datapath found for DPID: {src_dpid}")
            return

        lldp_pkt = packet.Packet()
        eth = ethernet.ethernet(dst=lldp.LLDP_MAC_NEAREST_BRIDGE,
                                src=mac.BROADCAST_STR,
                                ethertype=ether_types.ETH_TYPE_LLDP)
        lldp_pkt.add_protocol(eth)

        # Set the start time for delay calculation
        self.start_times[(src_dpid, dst_dpid)] = time.time()

        # Construct the Ethernet frame

        # Create LLDP TLVs
        chassis_id = lldp.ChassisID(subtype=lldp.ChassisID.SUB_LOCALLY_ASSIGNED,
                                     chassis_id=str(src_dpid).encode('utf-8'))
        port_id = lldp.PortID(subtype=lldp.PortID.SUB_LOCALLY_ASSIGNED,
                              port_id=str(src_port_no).encode('utf-8'))
        ttl = lldp.TTL(ttl=10)

        # Custom TLV carrying the name 'Ujjwal'
        custom_name_tlv = lldp.OrganizationallySpecific(oui = struct.pack('!I', 0x654213)[1:], subtype=1, info="Ujjwal".encode('utf-8'))

        
        lldp_pkt.add_protocol(lldp.lldp([chassis_id, port_id, ttl, custom_name_tlv,lldp.End()]))
        lldp_pkt.serialize()
        self.send_packet(datapath, src_port_no, lldp_pkt)

    def get_datapath(self, dpid):
        """Get the datapath object for a given DPID."""
        switches = get_switch(self)
        for switch in switches:
            if switch.dp.id == dpid:
                return switch.dp
        return None

    def send_packet(self, datapath, port_no, packet):
        """Send the packet to the specified port of the given datapath."""
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        actions = [parser.OFPActionOutput(port_no)]
        out = parser.OFPPacketOut(datapath=datapath, buffer_id=ofproto.OFP_NO_BUFFER,
                                   in_port=ofproto.OFPP_CONTROLLER, actions=actions,
                                   data=packet.data)
        datapath.send_msg(out)

    def calculate_shortest_paths(self):
        sp_paths = {}
        for src in self.links:
            sp_paths[src] = self.dijkstra(src)
        # print(sp_paths)
        
        return sp_paths

    def dijkstra(self, src):
        # print("switch_ports:", self.switch_ports)
        distances = {dpid: float('inf') for dpid in self.switch_ports}
        distances[src] = 0
        path = {src: []}
        priority_queue = [(0, src)]

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            if current_distance > distances[current_node]:
                continue

            for neighbor, port, weight,x in self.links[current_node]:
                distance = current_distance + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    path[neighbor] = path[current_node] + [(neighbor, port)]
                    heapq.heappush(priority_queue, (distance, neighbor))

        # print(distances)
        return path

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

        if eth.ethertype == ether_types.ETH_TYPE_LLDP:
            lldp_pkt = pkt.get_protocol(lldp.lldp)
            if lldp_pkt:
                for tlv in lldp_pkt.tlvs:
                    if isinstance(tlv, lldp.OrganizationallySpecific) and tlv.oui == struct.pack('!I', 0x654213)[1:]:
                        custom_data = tlv.info.decode('utf-8')
                        self.count+=1
                        if custom_data == "Ujjwal":
                            src_dpid = None
                            for tlv in lldp_pkt.tlvs:
                                if isinstance(tlv, lldp.ChassisID):
                                    src_dpid = int(tlv.chassis_id.decode())  
                            key = ( src_dpid,datapath.id) 
                            if key in self.start_times:
                                one_way_trip_time = time.time() - self.start_times[key]
                                self.link_delays[(src_dpid,datapath.id)] = one_way_trip_time  # Update the delay
                                # print(f"Updated Link Delay from {src_dpid} to {datapath.id}: {one_way_trip_time:.6f} seconds")
                            if self.count==len(self.link_list):
                                self.trigger()
            return
                
                            

        # Map the source MAC to the switch it came from
        if src not in self.mac_to_switch:
            self.mac_to_switch[src] = dpid

        self.mac_to_port.setdefault(dpid, {})
        self.mac_to_port[dpid][src] = in_port

        # Determine outgoing port
        if dst in self.mac_to_switch:
            dst_switch=self.mac_to_switch[dst]
            if dpid==dst_switch:
                out_port = self.mac_to_port[dpid][dst]
                actions = [parser.OFPActionOutput(out_port)]
            else:
                path = self.sp_paths[dpid][dst_switch]
                out_port = path[0][1]  # Next hop port
                actions = [parser.OFPActionOutput(out_port)]
            match = parser.OFPMatch(in_port=in_port, eth_dst=dst, eth_src=src)
            self.add_flow(datapath, 1, match, actions)
        else:
            available_ports = set(self.host_ports[dpid]) - {in_port}
        # print("forwarding to these ports",available_ports)
            actions = [parser.OFPActionOutput(port) for port in available_ports]

        match = parser.OFPMatch(in_port=in_port, eth_dst=dst, eth_src=src)
        self.add_flow(datapath, 1, match, actions)

        out = parser.OFPPacketOut(
            datapath=datapath,
            buffer_id=msg.buffer_id,
            in_port=in_port,
            actions=actions,
            data=msg.data
        )
        datapath.send_msg(out)
