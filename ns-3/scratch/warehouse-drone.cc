#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"

#include <iostream>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WarehouseDroneFleet");

static const uint16_t PORT = 9000;
static const double SIMULATION_STOP = 8.0;

// Receive a UDP message and print its source and payload.
void ReceivePacket(Ptr<Socket> socket)
{
    Address from;
    Ptr<Packet> packet;

    while ((packet = socket->RecvFrom(from)))
    {
        InetSocketAddress address = InetSocketAddress::ConvertFrom(from);
        uint32_t size = packet->GetSize();
        std::string payload(size, '\0');
        packet->CopyData(reinterpret_cast<uint8_t*>(&payload[0]), size);

        std::cout << "[" << Simulator::Now().GetSeconds() << "s] UDP received "
                  << size << " bytes from " << address.GetIpv4()
                  << ": " << payload << std::endl;
    }
}

// Send one UDP application message.
void SendMessage(Ptr<Socket> socket, Ipv4Address destination,
                 const std::string& message)
{
    Ptr<Packet> packet = Create<Packet>(
        reinterpret_cast<const uint8_t*>(message.data()), message.size());

    int sent = socket->SendTo(packet, 0,
                              InetSocketAddress(destination, PORT));

    std::cout << "[" << Simulator::Now().GetSeconds() << "s] "
              << (sent >= 0 ? "Sent" : "FAILED to send")
              << " UDP message to " << destination
              << ": " << message << std::endl;
}

// Repeatedly send controller commands until stopTime.
void SendRepeatedMessage(Ptr<Socket> socket, Ipv4Address destination,
                         std::string message, double interval,
                         double stopTime)
{
    if (Simulator::Now().GetSeconds() > stopTime)
    {
        return;
    }

    SendMessage(socket, destination, message);

    if (Simulator::Now().GetSeconds() + interval <= stopTime)
    {
        Simulator::Schedule(Seconds(interval), &SendRepeatedMessage,
                            socket, destination, message, interval, stopTime);
    }
}

void LogTaskEvent(const std::string& message)
{
    std::cout << "[" << Simulator::Now().GetSeconds()
              << "s] TASK EVENT: " << message << std::endl;
}

int main(int argc, char* argv[])
{
    bool verbose = false;
    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable additional logging", verbose);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("WarehouseDroneFleet", LOG_LEVEL_INFO);
    }

    // Node 0 is the controller; nodes 1-3 are drones.
    NodeContainer nodes;
    nodes.Create(4);

    // Ad hoc 802.11g Wi-Fi network.
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("ErpOfdmRate6Mbps"),
                                 "ControlMode", StringValue("ErpOfdmRate6Mbps"));

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    // Internet stack and IP addressing.
    InternetStackHelper internet;
    internet.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // Initial positions: controller and three drones.
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(20.0, 20.0, 0.0)); // Controller
    positionAlloc->Add(Vector(50.0, 20.0, 0.0)); // Drone 1
    positionAlloc->Add(Vector(50.0, 50.0, 0.0)); // Drone 2
    positionAlloc->Add(Vector(20.0, 50.0, 0.0)); // Drone 3

    MobilityHelper mobility;
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(nodes);

    // Controller remains stationary; drones move independently.
    nodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()
        ->SetVelocity(Vector(0.0, 0.0, 0.0));
    nodes.Get(1)->GetObject<ConstantVelocityMobilityModel>()
        ->SetVelocity(Vector(3.0, 0.0, 0.0));
    nodes.Get(2)->GetObject<ConstantVelocityMobilityModel>()
        ->SetVelocity(Vector(-2.0, 1.0, 0.0));
    nodes.Get(3)->GetObject<ConstantVelocityMobilityModel>()
        ->SetVelocity(Vector(0.0, -3.0, 0.0));

    // One UDP receive socket on every node.
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Ptr<Socket> receiver = Socket::CreateSocket(nodes.Get(i), UdpSocketFactory::GetTypeId());
        InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), PORT);
        if (receiver->Bind(local) == 0)
        {
            receiver->SetRecvCallback(MakeCallback(&ReceivePacket));
        }
        else
        {
            NS_FATAL_ERROR("Failed to bind UDP receiver socket on node " << i);
        }
    }

    // Controller transmit sockets: only the controller sends task commands.
    Ptr<Socket> controllerSocket = Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());

    // Each drone has a transmit socket for status reports back to controller.
    for (uint32_t i = 1; i <= 3; ++i)
    {
        Ptr<Socket> droneSocket = Socket::CreateSocket(nodes.Get(i), UdpSocketFactory::GetTypeId());
        Ipv4Address controllerAddress = interfaces.GetAddress(0);
        std::string status = "STATUS from Drone " + std::to_string(i) + ": ready";
        Simulator::Schedule(Seconds(1.5 + (i - 1) * 0.25),
                            &SendMessage, droneSocket, controllerAddress, status);
    }

    // Repeated controller-to-drone task commands.
    Simulator::Schedule(Seconds(1.0), &SendRepeatedMessage,
                        controllerSocket, interfaces.GetAddress(1),
                        std::string("TASK: Move to aisle A"), 0.5, 7.5);
    Simulator::Schedule(Seconds(1.0), &SendRepeatedMessage,
                        controllerSocket, interfaces.GetAddress(2),
                        std::string("TASK: Scan shelf B"), 0.5, 7.5);
    Simulator::Schedule(Seconds(1.0), &SendRepeatedMessage,
                        controllerSocket, interfaces.GetAddress(3),
                        std::string("TASK: Deliver package C"), 0.5, 7.5);

    // Demonstrate task lifecycle events in the simulation log.
    Simulator::Schedule(Seconds(2.0), &LogTaskEvent,
                        std::string("Drone 1 started aisle A task"));
    Simulator::Schedule(Seconds(3.0), &LogTaskEvent,
                        std::string("Drone 3 package delivery in progress"));
    Simulator::Schedule(Seconds(6.5), &LogTaskEvent,
                        std::string("Drone 1 task completed"));
    Simulator::Schedule(Seconds(7.0), &LogTaskEvent,
                        std::string("Drone 3 task completed"));

    // Simulate a temporary radio outage for Drone 2, then restore it.
    // Wi-Fi PHY state changes are visible in the simulation behavior/log.
    Ptr<WifiNetDevice> drone2Wifi = DynamicCast<WifiNetDevice>(devices.Get(2));
    Simulator::Schedule(Seconds(4.0), [drone2Wifi]() {
        std::cout << "[" << Simulator::Now().GetSeconds()
                  << "s] NETWORK EVENT: Drone 2 Wi-Fi radio disabled" << std::endl;
        drone2Wifi->GetPhy()->SetOffMode();
    });
    Simulator::Schedule(Seconds(6.0), [drone2Wifi]() {
        std::cout << "[" << Simulator::Now().GetSeconds()
                  << "s] NETWORK EVENT: Drone 2 Wi-Fi radio re-enabled" << std::endl;
        drone2Wifi->GetPhy()->ResumeFromOff();
    });

    // NetAnim output. Packet events are generated from real NS-3 traffic.
    AnimationInterface animation("warehouse-animation.xml");
    animation.EnablePacketMetadata(true);
    animation.SetMobilityPollInterval(MilliSeconds(100));

    animation.UpdateNodeDescription(0, "Controller");
    animation.UpdateNodeDescription(1, "Drone 1");
    animation.UpdateNodeDescription(2, "Drone 2");
    animation.UpdateNodeDescription(3, "Drone 3");

    animation.UpdateNodeColor(0, 30, 100, 220);
    animation.UpdateNodeColor(1, 20, 170, 70);
    animation.UpdateNodeColor(2, 230, 150, 20);
    animation.UpdateNodeColor(3, 160, 60, 200);

    // FlowMonitor statistics.
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.InstallAll();

    Simulator::Stop(Seconds(SIMULATION_STOP));
    Simulator::Run();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    std::cout << "\n========== FLOW MONITOR SUMMARY ==========\n";
    for (auto it = stats.begin(); it != stats.end(); ++it)
    {
        Ipv4FlowClassifier::FiveTuple tuple = classifier->FindFlow(it->first);
        std::cout << "Flow " << it->first << " ("
                  << tuple.sourceAddress << " -> " << tuple.destinationAddress << ")\n"
                  << "  Tx Packets: " << it->second.txPackets << "\n"
                  << "  Rx Packets: " << it->second.rxPackets << "\n"
                  << "  Lost Packets: " << it->second.lostPackets << "\n";
        if (it->second.rxPackets > 0)
        {
            std::cout << "  Throughput: "
                      << (it->second.rxBytes * 8.0 /
                          (SIMULATION_STOP * 1000000.0))
                      << " Mbps\n";
        }
    }
    std::cout << "NetAnim file: warehouse-animation.xml\n";
    std::cout << "=========================================\n";

    monitor->SerializeToXmlFile("warehouse-flowmon.xml", true, true);
    Simulator::Destroy();
    return 0;
}
