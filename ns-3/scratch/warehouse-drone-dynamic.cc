#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/flow-monitor-module.h"

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WarehouseDroneFleet");

static const uint16_t PORT = 9000;
static const double SIMULATION_STOP = 15.0;
static const double DRONE_SPEED = 10.0; // metres per simulated second
static const double WAREHOUSE_MIN = 0.0;
static const double WAREHOUSE_MAX = 100.0;

// Stop the drone once it reaches its commanded destination.
void StopDrone(Ptr<ConstantVelocityMobilityModel> mobility)
{
    mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
    Vector position = mobility->GetPosition();

    std::cout << "[" << Simulator::Now().GetSeconds()
              << "s] DESTINATION REACHED at ("
              << position.x << ", " << position.y << ")"
              << std::endl;
}

// Handle a destination command received over the simulated UDP network.
// Command format: DEST <x> <y>
void MoveDroneTo(Ptr<Node> node, double destX, double destY)
{
    Ptr<ConstantVelocityMobilityModel> mobility =
        node->GetObject<ConstantVelocityMobilityModel>();

    if (mobility == nullptr)
    {
        std::cerr << "No mobility model on node " << node->GetId()
                  << "; cannot move drone." << std::endl;
        return;
    }

    Vector current = mobility->GetPosition();
    const double dx = destX - current.x;
    const double dy = destY - current.y;
    const double distance = std::sqrt(dx * dx + dy * dy);

    if (distance < 0.01)
    {
        StopDrone(mobility);
        return;
    }

    const double travelTime = distance / DRONE_SPEED;
    mobility->SetVelocity(Vector(DRONE_SPEED * dx / distance,
                                 DRONE_SPEED * dy / distance,
                                 0.0));

    std::cout << "[" << Simulator::Now().GetSeconds()
              << "s] Drone " << node->GetId()
              << " moving from (" << current.x << ", " << current.y
              << ") to (" << destX << ", " << destY << ")"
              << "; estimated travel time: " << travelTime << "s"
              << std::endl;

    Simulator::Schedule(Seconds(travelTime), &StopDrone, mobility);
}

// Receive UDP messages. Destination commands affect the receiving drone's
// mobility model; status and other messages are logged normally.
void ReceivePacket(Ptr<Socket> socket)
{
    Address from;
    Ptr<Packet> packet;

    while ((packet = socket->RecvFrom(from)))
    {
        InetSocketAddress address = InetSocketAddress::ConvertFrom(from);
        uint32_t size = packet->GetSize();
        std::string payload(size, '\0');
        if (size > 0)
        {
            packet->CopyData(reinterpret_cast<uint8_t*>(&payload[0]), size);
        }

        std::cout << "[" << Simulator::Now().GetSeconds()
                  << "s] UDP received " << size << " bytes from "
                  << address.GetIpv4() << ": " << payload << std::endl;

        std::istringstream commandStream(payload);
        std::string command;
        double x = 0.0;
        double y = 0.0;

        if (commandStream >> command && command == "DEST" &&
            commandStream >> x >> y)
        {
            if (!std::isfinite(x) || !std::isfinite(y) ||
                x < WAREHOUSE_MIN || x > WAREHOUSE_MAX ||
                y < WAREHOUSE_MIN || y > WAREHOUSE_MAX)
            {
                std::cerr << "Rejected destination command: coordinates must "
                          << "be between 0 and 100." << std::endl;
                continue;
            }

            Ptr<Node> receivingNode = socket->GetNode();
            if (receivingNode->GetId() == 0)
            {
                std::cerr << "Controller received a destination command; "
                          << "ignoring it." << std::endl;
                continue;
            }

            MoveDroneTo(receivingNode, x, y);
        }
    }
}

// Send one UDP application message.
void SendMessage(Ptr<Socket> socket,
                 Ipv4Address destination,
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

int main(int argc, char* argv[])
{
    bool verbose = false;
    uint32_t droneId = 1;
    double destX = 80.0;
    double destY = 60.0;

    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable additional logging", verbose);
    cmd.AddValue("droneId", "Drone to assign (1, 2, or 3)", droneId);
    cmd.AddValue("destX", "Destination X coordinate (0-100)", destX);
    cmd.AddValue("destY", "Destination Y coordinate (0-100)", destY);
    cmd.Parse(argc, argv);

    if (droneId < 1 || droneId > 3)
    {
        std::cerr << "Invalid --droneId. Choose 1, 2, or 3." << std::endl;
        return 1;
    }

    if (!std::isfinite(destX) || !std::isfinite(destY) ||
        destX < WAREHOUSE_MIN || destX > WAREHOUSE_MAX ||
        destY < WAREHOUSE_MIN || destY > WAREHOUSE_MAX)
    {
        std::cerr << "Invalid destination. destX and destY must be between "
                  << "0 and 100." << std::endl;
        return 1;
    }

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
    wifi.SetRemoteStationManager(
        "ns3::ConstantRateWifiManager",
        "DataMode", StringValue("ErpOfdmRate6Mbps"),
        "ControlMode", StringValue("ErpOfdmRate6Mbps"));

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    // Install Internet stack and assign IPv4 addresses.
    InternetStackHelper internet;
    internet.Install(nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    // Initial positions: controller and three drones.
    Ptr<ListPositionAllocator> positionAlloc =
        CreateObject<ListPositionAllocator>();
    positionAlloc->Add(Vector(20.0, 20.0, 0.0)); // Controller
    positionAlloc->Add(Vector(50.0, 20.0, 0.0)); // Drone 1
    positionAlloc->Add(Vector(50.0, 50.0, 0.0)); // Drone 2
    positionAlloc->Add(Vector(20.0, 50.0, 0.0)); // Drone 3

    MobilityHelper mobility;
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(nodes);

    // Start with all nodes stationary. A drone moves only after it receives
    // a DEST command over the simulated Wi-Fi/UDP network.
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        nodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()
            ->SetVelocity(Vector(0.0, 0.0, 0.0));
    }

    // Install a UDP receive socket on every node.
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Ptr<Socket> receiver =
            Socket::CreateSocket(nodes.Get(i), UdpSocketFactory::GetTypeId());
        InetSocketAddress local(Ipv4Address::GetAny(), PORT);

        if (receiver->Bind(local) == 0)
        {
            receiver->SetRecvCallback(MakeCallback(&ReceivePacket));
        }
        else
        {
            NS_FATAL_ERROR("Failed to bind UDP receiver socket on node " << i);
        }
    }

    // Controller sends the user-selected destination to the chosen drone.
    Ptr<Socket> controllerSocket =
        Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());

    std::ostringstream destinationCommand;
    destinationCommand << "DEST " << destX << " " << destY;

    Simulator::Schedule(Seconds(1.0), &SendMessage,
                        controllerSocket, interfaces.GetAddress(droneId),
                        destinationCommand.str());

    // Drones send status reports back to the controller.
    for (uint32_t i = 1; i <= 3; ++i)
    {
        Ptr<Socket> droneSocket =
            Socket::CreateSocket(nodes.Get(i), UdpSocketFactory::GetTypeId());
        std::string status =
            "STATUS from Drone " + std::to_string(i) + ": ready";

        Simulator::Schedule(Seconds(1.5 + (i - 1) * 0.25),
                            &SendMessage, droneSocket,
                            interfaces.GetAddress(0), status);
    }

    // Demonstrate a temporary radio outage for Drone 2, then restore it.
    Ptr<WifiNetDevice> drone2Wifi =
        DynamicCast<WifiNetDevice>(devices.Get(2));

    Simulator::Schedule(Seconds(4.0), [drone2Wifi]() {
        std::cout << "[" << Simulator::Now().GetSeconds()
                  << "s] NETWORK EVENT: Drone 2 Wi-Fi radio disabled"
                  << std::endl;
        drone2Wifi->GetPhy()->SetOffMode();
    });

    Simulator::Schedule(Seconds(6.0), [drone2Wifi]() {
        std::cout << "[" << Simulator::Now().GetSeconds()
                  << "s] NETWORK EVENT: Drone 2 Wi-Fi radio re-enabled"
                  << std::endl;
        drone2Wifi->GetPhy()->ResumeFromOff();
    });

    // NetAnim output: node positions are sampled during the simulation.
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
                  << tuple.sourceAddress << " -> " << tuple.destinationAddress
                  << ")\n"
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

    std::cout << "Selected drone: " << droneId << "\n"
              << "Requested destination: (" << destX << ", " << destY << ")\n"
              << "NetAnim file: warehouse-animation.xml\n"
              << "=========================================\n";

    monitor->SerializeToXmlFile("warehouse-flowmon.xml", true, true);
    Simulator::Destroy();
    return 0;
}
