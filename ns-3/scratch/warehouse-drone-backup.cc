#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <map>
#include <string>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WarehouseDroneFleet");

static const uint16_t PORT = 9000;
static const double SIMULATION_STOP = 20.0;
static const double MOVE_CHECK_INTERVAL = 0.1;
static const double DRONE_SPEED = 8.0; // simulated metres per second

struct Destination
{
    std::string name;
    Vector position;
};

static std::map<uint32_t, Destination> g_droneDestinations;
static std::map<uint32_t, bool> g_taskStarted;
static std::map<uint32_t, bool> g_taskCompleted;

static void CheckDroneArrival(uint32_t droneNodeId);

static bool
ParseTask(const std::string& payload, std::string& houseName)
{
    const std::string prefix = "TASK:";
    if (payload.rfind(prefix, 0) != 0)
    {
        return false;
    }

    std::size_t start = payload.find("HOUSE=");
    if (start == std::string::npos)
    {
        return false;
    }

    start += 6;
    houseName = payload.substr(start);
    return !houseName.empty();
}

static void
ReceivePacket(Ptr<Socket> socket)
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

        std::cout << "[" << Simulator::Now().GetSeconds() << "s] UDP received " << size
                  << " bytes from " << address.GetIpv4() << ": " << payload << std::endl;

        uint32_t nodeId = socket->GetNode()->GetId();
        if (nodeId >= 1 && nodeId <= 3)
        {
            std::string houseName;
            if (ParseTask(payload, houseName) && g_droneDestinations.count(nodeId) &&
                g_droneDestinations[nodeId].name == houseName && !g_taskStarted[nodeId])
            {
                Ptr<ConstantVelocityMobilityModel> mobility =
                    socket->GetNode()->GetObject<ConstantVelocityMobilityModel>();

                Vector current = mobility->GetPosition();
                Vector target = g_droneDestinations[nodeId].position;
                double dx = target.x - current.x;
                double dy = target.y - current.y;
                double distance = std::sqrt(dx * dx + dy * dy);

                g_taskStarted[nodeId] = true;

                std::cout << "[" << Simulator::Now().GetSeconds() << "s] Drone " << nodeId
                          << " accepted destination House " << houseName << " (" << target.x << ", "
                          << target.y << ")" << std::endl;

                if (distance <= 0.5)
                {
                    mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
                    g_taskCompleted[nodeId] = true;
                    std::cout << "[" << Simulator::Now().GetSeconds()
                              << "s] DELIVERY COMPLETED: Drone " << nodeId << " reached House "
                              << houseName << std::endl;
                }
                else
                {
                    mobility->SetVelocity(
                        Vector(DRONE_SPEED * dx / distance, DRONE_SPEED * dy / distance, 0.0));
                    Simulator::Schedule(Seconds(MOVE_CHECK_INTERVAL), &CheckDroneArrival, nodeId);
                }
            }
        }
    }
}

static void
SendMessage(Ptr<Socket> socket, Ipv4Address destination, const std::string& message)
{
    Ptr<Packet> packet =
        Create<Packet>(reinterpret_cast<const uint8_t*>(message.data()), message.size());

    int sent = socket->SendTo(packet, 0, InetSocketAddress(destination, PORT));

    std::cout << "[" << Simulator::Now().GetSeconds() << "s] "
              << (sent >= 0 ? "Sent" : "FAILED to send") << " UDP message to " << destination
              << ": " << message << std::endl;
}

static void
SendRepeatedMessage(Ptr<Socket> socket,
                    Ipv4Address destination,
                    std::string message,
                    double interval,
                    double stopTime)
{
    if (Simulator::Now().GetSeconds() > stopTime)
    {
        return;
    }

    SendMessage(socket, destination, message);

    if (Simulator::Now().GetSeconds() + interval <= stopTime)
    {
        Simulator::Schedule(Seconds(interval),
                            &SendRepeatedMessage,
                            socket,
                            destination,
                            message,
                            interval,
                            stopTime);
    }
}

static void
SendStatus(Ptr<Socket> socket, Ipv4Address controllerAddress, uint32_t droneId)
{
    std::string status = "STATUS from Drone " + std::to_string(droneId) +
                         ": ready; destination House " + g_droneDestinations[droneId].name;
    SendMessage(socket, controllerAddress, status);
}

static void
CheckDroneArrival(uint32_t droneNodeId)
{
    if (g_taskCompleted[droneNodeId])
    {
        return;
    }

    Ptr<Node> node = NodeList::GetNode(droneNodeId);
    Ptr<ConstantVelocityMobilityModel> mobility = node->GetObject<ConstantVelocityMobilityModel>();

    Vector current = mobility->GetPosition();
    Vector target = g_droneDestinations[droneNodeId].position;
    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= DRONE_SPEED * MOVE_CHECK_INTERVAL)
    {
        mobility->SetPosition(target);
        mobility->SetVelocity(Vector(0.0, 0.0, 0.0));
        g_taskCompleted[droneNodeId] = true;

        std::cout << "[" << Simulator::Now().GetSeconds() << "s] DELIVERY COMPLETED: Drone "
                  << droneNodeId << " reached House " << g_droneDestinations[droneNodeId].name
                  << " at (" << target.x << ", " << target.y << ")" << std::endl;
        return;
    }

    Simulator::Schedule(Seconds(MOVE_CHECK_INTERVAL), &CheckDroneArrival, droneNodeId);
}

static std::string
ReadHouseChoice(uint32_t droneId)
{
    while (true)
    {
        std::string choice;
        std::cout << "Enter destination for Drone " << droneId << " (House A/B/C): " << std::flush;

        if (!(std::cin >> choice))
        {
            NS_FATAL_ERROR("Unable to read destination input.");
        }

        std::transform(choice.begin(), choice.end(), choice.begin(), [](unsigned char c) {
            return static_cast<char>(std::toupper(c));
        });

        if (choice == "A" || choice == "B" || choice == "C")
        {
            return choice;
        }

        std::cout << "Invalid choice. Please enter A, B, or C." << std::endl;
    }
}

int
main(int argc, char* argv[])
{
    bool verbose = false;
    CommandLine cmd(__FILE__);
    cmd.AddValue("verbose", "Enable additional logging", verbose);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        LogComponentEnable("WarehouseDroneFleet", LOG_LEVEL_INFO);
    }

    // Ask for destinations before starting the simulation.
    std::cout << "\n=== SMART WAREHOUSE DRONE FLEET ===\n"
              << "Choose a destination for each drone.\n"
              << "House A = (70, 20)\n"
              << "House B = (70, 50)\n"
              << "House C = (20, 70)\n\n";

    std::string choice1 = ReadHouseChoice(1);
    std::string choice2 = ReadHouseChoice(2);
    std::string choice3 = ReadHouseChoice(3);

    const std::map<std::string, Destination> houses = {{"A", {"A", Vector(70.0, 20.0, 0.0)}},
                                                       {"B", {"B", Vector(70.0, 50.0, 0.0)}},
                                                       {"C", {"C", Vector(20.0, 70.0, 0.0)}}};

    g_droneDestinations[1] = houses.at(choice1);
    g_droneDestinations[2] = houses.at(choice2);
    g_droneDestinations[3] = houses.at(choice3);
    for (uint32_t id = 1; id <= 3; ++id)
    {
        g_taskStarted[id] = false;
        g_taskCompleted[id] = false;
        std::cout << "Drone " << id << " -> House " << g_droneDestinations[id].name << std::endl;
    }
    std::cout << "\nStarting simulation...\n\n";

    // Node 0 is the controller; nodes 1-3 are drones.
    NodeContainer nodes;
    nodes.Create(4);

    // Ad hoc 802.11g Wi-Fi network.
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue("ErpOfdmRate6Mbps"),
                                 "ControlMode",
                                 StringValue("ErpOfdmRate6Mbps"));

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

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

    nodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0.0, 0.0, 0.0));
    for (uint32_t i = 1; i <= 3; ++i)
    {
        nodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(
            Vector(0.0, 0.0, 0.0));
    }

    // UDP receive socket on every node.
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

    Ptr<Socket> controllerSocket =
        Socket::CreateSocket(nodes.Get(0), UdpSocketFactory::GetTypeId());

    // Drone status reports back to the controller.
    for (uint32_t i = 1; i <= 3; ++i)
    {
        Ptr<Socket> droneSocket = Socket::CreateSocket(nodes.Get(i), UdpSocketFactory::GetTypeId());
        Simulator::Schedule(Seconds(1.5 + (i - 1) * 0.25),
                            &SendStatus,
                            droneSocket,
                            interfaces.GetAddress(0),
                            i);
    }

    // Send each chosen destination over UDP. Repeated packets retain the
    // project's network-traffic demonstration; a drone starts its task once.
    for (uint32_t i = 1; i <= 3; ++i)
    {
        std::string task = "TASK: HOUSE=" + g_droneDestinations[i].name;
        Simulator::Schedule(Seconds(1.0),
                            &SendRepeatedMessage,
                            controllerSocket,
                            interfaces.GetAddress(i),
                            task,
                            0.5,
                            7.5);
    }

    // Simulate a temporary radio outage for Drone 2, then restore it.
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

    NodeContainer houseNodes;
    houseNodes.Create(3);
    MobilityHelper houseMobility;
    Ptr<ListPositionAllocator> housePositions = CreateObject<ListPositionAllocator>();
    housePositions->Add(Vector(70.0, 20.0, 0.0));
    housePositions->Add(Vector(70.0, 50.0, 0.0));
    housePositions->Add(Vector(20.0, 70.0, 0.0));
    houseMobility.SetPositionAllocator(housePositions);
    houseMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    houseMobility.Install(houseNodes);

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

    // House markers are represented by fixed NetAnim nodes.

    animation.UpdateNodeDescription(4, "House A");
    animation.UpdateNodeDescription(5, "House B");
    animation.UpdateNodeDescription(6, "House C");
    animation.UpdateNodeColor(4, 220, 40, 40);
    animation.UpdateNodeColor(5, 220, 40, 40);
    animation.UpdateNodeColor(6, 220, 40, 40);

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.Install(nodes);

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
        std::cout << "Flow " << it->first << " (" << tuple.sourceAddress << " -> "
                  << tuple.destinationAddress << ")\n"
                  << "  Tx Packets: " << it->second.txPackets << "\n"
                  << "  Rx Packets: " << it->second.rxPackets << "\n"
                  << "  Lost Packets: " << it->second.lostPackets << "\n";

        if (it->second.rxPackets > 0)
        {
            std::cout << "  Throughput: "
                      << (it->second.rxBytes * 8.0 / (SIMULATION_STOP * 1000000.0)) << " Mbps\n";
        }
    }

    for (uint32_t id = 1; id <= 3; ++id)
    {
        if (!g_taskCompleted[id])
        {
            std::cout << "WARNING: Drone " << id << " did not report arrival at House "
                      << g_droneDestinations[id].name << " before simulation ended." << std::endl;
        }
    }

    std::cout << "NetAnim file: warehouse-animation.xml\n";
    std::cout << "=========================================\n";

    monitor->SerializeToXmlFile("warehouse-flowmon.xml", true, true);
    Simulator::Destroy();
    return 0;
}
