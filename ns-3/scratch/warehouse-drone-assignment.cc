#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/netanim-module.h"
#include "ns3/network-module.h"
#include "ns3/wifi-module.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <queue>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WarehouseDroneAssignment");

static const uint16_t PORT = 9000;
static const double STOP_TIME = 60.0;
static const double DRONE_SPEED = 8.0;
static const double CHECK_INTERVAL = 0.1;
static const uint32_t DRONE_COUNT = 3;

static NodeContainer g_nodes;
static Ptr<Socket> g_sockets[5];
static Ipv4Address g_addresses[5];

static std::map<std::string, Vector> g_houses;
static std::queue<std::string> g_pendingRequests;
static bool g_busy[4] = {false, false, false, false};
static bool g_taskStarted[4] = {false, false, false, false};

static void TryAssignPendingRequest();
static void CheckDroneArrival(uint32_t droneId);

static bool
ParseHouse(const std::string& message, std::string& house)
{
    std::size_t pos = message.find("HOUSE=");
    if (pos == std::string::npos)
    {
        return false;
    }

    house = message.substr(pos + 6);
    return g_houses.find(house) != g_houses.end();
}

static void
SendMessage(uint32_t senderId, Ipv4Address destination, const std::string& message)
{
    Ptr<Packet> packet = Create<Packet>(reinterpret_cast<const uint8_t*>(message.data()),
                                        static_cast<uint32_t>(message.size()));

    int result = g_sockets[senderId]->SendTo(packet, 0, InetSocketAddress(destination, PORT));

    std::cout << "[" << Simulator::Now().GetSeconds() << "s] "
              << (result >= 0 ? "UDP sent: " : "UDP send failed: ") << message << std::endl;
}

static void
TryAssignPendingRequest()
{
    while (!g_pendingRequests.empty())
    {
        std::string house = g_pendingRequests.front();

        double bestDistance = 1e30;
        int selectedDrone = -1;

        Vector target = g_houses.at(house);

        // Choose the nearest drone that is not currently busy.
        for (uint32_t id = 1; id <= DRONE_COUNT; ++id)
        {
            if (g_busy[id])
            {
                std::cout << "Drone " << id << " is BUSY; skipping.\n";
                continue;
            }

            Vector current = g_nodes.Get(id)->GetObject<MobilityModel>()->GetPosition();

            double dx = target.x - current.x;
            double dy = target.y - current.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            std::cout << "Drone " << id << " is AVAILABLE; distance to House " << house << " = "
                      << distance << " m\n";

            if (distance < bestDistance)
            {
                bestDistance = distance;
                selectedDrone = static_cast<int>(id);
            }
        }

        // Keep the request queued until a drone becomes available.
        if (selectedDrone < 0)
        {
            std::cout << "All drones are busy. House " << house << " request remains queued.\n";
            return;
        }

        g_pendingRequests.pop();
        g_busy[selectedDrone] = true;

        std::cout << "CONTROLLER: Assigning House " << house << " to Drone " << selectedDrone
                  << ". Drone marked BUSY.\n\n";

        SendMessage(0, g_addresses[selectedDrone], "TASK:HOUSE=" + house);
    }
}

static void
CheckDroneArrival(uint32_t droneId)
{
    Ptr<ConstantVelocityMobilityModel> mobility =
        g_nodes.Get(droneId)->GetObject<ConstantVelocityMobilityModel>();

    Vector current = mobility->GetPosition();

    // Find the target stored for this drone in its assigned task.
    // The destination is recorded in the node description-independent map.
    static std::map<uint32_t, std::string> unused;
    (void)unused;

    // The task target is kept in a dedicated per-drone table below.
    extern std::map<uint32_t, std::string> g_droneTargets;
    std::string house = g_droneTargets.at(droneId);
    Vector target = g_houses.at(house);

    double dx = target.x - current.x;
    double dy = target.y - current.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if (distance <= DRONE_SPEED * CHECK_INTERVAL)
    {
        mobility->SetPosition(target);
        mobility->SetVelocity(Vector(0, 0, 0));

        std::cout << "[" << Simulator::Now().GetSeconds() << "s] Drone " << droneId
                  << " reached House " << house << ".\n";

        SendMessage(droneId, g_addresses[0], "STATUS:COMPLETE:HOUSE=" + house);
        return;
    }

    mobility->SetVelocity(Vector(DRONE_SPEED * dx / distance, DRONE_SPEED * dy / distance, 0));

    Simulator::Schedule(Seconds(CHECK_INTERVAL), &CheckDroneArrival, droneId);
}

std::map<uint32_t, std::string> g_droneTargets;

static void
ReceivePacket(Ptr<Socket> socket)
{
    Address from;

    while (Ptr<Packet> packet = socket->RecvFrom(from))
    {
        uint32_t size = packet->GetSize();
        std::string message(size, '\0');

        if (size > 0)
        {
            packet->CopyData(reinterpret_cast<uint8_t*>(&message[0]), size);
        }

        uint32_t nodeId = socket->GetNode()->GetId();

        std::cout << "[" << Simulator::Now().GetSeconds() << "s] Node " << nodeId
                  << " received: " << message << '\n';

        if (nodeId == 0 && message.rfind("REQUEST:", 0) == 0)
        {
            std::string house;

            if (ParseHouse(message, house))
            {
                g_pendingRequests.push(house);

                std::cout << "Controller queued delivery request for House " << house << ".\n";

                TryAssignPendingRequest();
            }
        }
        else if (nodeId >= 1 && nodeId <= DRONE_COUNT && message.rfind("TASK:", 0) == 0 &&
                 !g_taskStarted[nodeId])
        {
            std::string house;

            if (!ParseHouse(message, house))
            {
                return;
            }

            g_taskStarted[nodeId] = true;
            g_droneTargets[nodeId] = house;

            Ptr<ConstantVelocityMobilityModel> mobility =
                g_nodes.Get(nodeId)->GetObject<ConstantVelocityMobilityModel>();

            Vector current = mobility->GetPosition();
            Vector target = g_houses.at(house);

            double dx = target.x - current.x;
            double dy = target.y - current.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            std::cout << "Drone " << nodeId << " accepted delivery to House " << house << ".\n";

            if (distance <= 0.5)
            {
                mobility->SetVelocity(Vector(0, 0, 0));
                SendMessage(nodeId, g_addresses[0], "STATUS:COMPLETE:HOUSE=" + house);
            }
            else
            {
                mobility->SetVelocity(
                    Vector(DRONE_SPEED * dx / distance, DRONE_SPEED * dy / distance, 0));

                Simulator::Schedule(Seconds(CHECK_INTERVAL), &CheckDroneArrival, nodeId);
            }
        }
        else if (nodeId == 0 && message.rfind("STATUS:COMPLETE:", 0) == 0)
        {
            Address source = from;
            Ipv4Address sourceAddress = InetSocketAddress::ConvertFrom(source).GetIpv4();

            int droneId = -1;
            for (uint32_t id = 1; id <= DRONE_COUNT; ++id)
            {
                if (sourceAddress == g_addresses[id])
                {
                    droneId = static_cast<int>(id);
                    break;
                }
            }

            if (droneId > 0 && g_busy[droneId])
            {
                std::cout << "CONTROLLER: Drone " << droneId
                          << " completed delivery and is now AVAILABLE.\n\n";

                g_busy[droneId] = false;
                g_taskStarted[droneId] = false;

                TryAssignPendingRequest();
            }
        }
    }
}

int
main(int argc, char* argv[])
{
    CommandLine cmd(__FILE__);
    cmd.Parse(argc, argv);

    std::vector<std::string> requests;
    uint32_t requestCount = 0;

    std::cout << "\n=== SMART WAREHOUSE: DELIVERY QUEUE ===\n";
    std::cout << "Enter the number of delivery requests (1-10): ";

    while (!(std::cin >> requestCount) || requestCount < 1 || requestCount > 10)
    {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "Enter a number between 1 and 10: ";
    }

    for (uint32_t i = 0; i < requestCount; ++i)
    {
        std::string destination;

        while (true)
        {
            std::cout << "Request " << i + 1 << " destination (A/B/C): ";

            std::cin >> destination;

            std::transform(destination.begin(),
                           destination.end(),
                           destination.begin(),
                           [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

            if (destination == "A" || destination == "B" || destination == "C")
            {
                requests.push_back(destination);
                break;
            }

            std::cout << "Invalid destination. Enter A, B, or C.\n";
        }
    }

    std::cout << "\nDelivery queue:\n";

    for (uint32_t i = 0; i < requests.size(); ++i)
    {
        std::cout << i + 1 << ". House " << requests[i] << '\n';
    }

    std::cout << '\n';

    g_houses["A"] = Vector(70, 20, 0);
    g_houses["B"] = Vector(70, 50, 0);
    g_houses["C"] = Vector(20, 70, 0);

    std::cout << "\nDelivery queue:\n";
    for (uint32_t i = 0; i < 3; ++i)
    {
        std::cout << i + 1 << ". House " << requests[i] << '\n';
    }
    std::cout << '\n';

    // Nodes: controller, drones 1-3, delivery request node.
    g_nodes.Create(5);

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

    NetDeviceContainer devices = wifi.Install(phy, mac, g_nodes);

    InternetStackHelper internet;
    internet.Install(g_nodes);

    Ipv4AddressHelper address;
    address.SetBase("10.2.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    for (uint32_t i = 0; i < g_nodes.GetN(); ++i)
    {
        g_addresses[i] = interfaces.GetAddress(i);
    }

    Ptr<ListPositionAllocator> positions = CreateObject<ListPositionAllocator>();

    positions->Add(Vector(20, 20, 0)); // Controller
    positions->Add(Vector(50, 20, 0)); // Drone 1
    positions->Add(Vector(50, 50, 0)); // Drone 2
    positions->Add(Vector(20, 50, 0)); // Drone 3
    positions->Add(Vector(15, 20, 0)); // Request node

    MobilityHelper mobility;
    mobility.SetPositionAllocator(positions);
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(g_nodes);

    for (uint32_t i = 0; i < g_nodes.GetN(); ++i)
    {
        g_nodes.Get(i)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0, 0, 0));
    }

    for (uint32_t i = 0; i < g_nodes.GetN(); ++i)
    {
        g_sockets[i] = Socket::CreateSocket(g_nodes.Get(i), UdpSocketFactory::GetTypeId());

        if (g_sockets[i]->Bind(InetSocketAddress(Ipv4Address::GetAny(), PORT)) != 0)
        {
            NS_FATAL_ERROR("Could not bind UDP socket for node " << i);
        }

        g_sockets[i]->SetRecvCallback(MakeCallback(&ReceivePacket));
    }

    // Houses are visual markers in NetAnim, not networked devices.
    NodeContainer houseNodes;
    houseNodes.Create(3);

    Ptr<ListPositionAllocator> housePositions = CreateObject<ListPositionAllocator>();

    housePositions->Add(g_houses["A"]);
    housePositions->Add(g_houses["B"]);
    housePositions->Add(g_houses["C"]);

    MobilityHelper houseMobility;
    houseMobility.SetPositionAllocator(housePositions);
    houseMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    houseMobility.Install(houseNodes);

    AnimationInterface animation("warehouse-assignment-animation.xml");
    animation.EnablePacketMetadata(true);
    animation.SetMobilityPollInterval(MilliSeconds(100));

    animation.UpdateNodeDescription(0, "Controller");
    animation.UpdateNodeDescription(1, "Drone 1");
    animation.UpdateNodeDescription(2, "Drone 2");
    animation.UpdateNodeDescription(3, "Drone 3");
    animation.UpdateNodeDescription(4, "Delivery Request Node");

    animation.UpdateNodeColor(0, 30, 100, 220);
    animation.UpdateNodeColor(1, 20, 170, 70);
    animation.UpdateNodeColor(2, 20, 170, 70);
    animation.UpdateNodeColor(3, 20, 170, 70);
    animation.UpdateNodeColor(4, 180, 60, 200);

    animation.UpdateNodeDescription(5, "House A");
    animation.UpdateNodeDescription(6, "House B");
    animation.UpdateNodeDescription(7, "House C");

    animation.UpdateNodeColor(5, 220, 40, 40);
    animation.UpdateNodeColor(6, 220, 40, 40);
    animation.UpdateNodeColor(7, 220, 40, 40);

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> monitor = flowmonHelper.Install(g_nodes);

    // Requests are transmitted sequentially over UDP.
    for (uint32_t i = 0; i < requests.size(); ++i)
    {
        std::string destination = requests[i];

        Simulator::Schedule(Seconds(1.0 + i * 0.5), [destination]() {
            SendMessage(4, g_addresses[0], "REQUEST:HOUSE=" + destination);
        });
    }

    Simulator::Stop(Seconds(STOP_TIME));
    Simulator::Run();

    monitor->CheckForLostPackets();

    std::cout << "\n=== FLOW MONITOR SUMMARY ===\n";
    auto stats = monitor->GetFlowStats();

    for (const auto& entry : stats)
    {
        std::cout << "Flow " << entry.first << ": TX=" << entry.second.txPackets
                  << ", RX=" << entry.second.rxPackets << ", Lost=" << entry.second.lostPackets
                  << '\n';
    }

    std::cout << "\nAnimation saved to warehouse-assignment-animation.xml\n";

    Simulator::Destroy();
    return 0;
}
