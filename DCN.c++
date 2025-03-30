#include <ns3/core-module.h>
#include <ns3/csma-module.h>
#include <ns3/internet-apps-module.h>
#include <ns3/internet-module.h>
#include <ns3/network-module.h>
#include <ns3/ofswitch13-module.h>
#include <ns3/netanim-module.h>
#include <ns3/mobility-module.h>

using namespace ns3;

class TrafficController;

int main(int argc, char* argv[])
{
    uint16_t simTime = 20; // Increased simulation time for drone movement
    bool verbose = false;
    bool trace = false;

    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation time (seconds)", simTime);
    cmd.AddValue("verbose", "Enable verbose output", verbose);
    cmd.AddValue("trace", "Enable datapath stats and pcap traces", trace);
    cmd.Parse(argc, argv);

    if (verbose)
    {
        OFSwitch13Helper::EnableDatapathLogs();
        LogComponentEnable("OFSwitch13Interface", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13Device", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13Port", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13Queue", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13SocketHandler", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13Controller", LOG_LEVEL_ALL);
        LogComponentEnable("TrafficController", LOG_LEVEL_ALL);
        LogComponentEnable("OFSwitch13InternalHelper", LOG_LEVEL_ALL);
    }

    // Create five drone nodes
    NodeContainer drones;
    drones.Create(5);

    // Create the switch and controller nodes
    Ptr<Node> switchNode = CreateObject<Node>();
    Ptr<Node> controllerNode = CreateObject<Node>();

    // Use the CsmaHelper to connect drones to the switch node
    CsmaHelper csmaHelper;
    csmaHelper.SetChannelAttribute("DataRate", DataRateValue(DataRate("100Mbps")));
    csmaHelper.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));

    NetDeviceContainer droneDevices;
    NetDeviceContainer switchPorts;
    for (size_t i = 0; i < drones.GetN(); i++)
    {
        NodeContainer pair(drones.Get(i), switchNode);
        NetDeviceContainer link = csmaHelper.Install(pair);
        droneDevices.Add(link.Get(0));
        switchPorts.Add(link.Get(1));
    }

    // Configure the OpenFlow network domain
    Ptr<OFSwitch13InternalHelper> of13Helper = CreateObject<OFSwitch13InternalHelper>();
    Ptr<TrafficController> trafficController = CreateObject<TrafficController>();
    of13Helper->InstallController(controllerNode, trafficController);
    of13Helper->InstallSwitch(switchNode, switchPorts);
    of13Helper->CreateOpenFlowChannels();

    // Install the TCP/IP stack into drone nodes
    InternetStackHelper internet;
    internet.Install(drones);

    // Set IPv4 addresses
    Ipv4AddressHelper ipv4Helper;
    ipv4Helper.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer droneIpIfaces = ipv4Helper.Assign(droneDevices);

    // Assign 3D mobility to each drone node
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (size_t i = 0; i < drones.GetN(); i++)
    {
        positionAlloc->Add(Vector(10 + i * 20, 10, 50)); // Initial positions in 3D space
    }
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel("ns3::WaypointMobilityModel");

    for (size_t i = 0; i < drones.GetN(); i++)
    {
        mobility.Install(drones.Get(i));
        Ptr<WaypointMobilityModel> mobilityModel = drones.Get(i)->GetObject<WaypointMobilityModel>();

        // Add waypoints for 3D movement
        mobilityModel->AddWaypoint(Waypoint(Seconds(0), Vector(10 + i * 20, 10, 50)));
        mobilityModel->AddWaypoint(Waypoint(Seconds(5), Vector(30 + i * 20, 40, 100)));
        mobilityModel->AddWaypoint(Waypoint(Seconds(10), Vector(50 + i * 20, 80, 150)));
    }

    // Set up mobility for switch node (static)
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(switchNode);
    Ptr<ConstantPositionMobilityModel> switchMobilityModel = switchNode->GetObject<ConstantPositionMobilityModel>();
    switchMobilityModel->SetPosition(Vector(50, 50, 0));

    // Set up mobility for controller node (static)
    mobility.Install(controllerNode);
    Ptr<ConstantPositionMobilityModel> controllerMobilityModel = controllerNode->GetObject<ConstantPositionMobilityModel>();
    controllerMobilityModel->Set