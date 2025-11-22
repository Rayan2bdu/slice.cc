#include <ns3/core-module.h>
#include <ns3/network-module.h>
#include <ns3/mobility-module.h>
#include <ns3/nr-module.h>
#include <ns3/applications-module.h>
#include <ns3/internet-module.h>
#include <ns3/flow-monitor-module.h>
#include "ns3/antenna-module.h"
#include "ns3/csma-module.h"  // For PcapHelper
#include "ns3/netanim-module.h"
#include <fstream>
#include <iostream>


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("NetworkSlicingDemo");

int main(int argc, char *argv[])
{
    // Simulation parameters
    double simTime = 1.0; // sekunder
    double centralFrequency = 28e9; // 28 GHz
    double bandwidth = 100e6; // 100 MHz
    uint16_t numerology = 2;  //numerology

    // Create nodes
    NodeContainer gNbNodes;
    NodeContainer ueNodes;
    gNbNodes.Create(1); //we will have ett gnb
    ueNodes.Create(6); // Now 6 UEs, 2 mobile and 4 static

    // Setup mobility - Mixed mobility for different UE types
    MobilityHelper mobility;
    
    // Mobile UEs (IndustrialRobot and AutonomousDrone) - URLLC
    MobilityHelper mobileMobility;
   
                                   
    mobileMobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                               "Bounds", RectangleValue(Rectangle(-50, 50, -50, 50)), // Smaller area
                               "Speed", StringValue("ns3::ConstantRandomVariable[Constant=8.0]"), // Faster: 8 m/s
                               "Time", TimeValue(Seconds(0.0)),  // No pause - immediate movement
                               "Mode", StringValue("Time"));     // Time-based mobility
    
    MobilityHelper staticMobility;
    staticMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>(); //lest use vector for position allocation
    
    // gNB position
    positionAlloc->Add(Vector(0.0, 0.0, 30.0));
    
    // UE positions - 6 UEs with specific placements
    // URLLC UEs (Mobile)
    positionAlloc->Add(Vector(30.0, 0.0, 1.5));    // UE0: IndustrialRobot
    positionAlloc->Add(Vector(0.0, 30.0, 1.5));    // UE1: AutonomousDrone
    
    // eMBB UEs (Static)
    positionAlloc->Add(Vector(60.0, 0.0, 1.5));    // UE2: 8KVideo
    positionAlloc->Add(Vector(0.0, 60.0, 1.5));    // UE3: VRHeadset
    
    // mMTC UEs (Static)
    positionAlloc->Add(Vector(60.0, 60.0, 1.5));   // UE4: SmartMeter
    positionAlloc->Add(Vector(30.0, 60.0, 1.5));   // UE5: WeatherSensor
    
    // Install mobility
    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(gNbNodes);
    
    // Apply different mobility models to different UEs
    staticMobility.Install(ueNodes.Get(0)); // IndustrialRobot - It wil be mobile and keep moving
    staticMobility.Install(ueNodes.Get(1)); // AutonomousDrone - mobile and keep movingn

    staticMobility.Install(ueNodes.Get(2)); // 8KVideo - static
    staticMobility.Install(ueNodes.Get(3)); // VRHeadset - static
    staticMobility.Install(ueNodes.Get(4)); // SmartMeter - static
    staticMobility.Install(ueNodes.Get(5)); // WeatherSensor - static
    
    // Make URLLC UEs mobile
    mobileMobility.Install(ueNodes.Get(0)); // IndustrialRobot mobile
    mobileMobility.Install(ueNodes.Get(1)); // AutonomousDrone mobile
    
   

    // Set names for the UEs for identification
    Names::Add("IndustrialRobot", ueNodes.Get(0));
    Names::Add("AutonomousDrone", ueNodes.Get(1));
    Names::Add("8KVideo", ueNodes.Get(2));
    Names::Add("VRHeadset", ueNodes.Get(3));
    Names::Add("SmartMeter", ueNodes.Get(4));
    Names::Add("WeatherSensor", ueNodes.Get(5));

    // Create NR helpers //here game startes :)
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    // Configure helpers  beamforming helper
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(epcHelper);

    // Configure the spectrum
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;

    // Create the configuration for the CcBwpHelper ,, will be used for slicing
    CcBwpCreator::SimpleOperationBandConf bandConf(centralFrequency, bandwidth, numCcPerBand);
    
    // Create the band required
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);

    // Configure the channels
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories("UMi", "Default", "ThreeGpp");
    channelHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    channelHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));
    
    // Assign channels to bands  
    channelHelper->AssignChannelsToBands({band});

    // Get all BWPs from the band
    BandwidthPartInfoPtrVector allBwps = CcBwpCreator::GetAllBwps({band});

    // Configure common attributes
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));
    
    // Configure antenna attributes,, important for antennas configuration
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement", 
                                   PointerValue(CreateObject<IsotropicAntennaModel>()));
    
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(4));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement", 
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Install devices
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gNbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);
    
    //Lets also install PCAP logging for packet analysis using wireshark
         // nrHelper->EnableTraces(); //for all traces
         nrHelper->EnablePdcpE2eTraces(); //for IP traces

    
    // Configure numerology and power for all gNB PHY instances
    for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
    {
        for (uint32_t bwpId = 0; bwpId < allBwps.size(); ++bwpId)
        {
            nrHelper->GetGnbPhy(gnbNetDev.Get(i), bwpId)->SetAttribute("Numerology", 
                                                                      UintegerValue(numerology));
            nrHelper->GetGnbPhy(gnbNetDev.Get(i), bwpId)->SetAttribute("TxPower", 
                                                                      DoubleValue(23.0)); // 200 mW
        }
    }

    // Install internet stack,, lets install stack and IP
    InternetStackHelper internet;
    internet.Install(ueNodes);

    // Assign IP addresses
    Ipv4InterfaceContainer ueIpIface = epcHelper->AssignUeIpv4Address(ueNetDev);

    // Attach UEs to gNB,, all UE to gNB
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);

    // Setup remote host
    auto [remoteHost, remoteHostIpv4Address] = epcHelper->SetupRemoteHost("1Gb/s", 2500, Seconds(0.000));

    // Install applications for network slicing demonstration
    ApplicationContainer serverApps, clientApps;

    // Define ports for different slice types,, slicing starts
    uint16_t portUrllc1 = 1234;  // IndustrialRobot
    uint16_t portUrllc2 = 1235;  // AutonomousDrone
    uint16_t portEmbb1 = 1236;   // 8KVideo
    uint16_t portEmbb2 = 1237;   // VRHeadset
    uint16_t portMmtc1 = 1238;   // SmartMeter
    uint16_t portMmtc2 = 1239;   // WeatherSensor

    // Install servers on UEs
    UdpServerHelper serverUrllc1(portUrllc1);
    UdpServerHelper serverUrllc2(portUrllc2);
    UdpServerHelper serverEmbb1(portEmbb1);
    UdpServerHelper serverEmbb2(portEmbb2);
    UdpServerHelper serverMmtc1(portMmtc1);
    UdpServerHelper serverMmtc2(portMmtc2);

    serverApps.Add(serverUrllc1.Install(ueNodes.Get(0))); // IndustrialRobot
    serverApps.Add(serverUrllc2.Install(ueNodes.Get(1))); // AutonomousDrone
    serverApps.Add(serverEmbb1.Install(ueNodes.Get(2)));  // 8KVideo
    serverApps.Add(serverEmbb2.Install(ueNodes.Get(3)));  // VRHeadset
    serverApps.Add(serverMmtc1.Install(ueNodes.Get(4)));  // SmartMeter
    serverApps.Add(serverMmtc2.Install(ueNodes.Get(5)));  // WeatherSensor

    // Configure clients for different slice types
    UdpClientHelper clientUrllc1, clientUrllc2;  // URLLC clients
    UdpClientHelper clientEmbb1, clientEmbb2;    // eMBB clients
    UdpClientHelper clientMmtc1, clientMmtc2;    // mMTC clients

    // URLLC: IndustrialRobot and AutonomousDrone - Low latency, small packets
    clientUrllc1.SetAttribute("MaxPackets", UintegerValue(1000000));
    clientUrllc1.SetAttribute("PacketSize", UintegerValue(100));
    clientUrllc1.SetAttribute("Interval", TimeValue(Seconds(0.000001))); // 1ms interval //0.001

    clientUrllc2.SetAttribute("MaxPackets", UintegerValue(1000000));
    clientUrllc2.SetAttribute("PacketSize", UintegerValue(80));
    clientUrllc2.SetAttribute("Interval", TimeValue(Seconds(0.0000005))); // 0.5ms interval //0.0005

    // eMBB: 8KVideo and VRHeadset - High throughput, large packets
    clientEmbb1.SetAttribute("MaxPackets", UintegerValue(1000000));
    clientEmbb1.SetAttribute("PacketSize", UintegerValue(1400));
    clientEmbb1.SetAttribute("Interval", TimeValue(Seconds(0.000001))); // 0.1ms interval //0.0001

    clientEmbb2.SetAttribute("MaxPackets", UintegerValue(1000000));
    clientEmbb2.SetAttribute("PacketSize", UintegerValue(1200));
    clientEmbb2.SetAttribute("Interval", TimeValue(Seconds(0.0000002))); // 0.2ms interval 0.0002

    // mMTC: SmartMeter and WeatherSensor - IoT traffic, infrequent small packets
    clientMmtc1.SetAttribute("MaxPackets", UintegerValue(1000));
    clientMmtc1.SetAttribute("PacketSize", UintegerValue(40));
    clientMmtc1.SetAttribute("Interval", TimeValue(Seconds(0.1))); // 1 second interval

    clientMmtc2.SetAttribute("MaxPackets", UintegerValue(500));
    clientMmtc2.SetAttribute("PacketSize", UintegerValue(30));
    clientMmtc2.SetAttribute("Interval", TimeValue(Seconds(0.1))); // 2 seconds interval

    // Install clients on remote host targeting specific UEs
    clientUrllc1.SetAttribute("Remote", AddressValue(InetSocketAddress(ueIpIface.GetAddress(0), portUrllc1)));
    clientUrllc2.SetAttribute("Remote", AddressValue(InetSocketAddress(ueIpIface.GetAddress(1), portUrllc2)));
    clientEmbb1.SetAttribute("Remote", AddressValue(InetSocketAddress(ueIpIface.GetAddress(2), portEmbb1)));
    clientEmbb2.SetAttribute("Remote", AddressValue(InetSocketAddress(ueIpIface.GetAddress(3), portEmbb2)));
    clientMmtc1.SetAttribute("Remote", AddressValue(InetSocketAddress(ueIpIface.GetAddress(4), portMmtc1)));
    clientMmtc2.SetAttribute("Remote", AddressValue(InetSocketAddress(ueIpIface.GetAddress(5), portMmtc2)));

    clientApps.Add(clientUrllc1.Install(remoteHost));
    clientApps.Add(clientUrllc2.Install(remoteHost));
    clientApps.Add(clientEmbb1.Install(remoteHost));
    clientApps.Add(clientEmbb2.Install(remoteHost));
    clientApps.Add(clientMmtc1.Install(remoteHost));
    clientApps.Add(clientMmtc2.Install(remoteHost));

    // Setup dedicated bearers for different QoS - NETWORK SLICING CONCEPT
    // URLLC bearers,, Slices 
    NrEpsBearer urllcBearer(NrEpsBearer::GBR_CONV_VOICE);
    
    Ptr<NrEpcTft> urllcTft1 = Create<NrEpcTft>();
    NrEpcTft::PacketFilter urllcPf1;
    urllcPf1.localPortStart = portUrllc1;
    urllcPf1.localPortEnd = portUrllc1;
    urllcTft1->Add(urllcPf1);

    Ptr<NrEpcTft> urllcTft2 = Create<NrEpcTft>();
    NrEpcTft::PacketFilter urllcPf2;
    urllcPf2.localPortStart = portUrllc2;
    urllcPf2.localPortEnd = portUrllc2;
    urllcTft2->Add(urllcPf2);

    // eMBB bearers  slices
    NrEpsBearer embbBearer(NrEpsBearer::NGBR_VIDEO_TCP_DEFAULT);
    
    Ptr<NrEpcTft> embbTft1 = Create<NrEpcTft>();
    NrEpcTft::PacketFilter embbPf1;
    embbPf1.localPortStart = portEmbb1;
    embbPf1.localPortEnd = portEmbb1;
    embbTft1->Add(embbPf1);

    Ptr<NrEpcTft> embbTft2 = Create<NrEpcTft>();
    NrEpcTft::PacketFilter embbPf2;
    embbPf2.localPortStart = portEmbb2;
    embbPf2.localPortEnd = portEmbb2;
    embbTft2->Add(embbPf2);

    // mMTC bearers  slices
    NrEpsBearer mmtcBearer(NrEpsBearer::NGBR_MC_DELAY_SIGNAL);
    
    Ptr<NrEpcTft> mmtcTft1 = Create<NrEpcTft>();
    NrEpcTft::PacketFilter mmtcPf1;
    mmtcPf1.localPortStart = portMmtc1;
    mmtcPf1.localPortEnd = portMmtc1;
    mmtcTft1->Add(mmtcPf1);

    Ptr<NrEpcTft> mmtcTft2 = Create<NrEpcTft>();
    NrEpcTft::PacketFilter mmtcPf2;
    mmtcPf2.localPortStart = portMmtc2;
    mmtcPf2.localPortEnd = portMmtc2;
    mmtcTft2->Add(mmtcPf2);

    // Activate the slices sgainst each type of UE
    // URLLC UEs for Industrial Robot and Autonomous Drone
    nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(0), urllcBearer, urllcTft1); // IndustrialRobot
    nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(1), urllcBearer, urllcTft2); // AutonomousDrone
    
    // eMBB UEs  high datartae for 8KVideo and VRHEADSet
    nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(2), embbBearer, embbTft1); // 8KVideo
    nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(3), embbBearer, embbTft2); // VRHeadset
    
    // mMTC UEs,, mMTC for smart meter and weather sensor
    nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(4), mmtcBearer, mmtcTft1); // SmartMeter
    nrHelper->ActivateDedicatedEpsBearer(ueNetDev.Get(5), mmtcBearer, mmtcTft2); // WeatherSensor

    // Start applications
    serverApps.Start(Seconds(0.1));
    clientApps.Start(Seconds(0.2));
    serverApps.Stop(Seconds(simTime));
    clientApps.Stop(Seconds(simTime));

    // Enable flow monitor  to calculate stats
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    
       //Lets add NetAnim Code for Animation purpsoe
    AnimationInterface anim("network_slicing_animation.xml");

    // Customize animation for better visualization
    anim.SetMaxPktsPerTraceFile(500000);
    anim.EnablePacketMetadata(true);

    // Set node descriptions
    anim.UpdateNodeDescription(gNbNodes.Get(0), "gNB");
    anim.UpdateNodeDescription(ueNodes.Get(0), "IndustrialRobot");
    anim.UpdateNodeDescription(ueNodes.Get(1), "AutonomousDrone"); 
    anim.UpdateNodeDescription(ueNodes.Get(2), "8KVideo");
    anim.UpdateNodeDescription(ueNodes.Get(3), "VRHeadset");
    anim.UpdateNodeDescription(ueNodes.Get(4), "SmartMeter");
    anim.UpdateNodeDescription(ueNodes.Get(5), "WeatherSensor");

    // Set node colors based on slice type
    anim.UpdateNodeColor(gNbNodes.Get(0), 0, 0, 255); // Blue for gNB
    anim.UpdateNodeColor(ueNodes.Get(0), 255, 0, 0);   // Red for URLLC
    anim.UpdateNodeColor(ueNodes.Get(1), 255, 0, 0);   // Red for URLLC  
    anim.UpdateNodeColor(ueNodes.Get(2), 0, 255, 0);   // Green for eMBB
    anim.UpdateNodeColor(ueNodes.Get(3), 0, 255, 0);   // Green for eMBB
    anim.UpdateNodeColor(ueNodes.Get(4), 255, 165, 0); // Orange for mMTC
    anim.UpdateNodeColor(ueNodes.Get(5), 255, 165, 0); // Orange for mMTC
    // for mathplotlib
    AsciiTraceHelper ascii;
    Ptr<OutputStreamWrapper> mobilityStream = ascii.CreateFileStream("mobility.txt");
    mobility.EnableAsciiAll(mobilityStream);
    // Run simulation
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // Calculate and print statistics
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats();

    std::cout << "\n=== NETWORK SLICING SIMULATION RESULTS (6 UEs) ===" << std::endl;
    std::cout << "Device Name        | Slice Type | Throughput (Mbps) | Avg Delay (ms) | Packet Loss (%)" << std::endl;
    std::cout << "----------------------------------------------------------------------------------------" << std::endl;

    // Map ports to device names
    std::map<uint16_t, std::string> portToDevice = {
        {portUrllc1, "IndustrialRobot"},
        {portUrllc2, "AutonomousDrone"},
        {portEmbb1, "8KVideo"},
        {portEmbb2, "VRHeadset"},
        {portMmtc1, "SmartMeter"},
        {portMmtc2, "WeatherSensor"}
    };

    std::map<uint16_t, std::string> portToSlice = {
        {portUrllc1, "URLLC"}, {portUrllc2, "URLLC"},
        {portEmbb1, "eMBB"}, {portEmbb2, "eMBB"},
        {portMmtc1, "mMTC"}, {portMmtc2, "mMTC"}
    };

    for (auto &flow : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
        
        if (portToDevice.find(t.destinationPort) == portToDevice.end()) continue;
        
        std::string deviceName = portToDevice[t.destinationPort];
        std::string sliceType = portToSlice[t.destinationPort];

        double throughput = flow.second.rxBytes * 8.0 / (simTime - 0.2) / 1e6;
        double avgDelay = 0.0;
        if (flow.second.rxPackets > 0)
        {
            avgDelay = flow.second.delaySum.GetSeconds() * 1000 / flow.second.rxPackets;
        }
        double packetLoss = 0.0;
        if (flow.second.txPackets > 0)
        {
            packetLoss = (flow.second.txPackets - flow.second.rxPackets) * 100.0 / flow.second.txPackets;
        }

        std::cout << std::setw(17) << deviceName << " | "
                  << std::setw(10) << sliceType << " | "
                  << std::setw(16) << std::fixed << std::setprecision(2) << throughput << " | "
                  << std::setw(13) << std::setprecision(3) << avgDelay << " | "
                  << std::setw(14) << std::setprecision(2) << packetLoss << std::endl;
    }

    std::cout << "\n=== NETWORK CONFIGURATION ===" << std::endl;
    std::cout << "gNB: 1 node" << std::endl;
    std::cout << "UEs: 6 nodes with mixed mobility" << std::endl;
    std::cout << "  - URLLC (Mobile): IndustrialRobot, AutonomousDrone" << std::endl;
    std::cout << "  - eMBB (Static): 8KVideo, VRHeadset" << std::endl;
    std::cout << "  - mMTC (Static): SmartMeter, WeatherSensor" << std::endl;
    std::cout << "Frequency: " << centralFrequency/1e9 << " GHz" << std::endl;
    std::cout << "Bandwidth: " << bandwidth/1e6 << " MHz" << std::endl;
    std::cout << "Numerology: " << numerology << std::endl;
    std::cout << "Simulation Time: " << simTime << " seconds" << std::endl;

    Simulator::Destroy();
    return 0;
}

