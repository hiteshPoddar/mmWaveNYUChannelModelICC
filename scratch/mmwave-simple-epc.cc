/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
*   Copyright (c) 2011 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
*   Copyright (c) 2015, NYU WIRELESS, Tandon School of Engineering, New York University
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation;
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program; if not, write to the Free Software
*   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*
*   Author: Marco Miozzo <marco.miozzo@cttc.es>
*           Nicola Baldo  <nbaldo@cttc.es>
*
*   Modified by: Marco Mezzavilla < mezzavilla@nyu.edu>
*                         Sourjya Dutta <sdutta@nyu.edu>
*                         Russell Ford <russell.ford@nyu.edu>
*                         Menglei Zhang <menglei@nyu.edu>
*/


#include "ns3/mmwave-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/command-line.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include <ns3/core-module.h>
#include <ns3/internet-module.h>
#include <ns3/point-to-point-helper.h>
#include <ns3/mobility-module.h>
#include <ns3/applications-module.h>
#include <ns3/node-list.h>
#include <ns3/lte-module.h>
#include <ns3/mmwave-module.h>
#include <ns3/trace-source-accessor.h>
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"

using namespace ns3;
using namespace mmwave;

struct AppSetupParams
{
  uint16_t basePort;
  uint32_t sendSize;
  DataRate rate;
  Time IPI;
  Time startTime;
  Time endTime;
  Time simTime;
};

class Utils
{
  public:
    static std::pair<Ptr<Node>, Ipv4InterfaceContainer> CreateInternet (Ptr<MmWavePointToPointEpcHelper> epcHelper);
    static Ipv4InterfaceContainer InstallUeInternet (Ptr<MmWavePointToPointEpcHelper> epcHelper, 
                                                     NodeContainer ueNodes, NetDeviceContainer ueNetDevices);
    static void InstallUdpApplication (Ptr<OutputStreamWrapper> traceStream, Ptr<Node> srcNode, 
                                       Ipv4Address targetAddress, AppSetupParams params);
    static void InstallUdpPacketSink (Ptr<OutputStreamWrapper> traceStream, Ipv4Address srcAddress, 
                                      Ptr<Node> targetNode, AppSetupParams params);
    static void SetTracesPath (std::string filePath);
    static double PacketByteSizeFromBitrateAndIpi (DataRate rate, Time IPI);
      
  };  

  class CallbackSinks
  {
    public:
      static void RxSinkHeader (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> packet, const Address &from,
                               const Address &to, const SeqTsSizeHeader& seqTs);
      static void TxSinkHeader (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> packet, const Address &from,
                               const Address &to, const SeqTsSizeHeader& seqTs);
  };

  std::pair<Ptr<Node>, Ipv4InterfaceContainer>
  Utils::CreateInternet (Ptr<MmWavePointToPointEpcHelper> epcHelper)
  {
    // Create the Internet by connecting remoteHost to pgw. Setup routing too
    Ptr<Node> pgw = epcHelper->GetPgwNode ();

    // Create remotehost
    NodeContainer remoteHostContainer;
    remoteHostContainer.Create (1);
    InternetStackHelper internet;
    internet.Install (remoteHostContainer);
    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    Ipv4InterfaceContainer internetIpIfaces;

    Ptr<Node> remoteHost = remoteHostContainer.Get (0);
    // Create the Internet
    PointToPointHelper p2ph;
    p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
    p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
    p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.001)));

    NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);

    Ipv4AddressHelper ipv4h;
    ipv4h.SetBase ("1.0.0.0", "255.255.0.0");
    internetIpIfaces = ipv4h.Assign (internetDevices);
    // interface 0 is localhost, 1 is the p2p device

    Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
    remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.255.0.0"), 1);

    return std::pair<Ptr<Node>, Ipv4InterfaceContainer> (remoteHost, internetIpIfaces);
  }

  Ipv4InterfaceContainer
  Utils::InstallUeInternet (Ptr<MmWavePointToPointEpcHelper> epcHelper, NodeContainer ueNodes, NetDeviceContainer ueNetDevices)
  {
    // Install the IP stack on the UEs
    InternetStackHelper internet;
    internet.Install (ueNodes);
    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = epcHelper->AssignUeIpv4Address (ueNetDevices);
    // Assign IP address to UEs, and install applications
    // Set the default gateway for the UE
    Ipv4StaticRoutingHelper ipv4RoutingHelper;

    for (uint32_t i = 0; i < ueNodes.GetN (); i++)
    {
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNodes.Get (i)->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }

    return ueIpIface;
  }

  // Installs an UDP server on a single node (srcNode) and connects the TxWithSeqTsSize TraceSource,
  // i.e., the trace source tracing transmitted packets, to the TxSinkHeader callback sink
  // The parameter traceStream indicates the output stream where TxSinkHeader will log its information
  void
  Utils::InstallUdpApplication (Ptr<OutputStreamWrapper> traceStream, Ptr<Node> srcNode, 
                                Ipv4Address targetAddress, AppSetupParams params)
  {
    ApplicationContainer app;
    UdpClientHelper client (targetAddress, params.basePort); 
    client.SetAttribute ("Interval", TimeValue (params.IPI));
    client.SetAttribute ("PacketSize", UintegerValue (PacketByteSizeFromBitrateAndIpi (params.rate, params.IPI)));
    client.SetAttribute ("MaxPackets", UintegerValue (std::numeric_limits <uint32_t>::max ()));
    app.Add (client.Install (srcNode));

    // Connect the TxWithSeqTsSize trace to the TxSinkHeader callback sink
    app.Get(0)->TraceConnectWithoutContext ("TxWithSeqTsSize", MakeBoundCallback (&CallbackSinks::TxSinkHeader, 
                                                                                  traceStream));
    app.Start (params.startTime);
    app.Stop (params.endTime);
  }

  // Installs an UDP client on a single node (targetNode) and connects the RxWithSeqTsSize TraceSource,
  // i.e., the trace source tracing received packets, to the RxSinkHeader callback sink
  // The parameter traceStream indicates the output stream where TxSinkHeader will log its information
  void
  Utils::InstallUdpPacketSink (Ptr<OutputStreamWrapper> traceStream, Ipv4Address srcAddress, 
                               Ptr<Node> targetNode, AppSetupParams params)
  {
    ApplicationContainer app;
    PacketSinkHelper packetSinkHelper ("ns3::UdpSocketFactory", 
                                        InetSocketAddress (srcAddress, params.basePort));
    app.Add (packetSinkHelper.Install (targetNode));

    // Connect the RxWithSeqTsSize trace to the RxSinkHeader callback sink
    app.Get (0)->TraceConnectWithoutContext ("RxWithSeqTsSize", MakeBoundCallback (&CallbackSinks::RxSinkHeader, 
                                                                                   traceStream));
    app.Start (Seconds (0));
    app.Stop (params.simTime);
  }

  double 
  Utils::PacketByteSizeFromBitrateAndIpi (DataRate rate, Time IPI)
  {
    double size = rate.GetBitRate () * IPI.GetSeconds ();
    return size / 8;
  }

  // Callback sink which logs received packets information
  void 
  CallbackSinks::RxSinkHeader (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> packet, const Address &from,
                               const Address &to, const SeqTsSizeHeader& seqTs)
  {
    *stream->GetStream () << "Rx\t" << Simulator::Now().GetNanoSeconds() << "\t" << seqTs.GetSize() 
                          << "\t" << (Simulator::Now() - seqTs.GetTs ()).GetNanoSeconds()
                          << "\t" << InetSocketAddress::ConvertFrom (from).GetIpv4()
                          << "\t" << InetSocketAddress::ConvertFrom (to).GetIpv4() 
                          << "\t" << InetSocketAddress::ConvertFrom (to).GetPort () << std::endl;
  }

  // Callback sink which logs transmitted packets information
  void 
  CallbackSinks::TxSinkHeader (Ptr<OutputStreamWrapper> stream, Ptr<const Packet> packet, const Address &from,
                               const Address &to, const SeqTsSizeHeader& seqTs)
  {
    *stream->GetStream () << "Tx\t" << Simulator::Now().GetNanoSeconds() << "\t" << seqTs.GetSize() 
                          << "\t" << (Simulator::Now() - seqTs.GetTs ()).GetNanoSeconds()
                          << "\t" << InetSocketAddress::ConvertFrom (from).GetIpv4()
                          << "\t" << InetSocketAddress::ConvertFrom (to).GetIpv4() 
                          << "\t" << InetSocketAddress::ConvertFrom (to).GetPort () << std::endl;

  }

/**
 * Sample simulation script for LTE+EPC. It instantiates several eNodeB,
 * attaches one UE per eNodeB starts a flow for each UE to  and from a remote host.
 * It also  starts yet another flow between each UE pair.
 */
NS_LOG_COMPONENT_DEFINE ("EpcFirstExample");

int
main (int argc, char *argv[])
{
  uint16_t numEnb = 1;
  uint16_t numUe = 1;
  double simTime = 10; // in seconds
  double minDistance = 10.0; // eNB-UE distance in meters
  double maxDistance = 150.0; // eNB-UE distance in meters
  bool harqEnabled = true;
  bool rlcAmEnabled = true;

  // Command line arguments
  CommandLine cmd;
  cmd.AddValue ("numEnb", "Number of eNBs", numEnb);
  cmd.AddValue ("numUe", "Number of UEs per eNB", numUe);
  cmd.AddValue ("simTime", "Total duration of the simulation [s])", simTime);
  cmd.AddValue ("harq", "Enable Hybrid ARQ", harqEnabled);
  cmd.AddValue ("rlcAm", "Enable RLC-AM", rlcAmEnabled);
  cmd.Parse (argc, argv);

  Config::SetDefault ("ns3::MmWaveHelper::RlcAmEnabled", BooleanValue (rlcAmEnabled));
  Config::SetDefault ("ns3::MmWaveHelper::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::MmWaveFlexTtiMacScheduler::HarqEnabled", BooleanValue (harqEnabled));
  Config::SetDefault ("ns3::LteRlcAm::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));
  Config::SetDefault ("ns3::LteRlcUmLowLat::ReportBufferStatusTimer", TimeValue (MicroSeconds (100.0)));
  Config::SetDefault ("ns3::PacketSink::EnableSeqTsSizeHeader", BooleanValue (true));
  Config::SetDefault ("ns3::LteRlcUmLowLat::MaxTxBufferSize", UintegerValue (100 * 1024));
  Config::SetDefault ("ns3::MmWaveSpectrumPhy::ErrorModelType", TypeIdValue (MmWaveEesmIrT1::GetTypeId ()));
  Config::SetDefault ("ns3::MmWaveAmc::ErrorModelType", TypeIdValue (MmWaveEesmIrT1::GetTypeId ()));

  Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper> ();
  mmwaveHelper->SetSchedulerType ("ns3::MmWaveFlexTtiMacScheduler");

  Ptr<MmWavePointToPointEpcHelper>  epcHelper = CreateObject<MmWavePointToPointEpcHelper> ();
  mmwaveHelper->SetEpcHelper (epcHelper);
  mmwaveHelper->SetHarqEnabled (harqEnabled);

 	// Create the eNB nodes
	NodeContainer gnbNodes;
	gnbNodes.Create (numEnb);

	// Create UE nodes
	NodeContainer ueNodes;
	ueNodes.Create (numUe);

  // Install Mobility Model
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  enbPositionAlloc->Add (Vector (0.0, 0.0, 0.0));
  MobilityHelper enbmobility;
  enbmobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbmobility.SetPositionAllocator (enbPositionAlloc);
  enbmobility.Install (gnbNodes);

  MobilityHelper uemobility;
  Ptr<ListPositionAllocator> uePositionAlloc = CreateObject<ListPositionAllocator> ();
  Ptr<UniformRandomVariable> distRv = CreateObject<UniformRandomVariable> ();
  for (unsigned i = 0; i < numUe; i++)
    {
      double dist = distRv->GetValue (minDistance, maxDistance);
      dist = 100; // 1 UE fixed at 100m
      uePositionAlloc->Add (Vector (dist, 0.0, 0.0));
    }
  uemobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  uemobility.SetPositionAllocator (uePositionAlloc);
  uemobility.Install (ueNodes);

  // Install UE devices
	NetDeviceContainer ueNetDevices = mmwaveHelper->InstallUeDevice (ueNodes);

	// Install eNB devices
	NetDeviceContainer gnbNetDevices = mmwaveHelper->InstallEnbDevice (gnbNodes); 
 
  // Create the Internet
	std::pair<Ptr<Node>, Ipv4InterfaceContainer> remotePair = Utils::CreateInternet (epcHelper);
	NodeContainer remoteHost;
	remoteHost.Add (remotePair.first); 
	Ipv4InterfaceContainer remoteIface = remotePair.second;

	// Install the Internet on the UEs
	Ipv4InterfaceContainer ueIfaces = Utils::InstallUeInternet (epcHelper, 
                                                              ueNodes, 
                                                              ueNetDevices);

	// Attach UEs to the gNBs
	mmwaveHelper->AttachToClosestEnb (ueNetDevices, gnbNetDevices);

  AsciiTraceHelper asciiTraceHelper;

	// Base ports
	uint16_t appBasePort = 1235; 		// base port for the UDP applications

	// General APP parameters
	unsigned int minStart = 300;		// application min start time [ms]
	unsigned int maxStart = 400; 		// application max start time [ms]
	unsigned int appEnd = simTime - 1;	// application end time [s]	
	std::string appRate = "50Mbps";	    // application data rate	

	// RngStream used to sample the app's starting time
	Ptr<RandomVariableStream> startRngStream = CreateObject<UniformRandomVariable> ();
	startRngStream->SetAttribute ("Min", DoubleValue (minStart));
	startRngStream->SetAttribute ("Max", DoubleValue (maxStart));

  Ptr<OutputStreamWrapper> appOutputStream = asciiTraceHelper.CreateFileStream ("tx-rx-trace.txt");
  AppSetupParams udpAppParams;
	udpAppParams.basePort = appBasePort;
	udpAppParams.startTime = MilliSeconds (startRngStream->GetValue ());
	udpAppParams.endTime = Seconds (appEnd);
	udpAppParams.IPI = MilliSeconds (10);		
	udpAppParams.rate = DataRate (appRate);
	udpAppParams.simTime = Seconds (simTime);

  std::cout << "Num UE :" << ueNodes.GetN() << std::endl;
  std::cout << "UE IP :" << ueIfaces.GetAddress(0) << std::endl;
  std::cout << "UE node id :" << ueNodes.Get (0) << std::endl;
  std::cout << "Remote host IP :" << remoteIface.GetAddress(0) << std::endl;


  for (unsigned int i = 0; i < ueNodes.GetN (); i++)
    {
		  Utils::InstallUdpApplication (appOutputStream, remoteHost.Get (0), ueIfaces.GetAddress (i), udpAppParams);
		  Utils::InstallUdpPacketSink (appOutputStream, ueIfaces.GetAddress (i), ueNodes.Get (i), udpAppParams);
		  udpAppParams.startTime = MilliSeconds (startRngStream->GetValue ());
		  udpAppParams.basePort++;
    }
  
  mmwaveHelper->EnableTraces ();
  Simulator::Stop (Seconds (simTime));

  // capture default configs
  Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
  Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
  Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));
  ConfigStore outputConfig2;
  outputConfig2.ConfigureDefaults ();
  outputConfig2.ConfigureAttributes ();

  Simulator::Run ();
  Simulator::Destroy ();
  return 0;

}
