/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

//On défini les différentes bibliothèque nécéssaire pour le bon déroulement du programme
#include <iomanip>
#include <iostream>
#include <time.h>

#include "ns3/network-module.h"
#include "ns3/rosvehsync-helper.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/netanim-module.h"

using namespace std;

using namespace ns3;

// Définition du nom du programme pour les LOGS
NS_LOG_COMPONENT_DEFINE("ROS2NS3Main");

void
initControlNode (std::string ip_ROS)
{

  Ptr<Node> controlNode = CreateObject<Node> ();

  // Add a default internet stack to the node (ARP, IPv4, ICMP, UDP and TCP).
  InternetStackHelper internetStackHelper;
  internetStackHelper.Install (controlNode);

  //Noeud de contrôle <=> tap0
  std::string tapName ("tap0");

  uint16_t port = 12000;// Attention au port qui doit être le même dans ROS

  //Mise en place de l'adressage IP du noeud de contrôle
  std::string network ("10.0.0.0");
  std::string mask ("255.255.255.0");
  bool modePi = false; // Pi mode = Protocol Information

  //Conversion des chaines de caractères en Ipv4
  Ipv4Address ipv4_address_ROS (ip_ROS.c_str ());
  Ipv4Address controlNetwork (network.c_str ());
  Ipv4Mask controlMask (mask.c_str ());

  //Une fois les adresses IP obtenu on les attribut au noeud de controle ( qui gère le tap device)
  Ipv4AddressHelper addressHelper;
  addressHelper.SetBase (controlNetwork, controlMask); // 10.0.0.0 et 255.255.255.0
  Ipv4Address controlNodeIp = addressHelper.NewAddress (); // 10.0.0.1

  //Mise en place du tap device
  TapFdNetDeviceHelper tapHelper; // Fd = File Descriptor
  tapHelper.SetDeviceName (tapName); //tap0
  tapHelper.SetModePi (modePi); //false (Pi = protocol information)
  tapHelper.SetTapIpv4Address (controlNodeIp);//ip du noeud de controle = 10.0.0.1
  tapHelper.SetTapIpv4Mask (controlMask);//255.255.255.0

  NetDeviceContainer netDeviceContainer = tapHelper.Install (controlNode);
  Ptr<NetDevice> netDevice = netDeviceContainer.Get (0);

  // Interface
  Ptr<Ipv4> ipv4 = controlNode->GetObject<Ipv4> ();
  uint32_t interface = ipv4->AddInterface (netDevice);
  Ipv4Address controlTapIp = addressHelper.NewAddress (); //10.0.0.2
  Ipv4InterfaceAddress controlInterfaceAddress = Ipv4InterfaceAddress (controlTapIp, controlMask);
  ipv4->AddAddress (interface, controlInterfaceAddress);
  ipv4->SetMetric (interface, 1);
  ipv4->SetUp (interface); // Activation de l'interface

  //Mise en place du routage
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (ipv4);
  staticRouting->SetDefaultRoute (controlNodeIp, interface);

  AddressValue remoteAddress(InetSocketAddress (ipv4_address_ROS, port));
  AddressValue sinkLocalAddress(InetSocketAddress (controlTapIp, port));

  MobilityHelper mobilityControlNode;
  mobilityControlNode.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobilityControlNode.Install(controlNode);

  /*** Mise en place de ROSVehSyncHelper ***/
  ROSVehSyncHelper ROSVehSyncHelper;
  ROSVehSyncHelper.SetAttribute ("Interval", TimeValue (Seconds (1)));    // packet sending default interval  On echelone la récupération de donnée
  ROSVehSyncHelper.SetAttribute ("ROS_IP_AddressValue", remoteAddress); //adresse de destination
  ROSVehSyncHelper.SetAttribute ("TAP_IP_AddressValue", sinkLocalAddress);     //adresse local
  ROSVehSyncHelper.SetAttribute ("Protocol", StringValue("ns3::UdpSocketFactory"));    //protocol de communication UDPSocket
  ROSVehSyncHelper.SetAttribute ("Port", UintegerValue(port));    //  port de Rtmaps par défaut
  ApplicationContainer ROSAppContainer = ROSVehSyncHelper.Install (controlNode); //On stock notre application dans un conteneur application et on installe sur notre neoud Contrôle Node

}

void
initVehicules (int nb_vehicule, std::string ip_ROS)
{

  // Create a container for the nodes
  NodeContainer nodes;
  nodes.Create(nb_vehicule); // Predefine nodes

  // Set up mobility
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(nodes);

  // Set up internet stack
  InternetStackHelper internetStackHelper;
  internetStackHelper.Install(nodes);

  for(int i=1; i<=nb_vehicule; i++) {

    Ptr<Node> nodei = NodeContainer::GetGlobal().Get(i);

    string nodeNumberString = to_string(i);

    NS_LOG_UNCOND("Creating node "+nodeNumberString);

    //IP:
    string tap_neti_string = "10.0."+nodeNumberString+".0";

    //Nom du tap device
    string nom_tap = "tap"+nodeNumberString;

    bool modePi = false;

    std::string tap_mask_string ("255.255.255.0"); //On lui assigne également un masque

    //On convertit les adresses/Masque de sous réseau en chaîne de caractère
    Ipv4Address tap_neti (tap_neti_string.c_str());
    Ipv4Mask tap_maski (tap_mask_string.c_str());

    //On assigne les bonnes adresses 10.0.i.1 -> IP noeud véhicule i
    Ipv4AddressHelper addressVehiclesHelper;
    addressVehiclesHelper.SetBase (tap_neti, tap_maski);
    Ipv4Address IP_node_veh = addressVehiclesHelper.NewAddress (); // Will give 10.0.i.1

    //IP noeud tap device 10.0.i.2
    Ipv4Address IP_tap_i = addressVehiclesHelper.NewAddress (); // Will give 10.0.i.2

    // Mise en place FdNetDevice device
    TapFdNetDeviceHelper helperi;
    helperi.SetDeviceName (nom_tap);//on lui attribut le nom tapi
    helperi.SetModePi (modePi);//On sélectionne le modePi ------------------
    helperi.SetTapIpv4Address (IP_tap_i);//doit contenir le noeud de control
    helperi.SetTapIpv4Mask (tap_maski);//et un masque de sous réseau.

    NetDeviceContainer netDeviceContaineri = helperi.Install (nodei);//On créer un device container et on lui attribut notre tap device
    Ptr<NetDevice> netDevicei = netDeviceContaineri.Get (0);//Pas utile vu qu'on a un seul noeud

    Ptr<Ipv4> ipv4_i = nodei->GetObject<Ipv4> ();
    uint32_t interfacei = ipv4_i->AddInterface (netDevicei);
    Ipv4InterfaceAddress addressi = Ipv4InterfaceAddress (IP_node_veh, tap_maski);
    ipv4_i->AddAddress (interfacei, addressi);
    ipv4_i->SetMetric (interfacei, 1);
    ipv4_i->SetUp (interfacei);

    // Routing
    Ipv4StaticRoutingHelper ipv4RoutingHelperi;
    Ptr<Ipv4StaticRouting> staticRoutingi = ipv4RoutingHelperi.GetStaticRouting (ipv4_i);
    staticRoutingi->SetDefaultRoute (IP_tap_i, interfacei);

  }

  std::string phyMode ("OfdmRate6MbpsBW10MHz");// A voir --------------

  YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default();
  Ptr<YansWifiChannel> sharedChannel = waveChannel.Create();

  /* Documentantion for YansWifiChannelHelper::Default()
  * Create a channel helper in a default working state. By default, we create
  * a channel model with a propagation delay equal to a constant, the speed of light,
  * and a propagation loss based on a log distance model with a reference loss of 46.6777 dB
  * at reference distance of 1m.
  */

  YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel(sharedChannel);
  wifiPhy.Set("TxPowerStart", DoubleValue(20.0));  // in dBm
  wifiPhy.Set("TxPowerEnd", DoubleValue(20.0));  // in dBm
  wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11);
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

  //wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging

  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                    "DataMode",StringValue (phyMode),
                                    "ControlMode",StringValue (phyMode));

  NetDeviceContainer devices_wifi = wifi80211p.Install(wifiPhy, wifi80211pMac, nodes);
  wifiPhy.EnablePcap("wave-simple-80211p", devices_wifi);
  Ipv4AddressHelper ipv4;
  ipv4.SetBase("11.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer ipv4_80211p = ipv4.Assign(devices_wifi);

  for(int i=1; i<=nb_vehicule; i++) {

    Ptr<Node> nodei = NodeContainer::GetGlobal().Get(i);

    string nodeNumberString = to_string(i);

    NS_LOG_UNCOND("Creating node "+nodeNumberString);
    //TAP
    //IP:
    string tap_neti_string = "10.0."+nodeNumberString+".1";
    //Nom du tap device
    string nom_tap = "tap"+nodeNumberString;
    //Port:
    uint16_t portveh = 12000+i;
    //On convertit les adresses/Masque de sous réseau en chaîne de caractère
    Ipv4Address tap_neti (tap_neti_string.c_str());
    Ipv4Address ros_ipv4 (ip_ROS.c_str ());

    AddressValue remoteAddressi(InetSocketAddress (ros_ipv4, portveh));
    AddressValue sinkLocalAddressi(InetSocketAddress (tap_neti, portveh));

    // WAVE
    Ptr<NetDevice> waveDevice = nodei->GetDevice(2);
    // Log the assigned IP address
    Ptr<Ipv4> ipv4_i = nodei->GetObject<Ipv4> ();

    uint16_t portwave = 14000;
    std::ostringstream ipWave;
	  ipWave << "11.0.0." << i;
    Ipv4Address wave_neti = Ipv4Address(ipWave.str().c_str());
    AddressValue waveLocalAddressi(InetSocketAddress (wave_neti, portwave));

    // Check installation
    Ptr<Ipv4> ipv4check = nodei->GetObject<Ipv4>();
    for (uint32_t j = 0; j < ipv4check->GetNInterfaces(); ++j)
    {
      NS_LOG_INFO("Node " << i << " Interface " << j  << " IP: " << ipv4check->GetAddress(j, 0).GetLocal());
    }

    for (uint32_t j = 0; j < nodei->GetNDevices(); ++j)
    {
      NS_LOG_INFO("Device " << j << ": " << nodei->GetDevice(j)->GetInstanceTypeId().GetName());
    }

    //Mettre en place les paramètres de ROS
    ROSVehiculeHelper rosVehiculeHelper;
    rosVehiculeHelper.SetAttribute ("RemoteROS",remoteAddressi);
    rosVehiculeHelper.SetAttribute ("LocalTap", sinkLocalAddressi);
    rosVehiculeHelper.SetAttribute ("LocalWave", waveLocalAddressi);
    rosVehiculeHelper.SetAttribute ("VehicleNumber", IntegerValue(i));
    rosVehiculeHelper.SetAttribute ("PortTap", UintegerValue(portveh));
    rosVehiculeHelper.SetAttribute ("PortWave", UintegerValue(portwave));
    //Ajout adresse destination dans le node 1 ex :  tap 1 -> wave

    ApplicationContainer ROSVehSyncApps1 = rosVehiculeHelper.Install (nodei);
  }
}

int
main (int argc, char *argv[])
{

  std::string phyMode ("OfdmRate6MbpsBW10MHz");

  /*** Options ***/

  //Mise en place de l'analyseur de ligne de commande
  CommandLine cmd (__FILE__);
  cmd.Parse (argc, argv);//mise en place de l'analyse
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  // Activation des LOGS
  LogComponentEnable ("ROS2NS3Main", LOG_LEVEL_INFO);
  LogComponentEnable ("ROSVehSync", LOG_LEVEL_INFO);
  LogComponentEnable ("ROSVehicule", LOG_LEVEL_INFO);
  LogComponentEnable ("ROSHeader", LOG_LEVEL_INFO);


  NS_LOG_INFO("Starting program");

  // Simulateur en temps réel
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  // Activation des sommes de contrôle
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  /*** Paramètres de la simulation ***/
  NS_LOG_INFO("Setting up parameters");
  Time simulationTime (Seconds(60));

  /*** Mise en place du noeud de contrôle ***/
  // Peu importe l'adresse ROS peut voir tout ce qui se passe sur le TAP
  std::string ip_ROS ("10.255.255.254");

  NS_LOG_INFO("Initialisation du noeud de contrôle");
  initControlNode(ip_ROS);

  // NetAnim does not support creating nodes at run-time
  // We have to create nodes and then update them according to ROS
  const uint32_t maxNodes = 5;

  NS_LOG_INFO("Initialisation des noeuds vehicules");
  initVehicules(maxNodes, ip_ROS);

  auto now = std::time(nullptr);
  std::tm localTime = *std::localtime(&now);

  std::ostringstream oss;
  oss << std::put_time(&localTime, "%Y%m%d_%H%M");

  std::string animFileName = "Animation_" + oss.str() + ".xml";

  PacketMetadata::Enable();

  AnimationInterface anim(animFileName);
  anim.EnablePacketMetadata(true);

  simInfo.filename = animFileName;
  simInfo.nodeCount = maxNodes;
  simInfo.duration = simulationTime;

  NS_LOG_INFO("Starting simulation of " << simulationTime.GetSeconds() << " seconds");
  Simulator::Stop (simulationTime);
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
