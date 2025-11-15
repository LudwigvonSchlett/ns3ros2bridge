/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

//On défini les différentes bibliothèque nécéssaire pour le bon déroulement du programme
#include <iostream>
#include <vector>
#include <typeinfo>
#include <time.h>
#include <iomanip>

#include "ns3/rosvehsync-helper.h"

#include "ns3/netanim-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/network-module.h"
#include "ns3/ipv4.h"
#include "ns3/timer.h"

using namespace std;

using namespace ns3;

// Définition du nom du programme pour les LOGS
NS_LOG_COMPONENT_DEFINE("ROS2NS3Main");

int
main (int argc, char *argv[])
{
  std::string phyMode ("OfdmRate6MbpsBW10MHz");// A voir --------------

  /*** Options ***/

  //Mise en place de l'analyseur de ligne de commande
  CommandLine cmd (__FILE__);
  cmd.Parse (argc, argv);//mise en place de l'analyse
  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  // Activation des LOGS
  LogComponentEnable ("ROS2NS3Main", LOG_LEVEL_INFO);
  LogComponentEnable ("ROSVehSync", LOG_LEVEL_INFO);
  LogComponentEnable ("ROSVehicule", LOG_LEVEL_INFO);


  NS_LOG_INFO("Starting program");

  // Simulateur en temps réel
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  // Activation des sommes de contrôle
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  /*** Paramètres de la simulation ***/
  NS_LOG_INFO("Setting up parameters");
  ns3::Time simulationTime (ns3::Seconds(180));

  /*** Mise en place du noeud de contrôle ***/
  NS_LOG_INFO("Initialisation du noeud de contrôle");
  //Noeud de contrôle <=> tap0
  std::string tapName ("tap0");

  // Peu importe l'adresse ROS peut voir tout ce qui se passe sur le TAP
  std::string ip_ROS ("10.255.255.254");
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

  Ptr<Node> controlNode = CreateObject<Node> ();
  NetDeviceContainer netDeviceContainer = tapHelper.Install (controlNode);
  Ptr<NetDevice> netDevice = netDeviceContainer.Get (0);

  // Add a default internet stack to the node (ARP, IPv4, ICMP, UDP and TCP).
  InternetStackHelper internetStackHelper;
  internetStackHelper.Install (controlNode);

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
  
  std::string protocol_name("ns3::UdpSocketFactory");

  //----------------------WAVE APPLICATION----------------------------------
  //Mise en place de l'interface wifi 80211p dans notre noeud véhicule

  // On crée un objet de type WifiPhyHelper (couche physique et canal)
  YansWifiPhyHelper wifiPhy;
  // On crée un canal avec un modele de propag par defaut (Methode YansWifiChannelHelper dans src/wifi/helper/yans-wifi-helper)
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  // On positionne le canal dans le wifiPhy
  wifiPhy.SetChannel (wifiChannel.Create ());

  //Dénifition des différents paramètre du wifi 80211p
  NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

  //La ligne qui nous permet de désactiver tout Les LOGS de WAVE :
  //wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging

  //Station Manager on l'implémente de cette façon
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                        "DataMode",StringValue (phyMode),
                                        "ControlMode",StringValue (phyMode));

  // NetAnim does not support creating nodes at run-time
  // We have to create nodes and then update them according to ROS
  const uint32_t maxNodes = 254;

  // Create a container for the nodes
  NodeContainer nodes;
  nodes.Create(maxNodes); // Predefine nodes

  // Set up mobility
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mobility.Install(nodes);

  auto now = std::time(nullptr);
  std::tm localTime = *std::localtime(&now);

  std::ostringstream oss;
  oss << std::put_time(&localTime, "%Y%m%d_%H%M");

  std::string animFileName = "Animation_" + oss.str() + ".xml";

  ns3::AnimationInterface anim(animFileName);

  netAnimFile.filename = animFileName;

  NS_LOG_INFO("Starting simulation of " << simulationTime.GetSeconds() << " seconds");
  Simulator::Stop (simulationTime);
  //Simulator::Stop (ns3::Seconds (60));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
