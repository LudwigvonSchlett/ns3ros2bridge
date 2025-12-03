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
initControlNode ()
{
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

}

void
initVehicules ()
{
  //  NodeContainer Container_veh;
  //  Container_veh.Create(nb_vehicule);

  //Je déclare les variables ci-dessous pour les réutiliser dans ma boulce For par la suite
  AddressValue remoteAddress1(InetSocketAddress());
  AddressValue sinkLocalAddress1(InetSocketAddress());

  AddressValue remoteAddress2(InetSocketAddress());
  AddressValue sinkLocalAddress2(InetSocketAddress());

  //Cette boucle FOR permet la mise en place des tap/FdNetdevice de nos noeuds véhicules
//  for(int i=1;i<=nb_vehicule;i++)
//  {
//    string nodeNumberString = to_string(i);
//
//    NS_LOG_UNCOND("Création du noeud "+nodeNumberString);
//
//    //IP:
//    string tap_neti_string = "10.0."+nodeNumberString+".0";
//
//    //Nom du tap device
//    string test_tap ="tap";
//    string nom_tap = test_tap+nodeNumberString;
//
//    //Port:
//    uint16_t port = 12000+i;
//    bool modePi = false;
//
//    //IP WAVE
//    string Adresse_Ip_Wave = "11.0.0."+nodeNumberString;
//    string wave_mask = "255.255.255.255";
//
//    //Récupération du noeud dans le ContainerNode
//    Container_veh.Get(i-1);
//    Ipv4Address remoteIpi (ip_ROS.c_str ());//On lui assigne une adresse IP
//    std::string tap_mask_string ("255.255.255.0"); //On lui assigne également un masque
//
//    //On convertit les adresses/Masque de sous réseau en chaîne de caractère
//    Ipv4Address tap_neti (tap_neti_string.c_str());
//    Ipv4Mask tap_maski (tap_mask_string.c_str());
//
//    //On assigne les bonnes adresses 10.0.i.1 -> IP noeud véhicule i
//    Ipv4AddressHelper addressVehiclesHelper;
//    addressVehiclesHelper.SetBase (tap_neti, tap_maski);
//    Ipv4Address IP_node_veh = addressVehiclesHelper.NewAddress (); // Will give 10.0.i.1
//
//    //IP noeud tap device 10.0.i.2
//    Ipv4Address IP_tap_i = addressVehiclesHelper.NewAddress (); // Will give 10.0.i.2
//
//    //Mise en place FdNetDevice device
//    TapFdNetDeviceHelper helperi;
//    helperi.SetDeviceName (nom_tap);//on lui attribut le nom tapi
//    helperi.SetModePi (modePi);//On sélectionne le modePi ------------------
//    helperi.SetTapIpv4Address (IP_tap_i);//doit contenir le noeud de control
//    helperi.SetTapIpv4Mask (tap_maski);//et un masque de sous réseau.
//
//
//    NetDeviceContainer netDeviceContaineri = helperi.Install (Container_veh.Get(i-1));//On créer un device container et on lui attribut notre tap device
//    Ptr<NetDevice> netDevicei = netDeviceContaineri.Get (0);//Pas utile vu qu'on a un seul noeud
//
//    internetStackHelper.Install (Container_veh.Get(i-1));
//
//    Ptr<Ipv4> ipv4_i = Container_veh.Get(i-1)->GetObject<Ipv4> (); // Second occurence of Ipv4 object
//    uint32_t interfacei = ipv4_i->AddInterface (netDevicei);
//    Ipv4InterfaceAddress addressi = Ipv4InterfaceAddress (IP_node_veh, tap_maski);
//    ipv4_i->AddAddress (interfacei, addressi);
//    ipv4_i->SetMetric (interfacei, 1);
//    ipv4_i->SetUp (interfacei);
//
//    //Mise en place du routage
//    Ipv4StaticRoutingHelper ipv4RoutingHelperi;
//    Ptr<Ipv4StaticRouting> staticRoutingi = ipv4RoutingHelperi.GetStaticRouting (ipv4_i);
//    staticRoutingi->SetDefaultRoute (IP_tap_i, interfacei);
//
//    AddressValue remoteAddressi(InetSocketAddress (remoteIpi, port));
//    AddressValue sinkLocalAddressi(InetSocketAddress (tap_neti, port));
//
//    //Mettre en place les paramètres de ROS
//    ROSVehicule1Helper rosVehicule1Helper;
//    rosVehicule1Helper.SetAttribute ("RemoteRTMaps",remoteAddressi);
//    rosVehicule1Helper.SetAttribute ("Local", sinkLocalAddressi );
//    rosVehicule1Helper.SetAttribute ("VehicleNumber", IntegerValue(i) );
//    //Ajout adresse destination dans le node 1 ex :  tap 1 -> wave
//
//    ApplicationContainer ROSVehSyncApps1 = rosVehicule1Helper.Install (Container_veh.Get(i-1));
//
//    ROSVehSyncApps1.Start(Seconds(1.0));
//    ROSVehSyncApps1.Stop(simulationTime);
//  }
  
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

  //On applique les paramètre à notre Node container
//  NetDeviceContainer devices_wifi = wifi80211p.Install (wifiPhy, wifi80211pMac, Container_veh);
//  //On autorise les PCAP -> acitver la sortie réseau
//  wifiPhy.EnablePcap ("vehcomm-example_flo", Container_veh);//A developper.
//
//  NS_LOG_INFO("Assignation d'une adresse IP au wave");
//  Ipv4AddressHelper ipv4_wifi;
//  ipv4_wifi.SetBase("11.0.0.0","255.255.255.0","0.0.0.1");
//  NS_LOG_INFO("TEST ASSIGNATION D'ADRESSE IP ");
//  Ipv4InterfaceContainer i_wifi = ipv4_wifi.Assign(devices_wifi);


  //On affiche le nombre de véhicule que contient le container.
  //NS_LOG_UNCOND("Le nombre de véhicule que contient le container est : "<<Container_veh.GetN());

  //Section concernant la vérification des adresses IP des différents noeuds GetAdresse(1,0) pour connaître l'interface TAP et GetAdresse(2,0) pour l'interface wave

  //----------------------VERIF ADRESSE WAVE DU GLOBAL ----------------------------
  //On vérifie toute les adresses qui sont assigné
//  NS_LOG_UNCOND("-------------------TEST DU GLOBAL-------------------------");
//  NodeContainer Globalnode;
//
//  Globalnode = NodeContainer::GetGlobal();
//  NS_LOG_UNCOND("Le noeud global contient : "<<Globalnode.GetN()<<" noeuds");
//
//  NS_LOG_UNCOND("Adresse du tap device 1 du premier objet:");
//  Ptr<Node> TEST_global = Globalnode.Get(0);
//  Ptr<Ipv4> ipv4_test_global = TEST_global->GetObject<Ipv4> ();
//  Ipv4InterfaceAddress iaddr1 = ipv4_test_global->GetAddress(2,0);
//  Ipv4Address ipAddr_global1 = iaddr1.GetLocal ();
//  NS_LOG_UNCOND("L'adresse IP du noeud qui envoie est : "<<ipAddr_global1); //On affiche les adresses

//  NS_LOG_UNCOND("Adresse 2 du premier objet du container global");
//  Ptr<Node> TEST_global1 = Globalnode.Get(1);
//  Ptr<Ipv4> ipv4_test_global1 = TEST_global1->GetObject<Ipv4> ();
//  Ipv4InterfaceAddress iaddr12 = ipv4_test_global1->GetAddress(2,0);
//  Ipv4Address ipAddr_global12 = iaddr12.GetLocal ();
//  NS_LOG_UNCOND("L'adresse IP du noeud qui envoie est : "<<ipAddr_global12); //On affiche les adresses
//  //----------------------FIN VÉRIF ADRESSE DU GLOBAL NODE--------------------
//
//  NS_LOG_UNCOND("Adresse 3 du premier objet du container global");
//  Ptr<Node> TEST_global2 = Globalnode.Get(2);
//  Ptr<Ipv4> ipv4_test_global2 = TEST_global2->GetObject<Ipv4> ();
//  Ipv4InterfaceAddress iaddr13 = ipv4_test_global2->GetAddress(1,0);
//  Ipv4Address ipAddr_global13 = iaddr13.GetLocal ();
//  NS_LOG_UNCOND("L'adresse IP du noeud qui envoie est : "<<ipAddr_global13); //On affiche les adresses
//
//  //Mise en place Mobilité sur les différents noeuds
//  MobilityHelper mobile;
//  mobile.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
//  mobile.Install(Container_veh);
//
//  //On cherche à réaliser le routage entre toutes les interfaces Waves et les interfaces FdNetDevice
//  NS_LOG_UNCOND("-----MISE EN PLACE DE LA TABLE DE ROUTAGE-------");
//  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();
//  NS_LOG_UNCOND("-----FIN DE LA MISE EN PLACE-------");
}

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
  Time simulationTime (Seconds(180));

  /*** Mise en place du noeud de contrôle ***/
  NS_LOG_INFO("Initialisation du noeud de contrôle");
  initControlNode();

  //----------------------WAVE APPLICATION----------------------------------
  //Mise en place de l'interface wifi 80211p

  //Dénifition des différents paramètre du wifi 80211p
  Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

  //La ligne qui nous permet de désactiver tout Les LOGS de WAVE :
  //wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging

  //Station Manager on l'implémente de cette façon
  wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                        "DataMode",StringValue (phyMode),
                                        "ControlMode",StringValue (phyMode));

  // NetAnim does not support creating nodes at run-time
  // We have to create nodes and then update them according to ROS
  const uint32_t maxNodes = 5;

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

  AnimationInterface anim(animFileName);
  anim.EnablePacketMetadata(true);

  simInfo.filename = animFileName;
  simInfo.duration = simulationTime;

  NS_LOG_INFO("Starting simulation of " << simulationTime.GetSeconds() << " seconds");
  Simulator::Stop (simulationTime);
  //Simulator::Stop (ns3::Seconds (60));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
