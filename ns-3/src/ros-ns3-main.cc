/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */



/*
Dans ce code vous trouverez plusieurs choses. Premièrement, il y a la partie Plan de controle où l'on récupère dans NS3 la position des noeuds, leurs vitesses afin de pouvoir placer dans NS3 
les différents neouds dans NetAnim. 

Nous avons également la mise en place du plan de données qui est réalisé par notre algorithme.


Voici Étape par étape ce que réalise ce code:


-Premièrement Nous avons la fonction ReceivePacket( ): cette fonction nous permet de réceptionner des paquets qui sont envoyés et aussi de les afficher par la suite 
-Deuxièmement nous avons la fonction GenerateTraffic celle ci permet d'envoyer des paquets d'un noeud à un autre et également d'afficher toutes les données de ces paquets 
Ces deux fonctions sont déclarées avant le main() car elle sont utilisées par la suite.

Dans le main, nous avons la disposition suivante:

Tout d'abord nous avons la création du container du controle node:
Nous lui assignonsn une adresse IP ainsi qu'un masque de sous réseaux (FdNetDevice)

Une fois que cela est fait, nous devons mettre en place le Tap device qui gère la réception des données provenant de RTMaps.



Et nous avons aussi le plan de données CAD tout les noeuds dans NS3 qui réalise le flux de données.
*/


//On défini les différentes bibliothèque nécéssaire pour le bon déroulement du programme
#include "ns3/rosvehcomm-helper.h"
#include "ns3/netanim-module.h"
#include <iostream>
#include <vector>
#include <typeinfo>
#include <time.h>
#include <iomanip>

#include "ns3/yans-wifi-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-interface-container.h"
#include <iostream>

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
NS_LOG_COMPONENT_DEFINE("ROSNS3Example");

void ReceivePacket (Ptr<Socket> socket)
{
  double time_recu = Simulator::Now().GetSeconds();

  while (socket->Recv ())
  {
    Ptr<Packet> paquet_rcv = socket->Recv();
    Ptr<Node> Node_rcv = socket->GetNode();
    Ptr<Ipv4> ipv4_test_socket = Node_rcv->GetObject<Ipv4> ();
    Ipv4InterfaceAddress iaddr_socket = ipv4_test_socket->GetAddress(2,0);
    Ipv4Address ipAddr1 = iaddr_socket.GetLocal ();

    //NS_LOG_UNCOND ("Received one packet!---------------------------------------------------");
    NS_LOG_UNCOND ("Paquet reçu par le noeud  : " << Node_rcv->GetId() <<" L'adresse IP est : " << ipAddr1<<" . Le message est reçu à :" << time_recu <<" secondes" );
    //NS_LOG_UNCOND("Received packet :"<< paquet_rcv->GetUid());
  }
}

//RECV from paquet -> à tester dans ma méthode.  

static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize,
                             uint32_t pktCount, Time pktInterval,Ptr<Node> node)
{
  double time_send = Simulator::Now().GetSeconds();

  if (pktCount > 0)
  {
    Ptr<Ipv4> ipv4_test = node->GetObject<Ipv4> ();
    Ipv4InterfaceAddress iaddr = ipv4_test->GetAddress(2,0);
    Ipv4Address ipAddr = iaddr.GetLocal ();

    Ptr<Packet> paquet = Create<Packet> (pktSize);
    socket->Send (paquet);
    Simulator::Schedule (pktInterval, &GenerateTraffic,
                           socket, pktSize,pktCount - 1, pktInterval,node);
    NS_LOG_UNCOND(" NODE :" << node->GetId() <<" , Adresse IP :" << ipAddr<<" , envoie le paquet numéros : " <<paquet->GetUid() << " , de taille : " <<paquet->GetSize() << " le paquet est envoyé à: "<< time_send << " secondes");
  }
  else
  {
    socket->Close ();
    NS_LOG_INFO("J'envoie pas de paquet ! attention il faut corrigé cette erreur ");
  }
}

void WhenConditionMet(std::string message)
{
  NS_LOG_UNCOND(message);
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
  LogComponentEnable ("ROSNS3Example", LOG_LEVEL_INFO);
  LogComponentEnable ("ROSVehSync", LOG_LEVEL_INFO);


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

  ns3::AnimationInterface anim(animFileName);

  netAnimFile.filename = animFileName;

  NS_LOG_INFO("Starting simulation of " << simulationTime.GetSeconds() << " seconds");
  Simulator::Stop (simulationTime);
  //Simulator::Stop (ns3::Seconds (60));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}
