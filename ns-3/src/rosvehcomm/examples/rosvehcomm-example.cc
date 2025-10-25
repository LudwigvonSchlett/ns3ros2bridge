/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

//On défini les différentes bibliothèque nécéssaire pour le bon déroulement du programme
#include "ns3/rosvehcomm-helper.h"
#include "ns3/netanim-module.h"



using namespace ns3;
NS_LOG_COMPONENT_DEFINE("RosvehcommExample");//On défini les LOG
int
main (int argc, char *argv[])
{

  /*** 0. Logging Options ***/
//Mise en place de l'analyseur de ligne de commande 
  CommandLine cmd;
  cmd.Parse (argc, argv);//mise en place de l'analyse
  LogComponentEnable ("RosvehcommExample", LOG_LEVEL_INFO);//On autorise/active les LOG_INFO pour connaître toute les infos de rosvehcommexample et RTmaps Vehsync 
  LogComponentEnable ("ROSVehSync", LOG_LEVEL_INFO);
  NS_LOG_INFO("Start.");

 // FOR ARTEMIPS
  // Since we are using a real piece of hardware we need to use the realtime
  // simulator.
  

  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  //
  // Since we are going to be talking to real-world machines, we need to enable
  // calculation of checksums in our protocols.
  //
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));
                  // FOR ARTEMIPS



  /*** I. Create node pool for all our node(s). ***/
//Dans cette partie nous allons mettre en place le timer plus précisémment la durée de la simulation.
  ns3::Time simulationTime (ns3::Seconds(500));
//On définit le nombre de véhicule -> paramètre à changer si notre cas se rapporte à 3 véhicules.
  int vehNb = 2;
//On affiche que les noeuds vont bien être créer avec le log_info qui peu être comparé à un cout.
  NS_LOG_INFO("Create Pool.");
  //le node container est le conteneur de noeud de ns3 dans notre cas à nous nous avons 2 noeuds 
  NodeContainer vehNodes; //vehNodes est le nom du conteneur de noeud
  vehNodes.Create (vehNb);//ici on lui attribut l( au conteneur ) les deux noeuds créer au préalable.
//On créer le noeud "controlNode"
  Ptr<Node> controlNode = CreateObject<Node> ();


  /*** II. Set up the Control Node ***/

  NS_LOG_INFO("Set up Tap Device for communication with RTMaps.");//cout -> mise en place du TAP device avec Rtmaps et de ces composants 

  uint16_t port_rtmaps = 11111;//on defini le port RTmaps 11111 qui est le port de RTmaps par défauts 
  std::string deviceName ("tap0");//mise en place du nom du tap device
  std::string remote ("192.168.1.45");//on lui assigne une adresse IP de la destination 
  std::string network ("10.0.0.0");//Adresse IP du TapDevice.
  std::string mask ("255.255.255.0");//Un masque de sous réseau
  std::string pi ("no");
//On convertit les chaînes de caractère en adresse IP, masque de sous réseau et l'adresse de destination 
  Ipv4Address remoteIp (remote.c_str ());
  Ipv4Address tapNetwork (network.c_str ());
  Ipv4Mask tapMask (mask.c_str ());

  bool modePi = ( pi == "yes" ? true : false);//
  
  // Create an fd device, set a MAC address and point the device to the
  // Linux device name.  The device needs a transmit queueing discipline so
  // create a droptail queue and give it to the device.  Finally, "install"
  // the device into the node.
  

  //Une fois les adresses IP obtenu on les attribut au noeud de controle ( qui gère le tap device)
  Ipv4AddressHelper addresses;
  addresses.SetBase (tapNetwork, tapMask);
  Ipv4Address controlNodeIp = addresses.NewAddress (); // Will give 10.0.0.1

  NS_LOG_INFO ("IP of control node is " << controlNodeIp); //on affiche les infos du node 
//Mise en place du tap device 
  TapFdNetDeviceHelper helper;
  helper.SetDeviceName (deviceName);//on lui attribut le nom tap0
  helper.SetModePi (modePi);//On sélectionne le modePi ------------------------------A voir car notion encore flou-----------------------------
  helper.SetTapIpv4Address (controlNodeIp);//doit contenir le noeud de control
  helper.SetTapIpv4Mask (tapMask);//et un masque de sous réseau.



//netdevice outil -> installer application 
  // Install the tap on first (and unique) control node
  NetDeviceContainer devices = helper.Install (controlNode);//On créer un device container et on lui attribut notre tap device
  Ptr<NetDevice> device = devices.Get (0);

  // Add a default internet stack to the node (ARP, IPv4, ICMP, UDP and TCP).
  InternetStackHelper InternetStackHelper;
  InternetStackHelper.Install (controlNode);
  //

  // Interface
  Ptr<Ipv4> ipv4 = controlNode->GetObject<Ipv4> (); // Second occurence of Ipv4 object
  uint32_t interface = ipv4->AddInterface (device);
  Ipv4Address devIp = addresses.NewAddress (); // Will give 10.0.0.2
  Ipv4InterfaceAddress address2 = Ipv4InterfaceAddress (devIp, tapMask);
  ipv4->AddAddress (interface, address2);
  ipv4->SetMetric (interface, 1);
  ipv4->SetUp (interface);

  NS_LOG_INFO ("IP of interface is " << devIp);

  //
  // Add a route to the ns-3 device so it can reach the outside world though the
  // TAP.

  //mettre en place le routage 
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> staticRouting = ipv4RoutingHelper.GetStaticRouting (ipv4);
  staticRouting->SetDefaultRoute (controlNodeIp, interface);

  AddressValue remoteAddress(InetSocketAddress (remoteIp, port_rtmaps));
  AddressValue sinkLocalAddress(InetSocketAddress (devIp, port_rtmaps));

  NS_LOG_INFO("Set Up ROSVehSync Application.");

//Mettre en place les paramètres de Rtmaps 
  ROSVehSyncHelper ROSVehSyncHelper; 
  ROSVehSyncHelper.SetAttribute ("Interval", TimeValue (Seconds (1)));    // packet sending default interval  On echelone la récupération de donnée
  ROSVehSyncHelper.SetAttribute ("RemoteRTMaps", remoteAddress); //adresse de destination 
  ROSVehSyncHelper.SetAttribute ("Local", sinkLocalAddress);     //adresse local 
  ROSVehSyncHelper.SetAttribute ("Protocol", StringValue("ns3::UdpSocketFactory"));    //protocol de communication UDPSocket
  ROSVehSyncHelper.SetAttribute ("PortRTMaps", UintegerValue(port_rtmaps));    //  port de Rtmaps par défaut
  ROSVehSyncHelper.SetAttribute ("VehiclesNumber", UintegerValue(vehNb));    //

  ApplicationContainer ROSVehSyncApps = ROSVehSyncHelper.Install (controlNode);


  /*** III. Set up the Vehicle Nodes ***/

  // Mobility 
  
  NS_LOG_INFO("Set up mobility for all nodes");//Mise en place de la mobilité 

  MobilityHelper mob;
  mob.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
  mob.Install(vehNodes);

  /* Here : Set up an IP and a tap interface for each vehicle node */



//Début du changement du prog ----------------------------------------

//AnimationInterface anim ("animation_veh.xml");


/*
double x = mob->GetPosition().x;
double y = mob->GetPosition().y;
double z = mob->GetPosition().z;

double x1 = mob->GetPosition().x;
double y2 = mob->GetPosition().y;
double z2 = mob->GetPosition().z;*/

//anim.SetConstantPosition(vehNodes.Get(0),1.0,2.0,1.0);
//anim.SetConstantPosition(vehNodes.Get(1),2.0,5.0,1.0);


//Fin du changement du programme 


  /*** IV. Launch applications ***/

  NS_LOG_INFO("Start ROSVehSync Application.");
  ROSVehSyncApps.Start (Seconds (1.0));
  ROSVehSyncApps.Stop (simulationTime);
  

   /*** IV. Set up and start simulation ***/

  Simulator::Stop (simulationTime);

  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}

