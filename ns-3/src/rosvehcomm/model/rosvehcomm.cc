/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "rosvehcomm.h"
#include "ns3/rosvehcomm-helper.h"
#include "ns3/log.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-address.h"
#include "ns3/address-utils.h"
#include "ns3/nstime.h"
#include "ns3/inet-socket-address.h"
#include "ns3/socket.h"
#include "ns3/udp-socket.h"
#include "ns3/simulator.h"
#include "ns3/socket-factory.h"
#include "ns3/udp-socket-factory.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include <string>
#include <stdlib.h>
#include <sstream>
#include <iterator>
#include <time.h>
#include "ns3/ipv4.h"



//Début de changement du code:
//Fin du changement

#include "ns3/trace-source-accessor.h"
#include "ns3/netanim-module.h"

using namespace std;

namespace ns3
{
  NS_LOG_COMPONENT_DEFINE("ROSVehSync");
  NS_OBJECT_ENSURE_REGISTERED(ROSVehSync);

  TypeId ROSVehSync::GetTypeId (void)
  {
  static TypeId tid = TypeId ("ns3::ROSVehSync")
    .SetParent<Application> ()
    .SetGroupName("Applications")
    .AddConstructor<ROSVehSync> ()
    .AddAttribute ("ROS_IP_AddressValue", "The AddressValue of ROS",
                   AddressValue (),
                   MakeAddressAccessor (&ROSVehSync::ros_ip),
                   MakeAddressChecker ())
    .AddAttribute ("Port", "Port on which we exchange packets with ROS",
                   UintegerValue (12000),
                   MakeUintegerAccessor (&ROSVehSync::port),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("TAP_IP_AddressValue", "The AddressValue of the tap",
                   AddressValue (),
                   MakeAddressAccessor (&ROSVehSync::tap_ip),
                   MakeAddressChecker ())
    .AddAttribute ("Protocol", "The type of protocol to use to communicate (a subclass of ns3::SocketFactory) ",
                   TypeIdValue (UdpSocketFactory::GetTypeId ()),
                   MakeTypeIdAccessor (&ROSVehSync::controlSocket_tid),
                   MakeTypeIdChecker ())
    .AddAttribute ("Interval",
                   "The time to wait between packets",
                   TimeValue (Seconds (1.0)),
                   MakeTimeAccessor (&ROSVehSync::m_interval),
                   MakeTimeChecker ())
    .AddAttribute ("EnableSeqTsSizeHeader",
                   "Enable optional header tracing of SeqTsSizeHeader",
                   BooleanValue (false),
                   MakeBooleanAccessor (&ROSVehSync::m_enableSeqTsSizeHeader),
                   MakeBooleanChecker ())
    .AddTraceSource ("Tx", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&ROSVehSync::m_txTrace),
                     "ns3::Packet::TracedCallback")
    .AddTraceSource ("Rx",
                     "A packet has been received",
                     MakeTraceSourceAccessor (&ROSVehSync::m_rxTrace),
                     "ns3::Packet::AddressTracedCallback")
    .AddTraceSource ("RxWithAddresses", "A packet has been received",
                     MakeTraceSourceAccessor (&ROSVehSync::m_rxTraceWithAddresses),
                     "ns3::Packet::TwoAddressTracedCallback")
    .AddTraceSource ("RxWithSeqTsSize",
                     "A packet with SeqTsSize header has been received",
                     MakeTraceSourceAccessor (&ROSVehSync::m_rxTraceWithSeqTsSize),
                     "ns3::ROSVehSync::SeqTsSizeCallback")
  	;
    return tid;
  }

  // Constructeur
  ROSVehSync::ROSVehSync ()
  {
    NS_LOG_FUNCTION(this);//pour obtenir des infos lors de la compilation de cette fonction.
    NS_LOG_INFO("Constructeur NUMEROS 0 ");
    m_sendEvent_rtmaps = EventId ();//obtenir l'id de l'evènement.

    // Create Wave PHY using the shared channel
	YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default();
	sharedChannel = waveChannel.Create();
  }

  // Destructeur
  ROSVehSync::~ROSVehSync ()
  {
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("destructeur NUMEROS 1 ");
    controlSocket = 0;
  }

  void ROSVehSync::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("DODISPOSE NUMEROS 2  ");
    m_socket_from_rtmaps = 0;
    m_socketList.clear ();//vider le conteneur de socket
    Application::DoDispose ();
  }

  uint64_t ROSVehSync::GetTotalRx () const
  {
    NS_LOG_FUNCTION (this);
    NS_LOG_INFO("GET TOTAL RX NUMEROS 3  ");
    return m_totalRx;//pour connaître le nombre de byte reçus.
  }

  Ptr<Socket> ROSVehSync::GetListeningSocket (void) const//fonction qui va recevoir les sockets de Rtmaps
  {
    NS_LOG_FUNCTION (this);
    NS_LOG_INFO(" GET LISTENNING SOCKET NUMEROS 4  ");

    return m_socket_from_rtmaps;
  }

  std::list<Ptr<Socket>> ROSVehSync::GetAcceptedSockets (void) const
  {
    NS_LOG_FUNCTION (this);
    NS_LOG_INFO("GET ACCEPTED SOCKET NUMEROS 5");
    return m_socketList;
  }

//Fonction qui lance l'appliction
  void ROSVehSync::StartApplication (void)
  {
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("STARTING ROSVehSync");

    Ptr<Node> node = GetNode();

	if (node->GetObject<MobilityModel>() == nullptr) {
    	Ptr<ConstantVelocityMobilityModel> mobility = CreateObject<ConstantVelocityMobilityModel>();
    	node->AggregateObject(mobility);
	}

    Ptr<ConstantPositionMobilityModel> mobility = node->GetObject<ConstantPositionMobilityModel>();
	mobility->SetPosition(Vector(0.0, 0.0, 0.0));
    controlSocket = 0;

    // Initialize socket
    controlSocket = Socket::CreateSocket(GetNode (), controlSocket_tid);
    controlSocket->SetAllowBroadcast (true);//autoriser la communication broadcast
    controlSocket->Connect(ros_ip);

    controlSocket->Bind(tap_ip);
    controlSocket->Listen ();

    if (addressUtils::IsMulticast (tap_ip))
      {
        Ptr<UdpSocket> udpSocket = DynamicCast<UdpSocket> (controlSocket);
        if (udpSocket)
          {
            // equivalent to setsockopt (MCAST_JOIN_GROUP)
            udpSocket->MulticastJoinGroup (0, tap_ip);
          }
        else
          {
            NS_FATAL_ERROR ("Error: joining multicast on a non-UDP socket");
          }
      }


    controlSocket->SetRecvCallback (MakeCallback (&ROSVehSync::HandleRead, this));
    controlSocket->SetAcceptCallback (
      MakeNullCallback<bool, Ptr<Socket>, const Address &> (),
      MakeCallback (&ROSVehSync::HandleAccept, this));
    controlSocket->SetCloseCallbacks (
      MakeCallback (&ROSVehSync::HandlePeerClose, this),
      MakeCallback (&ROSVehSync::HandlePeerError, this));
  }

  void ROSVehSync::HandleRead1 (Ptr<Socket> socket)
  {
    NS_LOG_UNCOND("HandleRead1");
    NS_LOG_FUNCTION (this << socket);

    Ptr<Packet> packet;
    Address from;
    Address localAddress;
    Address Adress_socket;

    uint32_t totalRx = 0;

    while ((packet = socket->RecvFrom (from))) {
      if (packet->GetSize () == 0) {
        //EOF
        break;
      }
      totalRx += packet->GetSize ();
      if (InetSocketAddress::IsMatchingType (from)) {
        NS_LOG_INFO ("In HandleRead1, at time " << Simulator::Now ().As (Time::S)
          << " packet sink received "
          <<  packet->GetSize () << " bytes from "
          << InetSocketAddress::ConvertFrom(from).GetIpv4 ()
          << " port " << InetSocketAddress::ConvertFrom (from).GetPort ()
          << " total Rx " << totalRx << " bytes");
      }

    /*NS_LOG_INFO ("At time " << Simulator::Now ().As (Time::S)
      << " packet sink received "
      <<  packet->GetSize () << " bytes from "
      << InetSocketAddress::ConvertFrom(from).GetIpv4 ()
      << " port " << InetSocketAddress::ConvertFrom (from).GetPort ()
      << " total Rx " << m_totalRx1 << " bytes");
    */

  //*************Creation de l'adresse à ajouter à la fin du paquet*************************

  //Fixe le nombre de véhicule:
  //string notation = ";";
  //int m_nombre_vehicule1 = 7;// Paramètre à fixer
  //int add_choix = rand() % m_nombre_vehicule1 + 1 ;
  //string adresse_add = notation+"11.0.0."+to_string(add_choix);
  //string adresse_add = notation+"11.0.0.255";
  /*string adresse_add = "11.0.0.255";

  Ptr<Packet> packet_add = Create<Packet> ((uint8_t*) adresse_add.c_str (), adresse_add.length ());
  packet->AddAtEnd(packet_add);
  uint8_t *buffer = new uint8_t[packet->GetSize ()];
  NS_LOG_INFO(packet->GetSize ());
  packet->CopyData(buffer, packet->GetSize ());
  char* contenu = (char *) buffer;
  NS_LOG_INFO("VOICI LE CONTENU :" <<contenu);
  NS_LOG_INFO(packet->GetSize ());

  socket->GetSockName (localAddress);
  m_rxTrace1 (packet, from);
  m_rxTraceWithAddresses1 (packet, from, localAddress);

    if (m_enableSeqTsSizeHeader1) {
      PacketReceived1 (packet, from, localAddress);
    }*/

    //Replace_destination (socket, packet);
    }
  //On passe le paquet avec l'adresse en paramètre de ma fonction.
  }

  void
  ROSVehSync::ScheduleArtemipsTransmit (Time dt)//cette fonction nous permet d'envoyer des informations  vers Rtmaps ( timer, et info du LOG)
  {
    NS_LOG_FUNCTION(this << dt);
    NS_LOG_INFO("SCHEDULE TRANSMIT NUMEROS 7");
    m_sendEvent_rtmaps = Simulator::Schedule (dt, &ROSVehSync::SendArtemips, this);
  }


  void
  ROSVehSync::StopApplication ()//cette fonction nous permet lors de l'arret de la simulation de premièrement fermer le socket créer vers RTmaps, et aussi ceux provenant de Rtmaps
  {
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("STOP APPLICATION NUMÉROS 8");
    if (controlSocket)//si le socket vers Rtmaps n'est pas vide alors on le ferme et on arrete le callback
      {
        controlSocket->Close ();
        controlSocket->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
      }

    Simulator::Cancel (m_sendEvent_rtmaps);

//tant que la liste des socket n'est pas vide alors on
    while(!m_socketList.empty ()) //these are accepted sockets, close them
    {
      Ptr<Socket> acceptedSocket = m_socketList.front ();//pour envoyer la référence du premier element et la stocker dans un variable qui s'appelle acceptedsocket
      m_socketList.pop_front ();//supprime le premier element de la liste m_socketlist
      acceptedSocket->Close ();//ferme le socket
    }
  if (m_socket_from_rtmaps) //si on recoit toujours quelque chose de rtmaps alors on le ferme
    {
      m_socket_from_rtmaps->Close ();
      m_socket_from_rtmaps->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
    }

  }

//cette fonction permet d'afficher ce qu'on envoie
  void ROSVehSync::SendArtemips ()
  {
    NS_LOG_FUNCTION(this << controlSocket);
    NS_LOG_INFO("SEND ARTEMIP NUMEROS 9 ");
    // We don't have anything to send, so we fill the message with imaginary data.

    std::ostringstream msg;
    int identifiant = 0;
    std::string sign = " ";

    // Send a big line with a message with various random integers, separated with spaces

    int route_to_follow = rand() % 20 + 1;//choisis des valeur aléatoirement
    int nb_veh = rand() % 8 + 1;// de même pour le nombre de véhicule
//On affiche dans la console
    NS_LOG_INFO("Now we SEND Message To Artemips [id: 0] & "
        << "[route: " << route_to_follow << "]"
        << " & [nb_veh: " << nb_veh  << "]");

//on envoie un paquet contenant un msg (chaine de caractère ) à rtmaps avec un rate spécifié par le schedule à la ligne 283
    msg << std::to_string(identifiant) << sign << std::to_string(route_to_follow) << sign << std::to_string (nb_veh) << '\n';
    std::ostringstream msg2;
    msg2 << std::to_string(0) << sign << std::to_string(19) << sign << std::to_string (7) << '\n';
    Ptr<Packet> packet = Create<Packet> ((uint8_t*) msg2.str ().c_str (), msg.str ().length ());
    NS_LOG_INFO("Message length: " << msg2.str().length());
    controlSocket->Send (packet);

    ScheduleArtemipsTransmit (m_interval);
  }

void ROSVehSync::CreateVehicle (int i, double x, double y, double z, double xs, double ys, double zs)
{
    Ptr<Node> nodei = NodeContainer::GetGlobal().Get(i);

    string nodeNumberString = to_string(i);

    NS_LOG_UNCOND("Creating node "+nodeNumberString);

    //IP:
    string tap_neti_string = "10.0."+nodeNumberString+".0";

    //Nom du tap device
    string test_tap ="tap";
    string nom_tap = test_tap+nodeNumberString;

    //Port:
    uint16_t portveh = 12000+i;
    bool modePi = false;

    //IP WAVE
    string Adresse_Ip_Wave = "11.0.0."+nodeNumberString;
    string wave_mask = "255.255.255.255";

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

    InternetStackHelper internetStackHelper;
    internetStackHelper.Install (nodei);

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

    Ipv4Address ros_ipv4 = InetSocketAddress::ConvertFrom(ros_ip).GetIpv4();
    AddressValue remoteAddressi(InetSocketAddress (ros_ipv4, portveh));
    AddressValue sinkLocalAddressi(InetSocketAddress (tap_neti, portveh));

    // WAVE
    /* Documentantion for YansWifiChannelHelper::Default()
	* Create a channel helper in a default working state. By default, we create
    * a channel model with a propagation delay equal to a constant, the speed of light,
    * and a propagation loss based on a log distance model with a reference loss of 46.6777 dB
    * at reference distance of 1m.
    */
    YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default();
    YansWavePhyHelper wavePhy = YansWavePhyHelper::Default();
    wavePhy.SetChannel(sharedChannel);
    wavePhy.Set("TxPowerStart", DoubleValue(200.0));  // in dBm
	wavePhy.Set("TxPowerEnd", DoubleValue(200.0));  // in dBm
    NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
  	Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();

    std::string phyMode ("OfdmRate6MbpsBW10MHz");
    wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                    "DataMode",StringValue (phyMode),
                                    "ControlMode",StringValue (phyMode));

    NetDeviceContainer devices_wifi = wifi80211p.Install(wavePhy, wifi80211pMac, nodei);
    Ptr<NetDevice> waveDevice = devices_wifi.Get(0);

    // Log the assigned IP address
    //Ptr<Ipv4> ipv4 = nodei->GetObject<Ipv4>();
    uint32_t interfaceIndex = ipv4_i->AddInterface(waveDevice);

    //uint16_t portwave = 14000 + i;
    uint16_t portwave = 14000;
    std::ostringstream ipWave;
	ipWave << "11.0.0." << i;
    Ipv4Address wave_neti = Ipv4Address(ipWave.str().c_str());
    AddressValue waveLocalAddressi(InetSocketAddress (wave_neti, portwave));

	Ipv4InterfaceAddress ifaceAddress = Ipv4InterfaceAddress(wave_neti, Ipv4Mask("255.255.255.0"));
	ipv4_i->AddAddress(interfaceIndex, ifaceAddress);
    ipv4_i->SetUp(interfaceIndex);

    // Positions and speeds
    Ptr<ConstantVelocityMobilityModel> mobilityi = nodei->GetObject<ConstantVelocityMobilityModel>();
    mobilityi->SetPosition (Vector(x,y,z));
    mobilityi->SetVelocity (Vector(xs,ys,zs));

    // Check installation
    Ptr<Ipv4> ipv4check = nodei->GetObject<Ipv4>();
	for (uint32_t j = 0; j < ipv4check->GetNInterfaces(); ++j)
    {
    	NS_LOG_INFO("Node " << i << " Interface " << j  << " IP: " << ipv4check->GetAddress(j, 0).GetLocal());
	}

    //Mettre en place les paramètres de ROS
    ROSVehicule1Helper rosVehicule1Helper;
    rosVehicule1Helper.SetAttribute ("RemoteROS",remoteAddressi);
    rosVehicule1Helper.SetAttribute ("LocalTap", sinkLocalAddressi);
    rosVehicule1Helper.SetAttribute ("LocalWave", waveLocalAddressi);
    rosVehicule1Helper.SetAttribute ("VehicleNumber", IntegerValue(i));
    rosVehicule1Helper.SetAttribute ("PortTap", UintegerValue(portveh));
    rosVehicule1Helper.SetAttribute ("PortWave", UintegerValue(portwave));
    //Ajout adresse destination dans le node 1 ex :  tap 1 -> wave

    ApplicationContainer ROSVehSyncApps1 = rosVehicule1Helper.Install (nodei);

    ROSVehSyncApps1.Start(Seconds(1.0));
    ROSVehSyncApps1.Stop(Seconds(500));
}

std::vector<std::string> ROSVehSync::SplitCharPointerController(const char* input) {
    std::vector<std::string> result;
    std::istringstream stream(input);
    std::string word;

    while (stream >> word) {
        result.push_back(word);
    }

    return result;
}

 void ROSVehSync::HandleRead (Ptr<Socket> socket)
{
  //NS_LOG_INFO("CONTROLLER SOCKET HAS RECEIVED A MESSAGE");
  NS_LOG_FUNCTION (this << socket);//affichage info de cette fonction

  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom(from)))
  {
    uint8_t *buffer = new uint8_t[packet->GetSize ()];
    packet->CopyData(buffer, packet->GetSize ());

    char* contenu = (char *) buffer;
    NS_LOG_INFO("CONTROLLER SOCKET has received " << contenu);

    std::vector<std::string> instructions = SplitCharPointerController(contenu);

    if (!instructions.empty())
    {
      const std::string& command = instructions[0];
      if(command == "hello")
      {
		//NS_LOG_INFO("Received command hello => responding hello");
    	std::string message = "hello from NS3";
		Ptr<Packet> packet = Create<Packet> ((uint8_t*) message.c_str (), message.length ());
    	socket->Send (packet);
      }
      else if (command == "create_node")
      {
		//NS_LOG_INFO("Create node command");
        unsigned long n = 1;
        int nodeNumber = 1;
        double x = 0.0, y = 0.0, z = 0.0, xs = 0.0, ys = 0.0, zs = 0.0;
        while(n <= instructions.size()-4)
        {
            nodeNumber = std::stoi(instructions[n]);
            ++n;
            std::istringstream(instructions[n]) >> x; // 2) x
            ++n;
    		std::istringstream(instructions[n]) >> y; // 3) y
            ++n;
    		std::istringstream(instructions[n]) >> z; // 4) z

        	CreateVehicle(nodeNumber, x, y, z, xs, ys, zs);

            ++n;
        }
		//NS_LOG_UNCOND("On récupère le total");
        int total = (NodeContainer::GetGlobal().GetN())-1;
        NS_LOG_UNCOND("Global total of vehicules is " << total);

    	std::string message = "create_success";
		Ptr<Packet> packet = Create<Packet> ((uint8_t*) message.c_str (), message.length ());
    	socket->Send (packet);
      }
      else if (command == "assert_total")
      {
		//NS_LOG_INFO("Assert total command");

    	int total = NodeContainer::GetGlobal().GetN()-1;
        NS_LOG_INFO("Total is " << total);

    	std::ostringstream msgStream;
    	msgStream << "total " << total;
    	std::string message = msgStream.str ();
    	Ptr<Packet> packet = Create<Packet> ((uint8_t*) message.c_str (), message.length ());
    	socket->Send (packet);
      }

      else if (command == "set_position")
      {
        //NS_LOG_INFO("Set position command");
		unsigned long n = 1;
        NodeContainer Globalnode;
    	Globalnode = NodeContainer::GetGlobal();
        double x = 0.0, y = 0.0, z = 0.0;
        while(n <= instructions.size()-4)
        {
          	Ptr<ConstantVelocityMobilityModel> mobilityi = Globalnode.Get(std::stoi(instructions[n]))->GetObject<ConstantVelocityMobilityModel>(); // 1) Number node
            ++n;
            std::istringstream(instructions[n]) >> x; // 2) x
            ++n;
    		std::istringstream(instructions[n]) >> y; // 3) y
            ++n;
    		std::istringstream(instructions[n]) >> z; // 4) z

        	const Vector NODE_I_POSITION(x, y, z);
        	mobilityi->SetPosition(NODE_I_POSITION);

            ++n;
        }
      }
      else if (command == "set_speed")
      {
        //NS_LOG_INFO("Set speed command");
		unsigned long n = 1;
        NodeContainer Globalnode;
    	Globalnode = NodeContainer::GetGlobal();
        double xs = 0.0, ys = 0.0, zs = 0.0;
        while(n <= instructions.size()-4)
        {
          	Ptr<ConstantVelocityMobilityModel> mobilityi = Globalnode.Get(std::stoi(instructions[n]))->GetObject<ConstantVelocityMobilityModel>(); // 1) Number node
            ++n;
            std::istringstream(instructions[n]) >> xs; // 2) xs
            ++n;
    		std::istringstream(instructions[n]) >> ys; // 3) ys
            ++n;
    		std::istringstream(instructions[n]) >> zs; // 4) zs

        	const Vector NODE_I_SPEED(xs, ys, zs);
        	mobilityi->SetPosition(NODE_I_SPEED);

            ++n;
        }
      }
      else if (command == "set_mobility")
      {
        //NS_LOG_INFO("Set mobility command");
        unsigned long n = 1;
        NodeContainer Globalnode;
    	Globalnode = NodeContainer::GetGlobal();
        double x = 0.0, y = 0.0, z = 0.0, xs = 0.0, ys = 0.0, zs = 0.0;
        while(n <= instructions.size()-7)
        {
          	Ptr<ConstantVelocityMobilityModel> mobilityi = Globalnode.Get(std::stoi(instructions[n]))->GetObject<ConstantVelocityMobilityModel>(); // 1) Number node
            ++n;
            std::istringstream(instructions[n]) >> x; // 2) xs
            ++n;
    		std::istringstream(instructions[n]) >> y; // 3) ys
            ++n;
    		std::istringstream(instructions[n]) >> z; // 4) zs
            ++n;
            std::istringstream(instructions[n]) >> xs; // 5) xs
            ++n;
    		std::istringstream(instructions[n]) >> ys; // 6) ys
            ++n;
    		std::istringstream(instructions[n]) >> zs; // 7) zs

            const Vector NODE_I_POSITION(x, y, z);
        	const Vector NODE_I_SPEED(xs, ys, zs);
            mobilityi->SetPosition(NODE_I_POSITION);
        	mobilityi->SetVelocity(NODE_I_SPEED);

            ++n;
        }
      }
    }
  }
}



//  Ptr<Packet> packet;//déclaration des différentes variable: packet, adresse IP local et celle de Rtmaps
//  Address from;
//  Address localAddress;
//  while ((packet = socket->RecvFrom (from)))//tant qu'on recoit des données provenant de Rtmaps alors on rentre dans la boucle while
//    {
//      if (packet->GetSize () == 0)//si le paquet est vide on arrête sinon on le comptabilise dans le totalRx -> en byte
//        { //EOF
//          break;
//        }
//      m_totalRx += packet->GetSize ();
//      if (InetSocketAddress::IsMatchingType (from))//si les adresses coïncide avec celle de Rtmaps on affiche les données recus
//        {
//          NS_LOG_INFO ("At time ************ " << Simulator::Now ().As (Time::S)
//                       << " packet sink received "
//                       <<  packet->GetSize () << " bytes from "
//                       << InetSocketAddress::ConvertFrom(from).GetIpv4 ()
//                       << " port " << InetSocketAddress::ConvertFrom (from).GetPort ()
//                       << " total Rx " << m_totalRx << " bytes");
//
//
//
//
////----------------------------------------------------------------------------------------------
//          uint8_t *buffer = new uint8_t[packet->GetSize ()];
//          packet->CopyData(buffer, packet->GetSize ());
//
//          char* contenu = (char *) buffer;
//
//          int nb_vehicles = 7;//Paramètre à modifier
//
//          std::vector<double>  data_in;
//          data_in.reserve(4*nb_vehicles);
//
//
//          const char* single_data_format = "%lf;%n";
//
//          NS_LOG_INFO("Contenu received is exactly " << contenu);
//
//          int used = 0;
//          int offset = 0;
//          // std::sscanf(contenu, data_format, &data_in[0], &data_in[1], &data_in[2], &data_in[3], &data_in[4], &data_in[5], &data_in[6], &data_in[7]);
//          for (int i=0; i<4*nb_vehicles; i++){
//                      std::sscanf(contenu + offset, single_data_format, &data_in[i], &used);
//                      NS_LOG_INFO("At step " << i << " the content is " << data_in[i]);
//                      NS_LOG_INFO("         & offset value \"used\" is " << used);
//                      offset += used;
//          }
//
//
//          for (int i=0; i<nb_vehicles; i++){
//
//            NS_LOG_DEBUG(i << ":ObservedPosition:" << vector_mobilities[i]->GetPosition());
//
//            vector_mobilities[i]->SetPosition(Vector(data_in[4*i],data_in[4*i+1],0));//caractéristique noeud mobilité
//            vector_mobilities[i]->SetVelocity(Vector(data_in[4*i+2],data_in[4*i+3],0));
//
//            NS_LOG_DEBUG(i << ":RequestedPosition:" << vector_mobilities[i]->GetPosition());
//            NS_LOG_DEBUG(i << ":RequestedVelocity:" << vector_mobilities[i]->GetVelocity());
//          }
//        }
//
//      socket->GetSockName (localAddress);
//      m_rxTrace (packet, from);
//      m_rxTraceWithAddresses (packet, from, localAddress);
//
//      if (m_enableSeqTsSizeHeader)
//        {
//          PacketReceived (packet, from, localAddress);
//        }
//    }
//}
//------------------------------------------------------------------------------------------------------------



void
ROSVehSync::PacketReceived (const Ptr<Packet> &p, const Address &from, const Address &localAddress)
{
  NS_LOG_INFO("Packet RECEIVED  NUMEROS 11  ");
  SeqTsSizeHeader header;
  Ptr<Packet> buffer;
//----------------------------------------------------------------------------
  auto itBuffer = m_buffer.find (from);//on selectionne et récupère la bonne adresse du destinataire
  if (itBuffer == m_buffer.end ())
    {
      itBuffer = m_buffer.insert (std::make_pair (from, Create<Packet> (0))).first;
    }
//-------------------------------------------


  buffer = itBuffer->second;
  buffer->AddAtEnd (p);
  buffer->PeekHeader (header);

  NS_ABORT_IF (header.GetSize () == 0);

  while (buffer->GetSize () >= header.GetSize ())
    {
      NS_LOG_DEBUG ("Removing packet of size " << header.GetSize () << " from buffer of size " << buffer->GetSize ());
      Ptr<Packet> complete = buffer->CreateFragment (0, static_cast<uint32_t> (header.GetSize ()));
      buffer->RemoveAtStart (static_cast<uint32_t> (header.GetSize ()));

      complete->RemoveHeader (header);

      m_rxTraceWithSeqTsSize (complete, from, localAddress, header);

      if (buffer->GetSize () > header.GetSerializedSize ())//couche présentation -> serialized.
        {
          buffer->PeekHeader (header);
        }
      else
        {
          break;
        }

    }
}

void ROSVehSync::HandlePeerClose (Ptr<Socket> socket)
{
    NS_LOG_INFO("HANDDLE PEER CLOSE   NUMEROS 12  ");

  NS_LOG_FUNCTION (this << socket);
}

void ROSVehSync::HandlePeerError (Ptr<Socket> socket)
{
  NS_LOG_INFO("HANDDLE PEER Error   NUMEROS 13  ");
  NS_LOG_FUNCTION (this << socket);
}

void ROSVehSync::HandleAccept (Ptr<Socket> s, const Address& from)
{

  NS_LOG_INFO("HANDDLE PEER ACCEPT   NUMEROS 13  ");
  NS_LOG_FUNCTION (this << s << from);
  s->SetRecvCallback (MakeCallback (&ROSVehSync::HandleRead, this));
  m_socketList.push_back (s);
}

//Definition des differentes fonction utile a la compilation de notre algorithme (callback)

//-------------------------------------Mise en place de la communication avec RTMaps et des vehicules------------------
// On a besoin de : 

//NS_LOG_COMPONENT_DEFINE("ROSVehicule");
//NS_OBJECT_ENSURE_REGISTERED(ROSVehicule);


  TypeId ROSVehicule::GetTypeId1 (void)
  {
    static TypeId tid1 = TypeId ("ns3::PacketSink2")
    .SetParent<Application> ()
    .SetGroupName("Applications")
    .AddConstructor<ROSVehicule> ()
    .AddAttribute ("PortTap", "Port on which we send packets to ROS",
                   UintegerValue (11000),
                   MakeUintegerAccessor (&ROSVehicule::portTapi),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("PortWave", "Port on which we send packets to WAVE",
                   UintegerValue (11000),
                   MakeUintegerAccessor (&ROSVehicule::portWavei),
                   MakeUintegerChecker<uint16_t> ())
    .AddAttribute ("RemoteROS", "The address of the destination ROS",
                   AddressValue (),
                   MakeAddressAccessor (&ROSVehicule::ros_ipi),
                   MakeAddressChecker ())
    .AddAttribute ("LocalTap",
                   "The Address on which to Bind the tap socket.",
                   AddressValue (),
                   MakeAddressAccessor (&ROSVehicule::tap_ipi),
                   MakeAddressChecker ())
    .AddAttribute ("LocalWave",
                   "The Address on which to Bind the WAVE socket.",
                   AddressValue (),
                   MakeAddressAccessor (&ROSVehicule::wave_ipi),
                   MakeAddressChecker ())
    .AddAttribute ("VehicleNumber",
                   "The Number of the vehicle",
                   IntegerValue (-1),
                   MakeIntegerAccessor (&ROSVehicule::vehicle_number),
                   MakeIntegerChecker<int> ())
    .AddAttribute ("ProtocolTapSocket", "The type of protocol to use to communicate with Tap device. This should be "
                   "a subclass of ns3::SocketFactory",
                   TypeIdValue (UdpSocketFactory::GetTypeId ()),
                   MakeTypeIdAccessor (&ROSVehicule::m_tapSocket_tidi),
                   MakeTypeIdChecker ())
    .AddAttribute ("ProtocolWaveSocket", "The type of protocol to use to communicate with WAVE. This should be "
                   "a subclass of ns3::SocketFactory",
                   TypeIdValue (UdpSocketFactory::GetTypeId ()),
                   MakeTypeIdAccessor (&ROSVehicule::m_waveSocket_tidi),
                   MakeTypeIdChecker ())
    .AddAttribute ("Adresse",
                   " Address of the wave interface",
                   AddressValue (),
                   MakeAddressAccessor (&ROSVehicule::m_adressewave),
                   MakeAddressChecker ())
    /*.AddAttribute ("Interval",
                   "The time to wait between packets",
                   TimeValue (Seconds (1.0)),
                   MakeTimeAccessor (&ROSVehicule::m_interval1),
                   MakeTimeChecker ())*/
    .AddAttribute ("EnableSeqTsSizeHeader",
                   "Enable optional header tracing of SeqTsSizeHeader",
                   BooleanValue (false),
                   MakeBooleanAccessor (&ROSVehicule::m_enableSeqTsSizeHeader1),
                   MakeBooleanChecker ())
    /*.AddTraceSource ("Tx", "A new packet is created and is sent",
                     MakeTraceSourceAccessor (&ROSVehicule::m_txTrace1),
                     "ns3::Packet::TracedCallback")*/
    .AddTraceSource ("Rx",
                     "A packet has been received",
                     MakeTraceSourceAccessor (&ROSVehicule::m_rxTrace1),
                     "ns3::Packet::AddressTracedCallback")
    .AddTraceSource ("RxWithAddresses", "A packet has been received",
                     MakeTraceSourceAccessor (&ROSVehicule::m_rxTraceWithAddresses1),
                     "ns3::Packet::TwoAddressTracedCallback")
    .AddTraceSource ("RxWithSeqTsSize",
                     "A packet with SeqTsSize header has been received",
                     MakeTraceSourceAccessor (&ROSVehicule::m_rxTraceWithSeqTsSize1),
                     "ns3::ROSVehicule::SeqTsSizeCallback")
    ;
    return tid1;
  }

//******************************************* VOICI LA SECTION QUI CONCERNE L'APPLICATION DES VÉHICULES *************************************** 

  // Constructor
  ROSVehicule::ROSVehicule () {
    NS_LOG_FUNCTION(this);
    tapSocketi = 0;
    m_totalRx1 = 0;
    m_sendEvent_rtmaps1 = EventId ();//obtenir l'id de l'evènement.
  }

  // Destructor
  ROSVehicule::~ROSVehicule() {
    NS_LOG_FUNCTION(this);
    //controlSocket1 = 0;
  }

  void ROSVehicule::DoDispose1 (void) {
    NS_LOG_FUNCTION(this);
    tapSocketi = 0;
    m_socketList1.clear ();//vider le conteneur de socket 
    Application::DoDispose ();
  }

  uint64_t ROSVehicule::GetTotalRx1 () const {
    NS_LOG_FUNCTION (this);
    return m_totalRx1;//pour connaître le nombre de byte reçus.
  }

  //fonction qui va recevoir les sockets de Rtmaps
  Ptr<Socket> ROSVehicule::GetListeningSocket1 (void) const {
    NS_LOG_FUNCTION (this);
    return tapSocketi;
  }

  std::list<Ptr<Socket>> ROSVehicule::GetAcceptedSockets1(void) const {
    NS_LOG_FUNCTION (this);
    return m_socketList1;
  }

  void ROSVehicule::StartApplication (void) {
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("STARTING ROSVehicle " << vehicle_number);

    // Get node
    Ptr<Node> nodei = NodeContainer::GetGlobal().Get(vehicle_number);

    tapSocketi = Socket::CreateSocket(nodei, m_tapSocket_tidi);
    tapSocketi->SetAllowBroadcast (true);//autoriser la communication broadcast
    tapSocketi->Connect(ros_ipi);

    tapSocketi->Bind(tap_ipi);

    if (addressUtils::IsMulticast (tap_ipi))
      {
        Ptr<UdpSocket> udpSocket = DynamicCast<UdpSocket> (tapSocketi);
        if (udpSocket)
          {
            // equivalent to setsockopt (MCAST_JOIN_GROUP)
            udpSocket->MulticastJoinGroup (0, tap_ipi);
          }
        else
          {
            NS_FATAL_ERROR ("Error: joining multicast on a non-UDP socket");
          }
      }


    tapSocketi->SetRecvCallback (MakeCallback (&ROSVehicule::HandleReadTapi, this));
    tapSocketi->SetAcceptCallback (
      MakeNullCallback<bool, Ptr<Socket>, const Address &> (),
      MakeCallback (&ROSVehicule::HandleAccepti, this));
    tapSocketi->SetCloseCallbacks (
      MakeCallback (&ROSVehicule::HandlePeerClosei, this),
      MakeCallback (&ROSVehicule::HandlePeerErrori, this));

    waveSocketi = Socket::CreateSocket(nodei, m_tapSocket_tidi);
    waveSocketi->SetAllowBroadcast(true);
    waveSocketi->Bind(wave_ipi);


    if (addressUtils::IsMulticast (wave_ipi))
    {
      Ptr<UdpSocket> udpSocket = DynamicCast<UdpSocket> (waveSocketi);
      if (udpSocket)
      {
      	// equivalent to setsockopt (MCAST_JOIN_GROUP)
      	udpSocket->MulticastJoinGroup (0, wave_ipi);
      }
      else
      {
      	NS_FATAL_ERROR ("Error: joining multicast on a non-UDP socket");
      }
    }

    waveSocketi->SetRecvCallback (MakeCallback (&ROSVehicule::HandleReadWavei, this));
    waveSocketi->SetAcceptCallback (
      MakeNullCallback<bool, Ptr<Socket>, const Address &> (),
      MakeCallback (&ROSVehicule::HandleAccepti, this));
    waveSocketi->SetCloseCallbacks (
      MakeCallback (&ROSVehicule::HandlePeerClosei, this),
      MakeCallback (&ROSVehicule::HandlePeerErrori, this));
    waveSocketi->SetAllowBroadcast(true);
   
    /* Dans ce code nous allons voir la mise en place du PLAN de données 
    Les changements se sitent dans le code des fonctions 

    StartApplication 
    HandleRead1 -> cette fonction va lire le paquet qu'elle reçoit puis ajouter à la fin de ce dernier l'adresse d'un destinataire qu'on lira par la suite 
    ReplaceDestination 
    ReceiveWave
    //SceduleArtemips  - supprimé car envoie un message toutes les secondes à RTMaps
    Send1 
    */

//************************* SECTION POUR LA RÉCEPTION DE PAQUETS ENNTRE LES NOEUDS**********************************

//    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory"); //Definition du protocole
//    Ptr<Socket> socket = Socket::CreateSocket (GetNode(),tid); //On crée le socket pour la réception des données
//
//    Ptr<Node> node = socket->GetNode(); //On récupère le noeud lié au socket (noeud 1 si application 1 par exemple )
//    Ptr<Ipv4> ip_add = node->GetObject <Ipv4> (); //Récupéraition de l'adresse IP
//    //Ipv4InterfaceAddress iadd_socket = ip_add-> GetAddress(2,0); //On récupère la deuxième adresse de l'interface car interface 1 TAP, interface 2 FdNetDevice
//    //Ipv4Address add_local = iadd_socket.GetLocal();
//    //NS_LOG_INFO("On rentre dans la boucle : --------------------" << add_local);//On affiche ce qu'on récupère
//    //InetSocketAddress local = InetSocketAddress (add_local,80);
//
//    //socket-> Bind(local);//On bind avec l'adresse local qu'on a récupéré auparavant.
//    socket->SetRecvCallback(MakeCallback(&ROSVehicule::ReceiveWave,this));//On appelle le CallBack qui recoit pour qu'on puisse récupérer les paquets et éviter qu'il soit perdu à tout jamais
//
//  // ****************************************** fin de la section pour la réception entre les noeuds***********************
//
//  //--------------------------------------------- SECTION POUR L'ENVOIE VERS RTMAPS-------------------------------------
//    TypeId tid1 = TypeId::LookupByName ("ns3::UdpSocketFactory");//Definition du protocole
//    socket_send_RT=0;//On initialise le socket qui nous sert à l'envoie
//    m_socket_wave = 0;

    //if (socket_send_RT ==0){ // On entre plus dans cette boucle qui envoie un message toutes les secondes à RTMaps
    /*if (socket_send_RT) {
      NS_LOG_UNCOND("CREATE SOCKET FOR SENDING...");
      socket_send_RT = Socket::CreateSocket (GetNode (), tid1);//creation du socket avec toutes les données : le port Rtmaps, l'adresse IP 
      socket_send_RT->SetAllowBroadcast (true);//autoriser la communication broadcast
      socket_send_RT->Connect(ros_ip1);//On connecte à la destination (Adresse IP RTMaps)
      socket_send_RT->ShutdownRecv ();
      NS_LOG_INFO("Start to Send Info.");
      ScheduleArtemipsTransmit1 (Seconds (1.0));
    }*/

    //TypeId tid1 = TypeId::LookupByName("ns3::UdpSocketFactory");
    // Create the socket if not already
//    if (!m_socket1) {
//      m_socket1 = Socket::CreateSocket (GetNode (), controlSocket_tid1);
//      if (m_socket1->Bind (tap_ip1) == -1) {
//        NS_FATAL_ERROR ("Failed to bind socket");
//        NS_LOG_INFO("FAILED TO CREATE SOCKET start mtd");
//      }
//      m_socket1->Listen ();
//      m_socket1->ShutdownSend ();
//      if (addressUtils::IsMulticast (tap_ip1)) {
//        Ptr<UdpSocket> udpSocket = DynamicCast<UdpSocket> (m_socket1);
//        if (udpSocket) {
//          // equivalent to setsockopt (MCAST_JOIN_GROUP)
//          udpSocket->MulticastJoinGroup (0, tap_ip1);
//        }
//        else {
//          NS_FATAL_ERROR ("Error: joining multicast on a non-UDP socket");
//          NS_LOG_INFO("TEST SOCKET ");
//        }
//      }
//    }
//
//    m_socket1->SetRecvCallback (MakeCallback (&ROSVehicule::HandleRead1, this));
//    m_socket1->SetAcceptCallback (MakeNullCallback<bool, Ptr<Socket>, const Address &> (), MakeCallback (&ROSVehicule::HandleAccept1, this));
//    m_socket1->SetCloseCallbacks (MakeCallback (&ROSVehicule::HandlePeerClose1, this), MakeCallback (&ROSVehicule::HandlePeerError1, this));
  }


  void ROSVehicule::StopApplication () { //cette fonction nous permet lors de l'arret de la simulation de premièrement fermer le socket créer vers RTmaps, et aussi ceux provenant de Rtmaps
    NS_LOG_UNCOND("TEST STOP APPLI ");
    NS_LOG_FUNCTION (this);
    while(!m_socketList1.empty ()) {//these are accepted sockets, close them
      NS_LOG_INFO("Wait for socket");
      Ptr<Socket> acceptedSocket = m_socketList1.front ();
      m_socketList1.pop_front ();
      acceptedSocket->Close ();
    }
    if (tapSocketi) {
      tapSocketi->Close ();
      tapSocketi->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
    }
  }

  void  ROSVehicule::SendWithWave (Address WaveSrc, Address WaveDst ,Ptr<Socket>socket, Ptr<Packet>packet) {
    NS_LOG_INFO("La fonction qui envoie le paquet a bien reçus le paquet à envoyer ");

  //Ptr<Node> node = socket->GetNode();
  //Ptr<Ipv4> ip_add = node->GetObject <Ipv4> ();
  //Ipv4InterfaceAddress iadd_socket = ip_add-> GetAddress(1,0);
  //Ipv4Address add_fin = iadd_socket.GetLocal();

  //NS_LOG_INFO(" Adresse qui envoie : " << add_fin);


  //InetSocketAddress remote1 = InetSocketAddress (Ipv4Address("11.0.0.0"),80);
  /*
  source->SetAllowBroadcast (true);
  source->Connect (wave_dest);
  Ptr<Packet> paquet = Create<Packet>();
  source->Send(paquet);
  ReceiveWave(wave_dest,source);
  */
  /*
  socket->SetAllowBroadcast(true);
  socket->Connect(wave_dest);

  Ptr<UdpSocket> Socket_send = DynamicCast<UdpSocket> (socket);
  if(Socket_send) {
    Socket_send->MulticastJoinGroup(0,wave_dest);
  }

  ReceiveWave(wave_dest,Socket_send);
  */
  }



// Voici la fonction qui nous permet de recevoir les paquets provenant d'autre neoud 

  void ROSVehicule::ReceiveWave (Ptr<Socket> socket ) {
    Ptr<Packet> packet;

    while(packet = socket->Recv()) {
      uint8_t *buffer = new uint8_t[packet->GetSize ()];
      packet->CopyData(buffer, packet->GetSize ());
      char* contenu = (char *) buffer;

      NS_LOG_INFO("Paquet recu sur l'interface Wave du noeud " << socket->GetNode()->GetId());
      NS_LOG_INFO(" Contenu du paquet : " << contenu);

      //Send1(packet);
    }
  }

// Modifier le nom Forward  ( Majuscule pour le nom des fonctions  et __ pour les variables )
  void ROSVehicule::Replace_destination (Ptr<Socket> socket, Ptr<Packet> packet) {

  //Recuperation de l'adresse IP et le port  du socket
      
    uint8_t *buffer = new uint8_t[packet->GetSize ()];
    packet->CopyData(buffer, packet->GetSize ());
    char* contenu = (char *) buffer;
    NS_LOG_INFO(" Contenu du paquet dans Replace_Destination " << contenu);

    double gpsx; // Positions GPS du véhicule
    double gpsy; // Positions GPS du véhicule
    int32_t srcVehicle; // Vehicule qui a emis le paquet
    std::sscanf(contenu,"%lf;%lf;%i",&gpsx,&gpsy,&srcVehicle); // Les informations sont separees par des ;
    NS_LOG_INFO("Positions Gps du vehicule " << srcVehicle << " : " << gpsx << " / " << gpsy << ".");

    // Reemission des paquets vers les noeuds RTMaps : @IP  = remote_RTMaps et port udp 12110, 12111, 1211(N-1) (page 36 du rapport de Florian)
    //std::string remote_RTMAPS ("10.58.59.127"); // Adresse de RTMaps (=Adresse du PC) // Adresse ros_ip1
    //uint16_t port_rtmaps_vehicle = 12110;  // Port RTmaps du premier vehicule = 12110 -> 1211(N-1)

    NS_LOG_UNCOND("Create socket for sending to RTMaps");
    
    /**/ // Depuis udp-test.cc dans src/internet/test
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory"); //Definition du protocole 
    socket_send_RT = Socket::CreateSocket (GetNode (), tid); // Creation d'un socket  
  
    //NS_LOG_INFO("Send message from to socket_send_RT");
    // socket_send_RT->SendTo (packet, 0, InetSocketAddress (remote_RTMAPS, 12111)); // Envoi au vehicule 1
    for (int32_t i=0; i<7; i++) {
      if (srcVehicle-1 != i) {
      int32_t udpDestPort = 12110+i;
      NS_LOG_INFO("Send information " << gpsx << "/" << gpsy << " - " << srcVehicle << " to port " << udpDestPort << " Veh " << i);
      //socket_send_RT->SendTo (packet, 0, InetSocketAddress ("10.58.59.127", udpDestPort)); // Pc Benoit
      socket_send_RT->SendTo (packet, 0, InetSocketAddress ("10.58.59.35", udpDestPort)); // Pc Jonathan
      }
    }
    //socket_send_RT->SendTo (packet, 0, InetSocketAddress ("10.58.59.127", 12111)); // Envoi au vehicule 1

    //socket_send_RT->Send (packet);

    /* void UdpSocketLoopbackTest::DoRun () {
     Ptr<Node> rxNode = CreateObject<Node> ();
     InternetStackHelper internet;
     internet.Install (rxNode);

    Ptr<SocketFactory> rxSocketFactory = rxNode->GetObject<UdpSocketFactory> ();
    Ptr<Socket> rxSocket = rxSocketFactory->CreateSocket ();
    rxSocket->Bind (InetSocketAddress (Ipv4Address::GetAny (), 80));
    rxSocket->SetRecvCallback (MakeCallback (&UdpSocketLoopbackTest::ReceivePkt, this));

    Ptr<Socket> txSocket = rxSocketFactory->CreateSocket ();
    txSocket->SendTo (Create<Packet> (246), 0, InetSocketAddress ("127.0.0.1", 80));
    Simulator::Run ();
    Simulator::Destroy ();
    NS_TEST_EXPECT_MSG_EQ (m_receivedPacket->GetSize (), 246, "first socket should not receive it (it is bound specifically to the second interface's address");
    } */

    /*
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    Ptr<Socket> source = Socket::CreateSocket (GetNode(), tid);
    InetSocketAddress remote1 = InetSocketAddress (Addresse_destination,80);
    source->Connect(remote1);
    uint32_t flag=1;

    NS_LOG_INFO("On envoie le paquet ETAPE 1");
    source->SendTo(packet,flag,remote1); // On fait appelle à la fonction qui envoie les paquets !
    NS_LOG_INFO(" ETAPE 1 réalisé");
    */
 }


//Cette fonction permet de scheduler l'envoie des paquets NS3 vers RTMaps
  void ROSVehicule::ScheduleArtemipsTransmiti (Time dt) { //cette fonction nous permet d'envoyer des informations  vers Rtmaps ( timer, et info du LOG)
    NS_LOG_FUNCTION(this << dt);
    m_sendEvent_rtmaps1 = Simulator::Schedule (dt, &ROSVehicule::SendArtemips1, this);
  }

//Voici la fonction qui me permet de faire l'envoie de paquet vers RTMaps
  void ROSVehicule::Send1(Ptr<Packet> packet) {
    NS_LOG_FUNCTION(this << packet);
    NS_LOG_UNCOND("NOUVELLE FONCTION");

    socket_send_RT->Send (packet);
  }

  void ROSVehicule::SendArtemips1 () {
    NS_LOG_FUNCTION(this << socket_send_RT);

    // We don't have anything to send, so we fill the message with imaginary data.

    std::ostringstream msg1;
    //int identifiant1 = 0; 
    //std::string sign1 = " ";

    // Send a big line with a message with various random integers, separated with spaces

    //int route_to_follow1 = rand() % 20 + 1;//choisis des valeur aléatoirement 
    //int nb_veh1 = rand() % 8 + 1;// de même pour le nombre de véhicule 
    //On affiche dans la console 
    NS_LOG_UNCOND("---Envoie RT--- ");

    //on enoie un paquet contenant un msg (chaine de caractère ) à rtmaps avec un rate spécifié par le schedule à la ligne 283 
    msg1 << std::string("HELLOWorld");

    Ptr<Packet> packet = Create<Packet> ((uint8_t*) msg1.str ().c_str (), msg1.str ().length ());
    socket_send_RT->Send (packet);
    
    ScheduleArtemipsTransmiti (Seconds (1.0));
  }

std::vector<std::string> ROSVehicule::SplitCharPointer(const char* input) {
    std::vector<std::string> result;
    std::istringstream stream(input);
    std::string word;

    while (stream >> word) {
        result.push_back(word);
    }

    return result;
}

void ROSVehicule::HandleReadTapi (Ptr<Socket> socket)
{
  //NS_LOG_INFO("VEHICLE SOCKET " << vehicle_number << " HAS RECEIVED A MESSAGE");
  NS_LOG_FUNCTION (this << socket);//affichage info de cette fonction

  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom(from)))
  {
    uint8_t *buffer = new uint8_t[packet->GetSize ()];
    packet->CopyData(buffer, packet->GetSize ());

    char* contenu = (char *) buffer;
    NS_LOG_INFO("VEHICLE TAP "<< vehicle_number <<" has received " << contenu);

    std::vector<std::string> instructions = SplitCharPointer(contenu);

    if (!instructions.empty())
    {
      Ptr<Node> node = NodeContainer::GetGlobal().Get(vehicle_number); // Get each node globally

      Ptr<NetDevice> device = node->GetDevice(2);
      Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
      //Ipv4Address ipAddress = ipv4->GetAddress(1, 0).GetLocal(); // Get the local IP of the node

      // Broadcast address (all nodes in the network)
      for(uint32_t i = 1; i < NodeContainer::GetGlobal().GetN(); i++) {
        if ((int) i == vehicle_number) {
            continue; // Skip our own
        }
        std::ostringstream ipWave;
		ipWave << "11.0.0." << i;
    	Ipv4Address singleAddress = Ipv4Address(ipWave.str().c_str());
        InetSocketAddress remoteAddr(singleAddress, portWavei);
      	waveSocketi->SendTo(packet, 0, remoteAddr);
      }
    }
  }



//  MobilityHelper mobility;
//  mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
//  mobility.Install(GetNode());

//  NS_LOG_UNCOND("HandleRead1");
//  NS_LOG_FUNCTION (this << socket);
//
//  Ptr<Packet> packet;
//  Address from;
//  Address localAddress;
//  Address Adress_socket;
//
//  while ((packet = socket->RecvFrom (from))) {
//    if (packet->GetSize () == 0) {
//      //EOF
//      break;
//    }
//    m_totalRx1 += packet->GetSize ();
//    if (InetSocketAddress::IsMatchingType (from)) {
//      NS_LOG_INFO ("In HandleRead1, at time " << Simulator::Now ().As (Time::S)
//        << " packet sink received "
//        <<  packet->GetSize () << " bytes from "
//        << InetSocketAddress::ConvertFrom(from).GetIpv4 ()
//        << " port " << InetSocketAddress::ConvertFrom (from).GetPort ()
//        << " total Rx " << m_totalRx1 << " bytes");
//    }

    /*NS_LOG_INFO ("At time " << Simulator::Now ().As (Time::S)
      << " packet sink received "
      <<  packet->GetSize () << " bytes from "
      << InetSocketAddress::ConvertFrom(from).GetIpv4 ()
      << " port " << InetSocketAddress::ConvertFrom (from).GetPort ()
      << " total Rx " << m_totalRx1 << " bytes");
    */

  //*************Creation de l'adresse à ajouter à la fin du paquet*************************

  //Fixe le nombre de véhicule:
  //string notation = ";";
  //int m_nombre_vehicule1 = 7;// Paramètre à fixer 
  //int add_choix = rand() % m_nombre_vehicule1 + 1 ;
  //string adresse_add = notation+"11.0.0."+to_string(add_choix);
  //string adresse_add = notation+"11.0.0.255";
  /*string adresse_add = "11.0.0.255";
  
  Ptr<Packet> packet_add = Create<Packet> ((uint8_t*) adresse_add.c_str (), adresse_add.length ());
  packet->AddAtEnd(packet_add);  
  uint8_t *buffer = new uint8_t[packet->GetSize ()];
  NS_LOG_INFO(packet->GetSize ());
  packet->CopyData(buffer, packet->GetSize ());
  char* contenu = (char *) buffer;
  NS_LOG_INFO("VOICI LE CONTENU :" <<contenu);
  NS_LOG_INFO(packet->GetSize ());

  socket->GetSockName (localAddress);
  m_rxTrace1 (packet, from);
  m_rxTraceWithAddresses1 (packet, from, localAddress);
 
    if (m_enableSeqTsSizeHeader1) {
      PacketReceived1 (packet, from, localAddress);
    }*/
    
    //Replace_destination (socket, packet);
  //}
//On passe le paquet avec l'adresse en paramètre de ma fonction. 
}

void ROSVehicule::HandleReadWavei (Ptr<Socket> socket)
{
  //NS_LOG_INFO("VEHICLE WAVE " << vehicle_number << " HAS RECEIVED A MESSAGE");
  NS_LOG_FUNCTION (this << socket);//affichage info de cette fonction

  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom(from)))
  {
    uint8_t *buffer = new uint8_t[packet->GetSize ()];
    packet->CopyData(buffer, packet->GetSize ());

    char* contenu = (char *) buffer;
    NS_LOG_INFO("VEHICLE WAVE " << vehicle_number << " has received " << contenu);

    std::vector<std::string> instructions = SplitCharPointer(contenu);

    if (!instructions.empty())
    {
      NS_LOG_INFO("Sending packet received from wave to tap"<< vehicle_number);
      tapSocketi->Send (packet);
    }
  }
}

void ROSVehicule::PacketReceived1 (const Ptr<Packet> &p, const Address &from, const Address &localAddress)
{
  SeqTsSizeHeader header;
  Ptr<Packet> buffer;
  Ptr<Packet> choice_dest;
  Ptr<Packet> copy_pkt = p->Copy ();

  int choice = rand() % 2 + 1;
  std::cout<<choice<<std::endl;
  //choice_dest -> AddAttribute (choice)
  //----------------------------------------------------------------------------
  auto itBuffer = m_buffer1.find (from);//on selectionne et récupère la bonne adresse du destinataire 
  if (itBuffer == m_buffer1.end ())
    {
      itBuffer = m_buffer1.insert (std::make_pair (from, Create<Packet> (0))).first;
    }
  //-------------------------------------------
  buffer = itBuffer->second;
  buffer->AddAtEnd (p);
  buffer->PeekHeader (header);

  NS_ABORT_IF (header.GetSize () == 0);

  while (buffer->GetSize () >= header.GetSize ())
  {
    NS_LOG_DEBUG ("Removing packet of size " << header.GetSize () << " from buffer of size " << buffer->GetSize ());
    Ptr<Packet> complete = buffer->CreateFragment (0, static_cast<uint32_t> (header.GetSize ()));
    buffer->RemoveAtStart (static_cast<uint32_t> (header.GetSize ()));

    complete->RemoveHeader (header);

    m_rxTraceWithSeqTsSize1 (complete, from, localAddress, header);

    if (buffer->GetSize () > header.GetSerializedSize ())//couche présentation -> serialized.
    {
      buffer->PeekHeader (header);
    }
    else
    {
      break;
    }
  }
}

void ROSVehicule::HandlePeerClosei (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
}
 
void ROSVehicule::HandlePeerErrori (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION (this << socket);
}

void ROSVehicule::HandleAccepti (Ptr<Socket> s, const Address& from)
{
  NS_LOG_FUNCTION (this << s << from);
  s->SetRecvCallback (MakeCallback (&ROSVehicule::HandleReadTapi, this));
  m_socketList1.push_back (s);
}


}
