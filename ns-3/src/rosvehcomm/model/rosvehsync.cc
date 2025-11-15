#include "rosvehsync.h"

using namespace std;

namespace ns3
{

  SimInfo simInfo;

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
    NS_LOG_INFO("Constructeur RosvehSync");
    m_sendEvent_rtmaps = EventId ();//obtenir l'id de l'evènement.

    // Create Wave PHY using the shared channel
	  YansWifiChannelHelper waveChannel = YansWifiChannelHelper::Default();
	  sharedChannel = waveChannel.Create();
  }

  // Destructeur
  ROSVehSync::~ROSVehSync ()
  {
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Destructeur RosvehSync");
    controlSocket = 0;
  }

  void ROSVehSync::DoDispose (void)
  {
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("DoDispose RosvehSync");
    m_socket_from_rtmaps = 0;
    m_socketList.clear ();//vider le conteneur de socket
    Application::DoDispose ();
  }

  //Fonction qui lance l'appliction
  void ROSVehSync::StartApplication (void)
  {
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Starting ROSVehSync");

    Ptr<Node> node = GetNode();

	  if (node->GetObject<MobilityModel>() == nullptr)
    {
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
      MakeCallback (&ROSVehSync::HandleAccept, this)
    );
    controlSocket->SetCloseCallbacks (
      MakeCallback (&ROSVehSync::HandlePeerClose, this),
      MakeCallback (&ROSVehSync::HandlePeerError, this)
    );
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
    ROSVehiculeHelper rosVehiculeHelper;
    rosVehiculeHelper.SetAttribute ("RemoteROS",remoteAddressi);
    rosVehiculeHelper.SetAttribute ("LocalTap", sinkLocalAddressi);
    rosVehiculeHelper.SetAttribute ("LocalWave", waveLocalAddressi);
    rosVehiculeHelper.SetAttribute ("VehicleNumber", IntegerValue(i));
    rosVehiculeHelper.SetAttribute ("PortTap", UintegerValue(portveh));
    rosVehiculeHelper.SetAttribute ("PortWave", UintegerValue(portwave));
    //Ajout adresse destination dans le node 1 ex :  tap 1 -> wave

    ApplicationContainer ROSVehSyncApps1 = rosVehiculeHelper.Install (nodei);

    ROSVehSyncApps1.Start(Seconds(1.0));
    ROSVehSyncApps1.Stop(simInfo.duration);
  }

  std::vector<std::string> ROSVehSync::SplitCharPointerController(const char* input)
  {
    std::vector<std::string> result;
    std::istringstream stream(input);
    std::string word;

    while (stream >> word)
    {
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
        if(command == "hello_ROS2")
        {
		      //NS_LOG_INFO("Received command hello => responding hello");
    	    std::string message = "hello_NS3";
		      Ptr<Packet> packet = Create<Packet> ((uint8_t*) message.c_str (), message.length ());
    	    socket->Send (packet);
        }
        else if (command == "request_animfile")
        {
          std::string message = "file " + simInfo.filename;
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

}  