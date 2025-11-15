#include "rosvehicule.h"

using namespace std;

namespace ns3
{

  NS_LOG_COMPONENT_DEFINE("ROSVehicule");
  NS_OBJECT_ENSURE_REGISTERED(ROSVehicule);

  TypeId ROSVehicule::GetTypeId (void)
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
  ROSVehicule::ROSVehicule ()
  {
    NS_LOG_FUNCTION(this);
    tapSocketi = 0;
    m_totalRx1 = 0;
    m_sendEvent_rtmaps1 = EventId ();//obtenir l'id de l'evènement.
  }

  // Destructor
  ROSVehicule::~ROSVehicule()
  {
    NS_LOG_FUNCTION(this);
    //controlSocket1 = 0;
  }

  void ROSVehicule::DoDispose1 (void)
  {
    NS_LOG_FUNCTION(this);
    tapSocketi = 0;
    m_socketList1.clear ();//vider le conteneur de socket 
    Application::DoDispose ();
  }

  uint64_t ROSVehicule::GetTotalRx1 () const
  {
    NS_LOG_FUNCTION (this);
    return m_totalRx1;//pour connaître le nombre de byte reçus.
  }

  //fonction qui va recevoir les sockets de Rtmaps
  Ptr<Socket> ROSVehicule::GetListeningSocket1 (void) const
  {
    NS_LOG_FUNCTION (this);
    return tapSocketi;
  }

  std::list<Ptr<Socket>> ROSVehicule::GetAcceptedSockets1(void) const
  {
    NS_LOG_FUNCTION (this);
    return m_socketList1;
  }

  void ROSVehicule::StartApplication (void)
  {
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
  }


  void ROSVehicule::StopApplication () { //cette fonction nous permet lors de l'arret de la simulation de premièrement fermer le socket créer vers RTmaps, et aussi ceux provenant de Rtmaps
    NS_LOG_UNCOND("TEST STOP APPLI ");
    NS_LOG_FUNCTION (this);
    while(!m_socketList1.empty ())
    {//these are accepted sockets, close them
      NS_LOG_INFO("Wait for socket");
      Ptr<Socket> acceptedSocket = m_socketList1.front ();
      m_socketList1.pop_front ();
      acceptedSocket->Close ();
    }
    if (tapSocketi)
    {
      tapSocketi->Close ();
      tapSocketi->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
    }
  }

  void  ROSVehicule::SendWithWave (Address WaveSrc, Address WaveDst ,Ptr<Socket>socket, Ptr<Packet>packet)
  {
    NS_LOG_INFO("La fonction qui envoie le paquet a bien reçus le paquet à envoyer ");
  }



// Voici la fonction qui nous permet de recevoir les paquets provenant d'autre neoud 

  void ROSVehicule::ReceiveWave (Ptr<Socket> socket )
  {
    Ptr<Packet> packet;

    while(packet = socket->Recv())
    {
      uint8_t *buffer = new uint8_t[packet->GetSize ()];
      packet->CopyData(buffer, packet->GetSize ());
      char* contenu = (char *) buffer;

      NS_LOG_INFO("Paquet recu sur l'interface Wave du noeud " << socket->GetNode()->GetId());
      NS_LOG_INFO(" Contenu du paquet : " << contenu);
      //Send1(packet);
    }
  }

  // Modifier le nom Forward  ( Majuscule pour le nom des fonctions  et __ pour les variables )
  void ROSVehicule::Replace_destination (Ptr<Socket> socket, Ptr<Packet> packet)
  {

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
    for (int32_t i=0; i<7; i++)
    {
      if (srcVehicle-1 != i)
      {
        int32_t udpDestPort = 12110+i;
        NS_LOG_INFO("Send information " << gpsx << "/" << gpsy << " - " << srcVehicle << " to port " << udpDestPort << " Veh " << i);
        //socket_send_RT->SendTo (packet, 0, InetSocketAddress ("10.58.59.127", udpDestPort)); // Pc Benoit
        socket_send_RT->SendTo (packet, 0, InetSocketAddress ("10.58.59.35", udpDestPort)); // Pc Jonathan
      }
    }
 }


  //Cette fonction permet de scheduler l'envoie des paquets NS3 vers RTMaps
  void ROSVehicule::ScheduleArtemipsTransmiti (Time dt)
  { //cette fonction nous permet d'envoyer des informations  vers Rtmaps ( timer, et info du LOG)
    NS_LOG_FUNCTION(this << dt);
    m_sendEvent_rtmaps1 = Simulator::Schedule (dt, &ROSVehicule::SendArtemips1, this);
  }

  //Voici la fonction qui me permet de faire l'envoie de paquet vers RTMaps
  void ROSVehicule::Send1(Ptr<Packet> packet)
  {
    NS_LOG_FUNCTION(this << packet);
    NS_LOG_UNCOND("NOUVELLE FONCTION");

    socket_send_RT->Send (packet);
  }

  void ROSVehicule::SendArtemips1 ()
  {
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

  std::vector<std::string> ROSVehicule::SplitCharPointer(const char* input)
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
        for(uint32_t i = 1; i < NodeContainer::GetGlobal().GetN(); i++)
        {
          if ((int) i == vehicle_number)
          {
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
