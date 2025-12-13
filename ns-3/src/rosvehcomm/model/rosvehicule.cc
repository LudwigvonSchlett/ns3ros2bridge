#include "rosvehicule.h"

using namespace std;

namespace ns3
{

  NS_LOG_COMPONENT_DEFINE("ROSVehicule");
  NS_OBJECT_ENSURE_REGISTERED(ROSVehicule);

  TypeId ROSVehicule::GetTypeId()
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
    NS_LOG_INFO("Constructeur Rosvehicule");
    tapSocketi = nullptr;
    m_totalRx1 = 0;
    m_sendEvent_rtmaps1 = EventId ();//obtenir l'id de l'evènement.
  }

  // Destructor
  ROSVehicule::~ROSVehicule()
  {
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Destructeur Rosvehicule "  << vehicle_number);
    //controlSocket1 = 0;
  }

  void ROSVehicule::DoDispose ()
  {
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("DoDispose Rosvehicule "  << vehicle_number);
    tapSocketi = nullptr;
    m_socketList1.clear ();//vider le conteneur de socket 
    Application::DoDispose ();
  }

  void ROSVehicule::StartApplication ()
  {
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Starting RosVehicle " << vehicle_number);

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
    NS_LOG_INFO("Stopping Rosvehicule "  << vehicle_number);
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

      // unicast basique
      if (!instructions.empty()) {

        std::ostringstream ipWave;
        ipWave << "11.0.0." << instructions[0];
        Ipv4Address singleAddress = Ipv4Address(ipWave.str().c_str());
        InetSocketAddress remoteAddr(singleAddress, portWavei);
        waveSocketi->SendTo(packet, 0, remoteAddr);

      }
    }
  }

  void ROSVehicule::HandleReadWavei (Ptr<Socket> socket)
  {
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

  void ROSVehicule::HandlePeerClosei (Ptr<Socket> socket)
  {
    NS_LOG_FUNCTION (this << socket);
  }

  void ROSVehicule::HandlePeerErrori (Ptr<Socket> socket)
  {
    NS_LOG_FUNCTION (this << socket);
  }

  void ROSVehicule::HandleAccepti (Ptr<Socket> socket, const Address& from)
  {
    NS_LOG_FUNCTION (this << socket << from);
    socket->SetRecvCallback (MakeCallback (&ROSVehicule::HandleReadTapi, this));
    m_socketList1.push_back (socket);
  }

}
