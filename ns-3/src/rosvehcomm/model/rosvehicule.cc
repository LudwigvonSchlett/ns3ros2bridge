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
    waveSocketi = nullptr;
    m_totalRx1 = 0;
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
    waveSocketi = nullptr;
    Application::DoDispose ();
  }

  void ROSVehicule::StartApplication ()
  {
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("Starting RosVehicle " << vehicle_number);

    // Get node
    Ptr<Node> nodei = GetNode();

    Ipv4Address tapIp = InetSocketAddress::ConvertFrom(tap_ipi).GetIpv4();  // extract Ipv4Address
    InetSocketAddress tapAddr(tapIp, portTapi);

    tapSocketi = Socket::CreateSocket(nodei, m_tapSocket_tidi);
    tapSocketi->SetAllowBroadcast (true);//autoriser la communication broadcast
    tapSocketi->Bind(tapAddr);
    tapSocketi->Connect(ros_ipi);
    tapSocketi->SetRecvCallback (MakeCallback (&ROSVehicule::HandleReadTapi, this));


    Ipv4Address waveIp = InetSocketAddress::ConvertFrom(wave_ipi).GetIpv4();  // extract Ipv4Address
    InetSocketAddress waveAddr(waveIp, portWavei);

    waveSocketi = Socket::CreateSocket(nodei, m_waveSocket_tidi);
    waveSocketi->SetAllowBroadcast(true);
    waveSocketi->Bind(waveAddr);
    waveSocketi->SetRecvCallback (MakeCallback (&ROSVehicule::HandleReadWavei, this));
  }


  void ROSVehicule::StopApplication () { //cette fonction nous permet lors de l'arret de la simulation de premièrement fermer le socket créer vers RTmaps, et aussi ceux provenant de Rtmaps
    NS_LOG_UNCOND("TEST STOP APPLI ");
    NS_LOG_FUNCTION (this);
    NS_LOG_INFO("Stopping Rosvehicule "  << vehicle_number);
    if (tapSocketi)
    {
      tapSocketi->Close ();
      tapSocketi->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
    }
    if (waveSocketi)
    {
      waveSocketi->Close ();
      waveSocketi->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
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

  void ROSVehicule::HandleReadTapi(Ptr<Socket> socket)
  {
    NS_LOG_FUNCTION(this << socket);

    Ptr<Packet> packet;
    Address from;

    while ((packet = socket->RecvFrom(from)))
    {
      uint8_t *buffer = new uint8_t[packet->GetSize ()];
      packet->CopyData(buffer, packet->GetSize ());

      char* contenu = reinterpret_cast<char*>(buffer);

      NS_LOG_INFO("VEHICLE TAP " << vehicle_number << " received: " << contenu);

      std::vector<std::string> instructions = SplitCharPointer(contenu);

      // unicast basique
      if (!instructions.empty()) {

        std::ostringstream ipWave;
        ipWave << "11.0.0." << instructions[0];
        Ipv4Address singleAddress = Ipv4Address(ipWave.str().c_str());
        InetSocketAddress remoteAddr(singleAddress, portWavei);
        Ptr<Packet> copy = packet->Copy();
        waveSocketi->SendTo(copy, 0, remoteAddr);

      }
      delete[] buffer;
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

      char* contenu = reinterpret_cast<char*>(buffer);

      NS_LOG_INFO("VEHICLE WAVE " << vehicle_number
                   << " has received " << contenu);

      std::vector<std::string> instructions = SplitCharPointer(contenu);

      if (!instructions.empty())
      {
        Ptr<Packet> copy = packet->Copy();
        tapSocketi->Send(copy);
      }
      delete[] buffer;
    }
  }


}
