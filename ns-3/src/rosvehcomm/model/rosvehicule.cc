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
    tapSocketi->SetRecvCallback (MakeCallback (&ROSVehicule::HandleReadTap, this));


    Ipv4Address waveIp = InetSocketAddress::ConvertFrom(wave_ipi).GetIpv4();  // extract Ipv4Address
    InetSocketAddress waveAddr(waveIp, portWavei);

    waveSocketi = Socket::CreateSocket(nodei, m_waveSocket_tidi);
    waveSocketi->SetAllowBroadcast(true);
    waveSocketi->Bind(waveAddr);
    waveSocketi->SetRecvCallback (MakeCallback (&ROSVehicule::HandleReadWave, this));
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

  void ROSVehicule::HandleReadTap(Ptr<Socket> socket)
  {
    NS_LOG_FUNCTION(this << socket);

    Ptr<ConstantVelocityMobilityModel> mobility = GetNode()->GetObject<ConstantVelocityMobilityModel>();
    Ptr<Packet> packet;
    Address from;

    while ((packet = socket->RecvFrom(from)))
    {
      
      uint32_t size = packet->GetSize();
      std::vector<uint8_t> buffer(size);
      packet->CopyData(buffer.data(), size);

      std::ostringstream oss;
      oss << std::hex << std::setfill('0');

      for (uint8_t b : buffer)
      {
        oss << std::setw(2) << static_cast<int>(b) << " ";
      }

      NS_LOG_INFO("VEHICLE TAP " << vehicle_number << " received (hex): " << oss.str());

      uint32_t parse = 0;
      int src = 0;
      int dst = 0;
      const uint8_t* data = buffer.data();

      while (parse < size) {

        uint8_t type = buffer[parse];
        parse++;
        uint8_t length = buffer[parse];
        parse++;

        if ((type == 1) && (length == 1)) {
          src = buffer[parse];
        }
        else if ((type == 2) && (length == 1)) {
          dst = buffer[parse];
        }
        else if ((type == 3) && (length == 12)) {
          float x, y, z;
          std::memcpy(&x, data + parse, sizeof(float));
          std::memcpy(&y, data + parse + 4, sizeof(float));
          std::memcpy(&z, data + parse + 8, sizeof(float));
          const Vector NODE_POSITION(x, y, z);
          mobility->SetPosition(NODE_POSITION);
        }
        else if ((type == 4) && (length == 12)) {
          float vx, vy, vz;
          std::memcpy(&vx, data + parse, sizeof(float));
          std::memcpy(&vy, data + parse + 4, sizeof(float));
          std::memcpy(&vz, data + parse + 8, sizeof(float));
          const Vector NODE_SPEED(vx, vy, vz);
          mobility->SetVelocity(NODE_SPEED);
        }

        parse+=length;
      }

      // unicast basique
      if (dst != 0) {

        std::ostringstream ipWave;
        ipWave << "11.0.0." << dst;
        Ipv4Address singleAddress = Ipv4Address(ipWave.str().c_str());
        InetSocketAddress remoteAddr(singleAddress, portWavei);

        ROSHeader hdr(dst, src);

        Ptr<Packet> p = packet->Copy();
        p->AddHeader(hdr);

        waveSocketi->SendTo(p, 0, remoteAddr);

      }
    }
  }

void ROSVehicule::HandleReadWave (Ptr<Socket> socket)
{
  NS_LOG_FUNCTION(this << socket);

  Ptr<Packet> packet;
  Address from;

  while ((packet = socket->RecvFrom(from)))
  {
    ROSHeader hdr;
    packet->RemoveHeader(hdr);

    uint32_t size = packet->GetSize();
    std::vector<uint8_t> buffer(size);
    packet->CopyData(buffer.data(), size);

    std::ostringstream oss;
    oss << std::hex << std::setfill('0');

    for (uint8_t b : buffer)
    {
      oss << std::setw(2) << static_cast<int>(b) << " ";
    }

    NS_LOG_INFO("VEHICLE WAVE " << vehicle_number << " received: " << hdr << " payload (hex): " << oss.str());

    Ptr<Packet> tapPacket = packet->Copy();

    tapSocketi->Send(tapPacket);
  }
}


}
