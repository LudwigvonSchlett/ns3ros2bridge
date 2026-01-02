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
    Ptr<NetDevice> tapDev = nodei->GetDevice(1);
    Ptr<NetDevice> wifiDev = nodei->GetDevice(2);

    //Tap
    tapSocketi = Socket::CreateSocket(nodei, m_tapSocket_tidi);
    tapSocketi->SetAllowBroadcast (true);//autoriser la communication broadcast
    tapSocketi->Bind(tap_ipi); // accepte adrresseTap::portTap
    tapSocketi->BindToNetDevice(tapDev); // bloque à uniquement l'interface tap
    tapSocketi->Connect(ros_ipi);
    tapSocketi->SetRecvCallback (MakeCallback (&ROSVehicule::HandleReadTap, this));

    //Wave
    InetSocketAddress wave_ipi(Ipv4Address::GetAny(), portWavei); // 0.0.0.0::portWavei

    waveSocketi = Socket::CreateSocket(nodei, m_waveSocket_tidi);
    waveSocketi->SetAllowBroadcast(true);
    waveSocketi->Bind(wave_ipi); // accepte tout sur son le portWavei
    waveSocketi->BindToNetDevice(wifiDev); // bloque à uniquement l'interface wave
    waveSocketi->SetRecvCallback (MakeCallback (&ROSVehicule::HandleReadWave, this));
  }


  void ROSVehicule::StopApplication () { //cette fonction nous permet lors de l'arret de la simulation de premièrement fermer le socket créer vers ros2
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

        uint8_t type = data[parse];
        parse++;
        uint8_t length = data[parse];
        parse++;

        if ((type == 1) && (length == 1)) {
          src = data[parse];
        }
        else if ((type == 2) && (length == 1)) {
          dst = data[parse];
        }
        // permet de lire les messages de position et de mettre à jour sa propre position
        else if ((type == 3) && (length == 16)) {
          float x, y, z;
          uint8_t pos_src = data[parse];
          std::memcpy(&x, data + parse + 4, sizeof(float));
          std::memcpy(&y, data + parse + 8, sizeof(float));
          std::memcpy(&z, data + parse + 12, sizeof(float));
          const Vector NODE_POSITION(x, y, z);
          if (pos_src == vehicle_number) {
            //mobility->SetPosition(NODE_POSITION); // permet au noeud de mettre à jour sa position
            //NS_LOG_INFO("VEHICLE TAP " << vehicle_number << " would update position to " << NODE_POSITION);
          }
        }
        // permet de lire les messages de position et de mettre à jour sa propre vitesse
        else if ((type == 4) && (length == 16)) {
          float vx, vy, vz;
          uint8_t vel_src = data[parse];
          std::memcpy(&vx, data + parse + 4, sizeof(float));
          std::memcpy(&vy, data + parse + 8, sizeof(float));
          std::memcpy(&vz, data + parse + 12, sizeof(float));
          const Vector NODE_SPEED(vx, vy, vz);
          if (vel_src == vehicle_number) {
            //mobility->SetVelocity(NODE_SPEED); // permet au noeud de mettre à jour sa vitesse
            //NS_LOG_INFO("VEHICLE TAP " << vehicle_number << " would update speed to " << NODE_SPEED);
          }
        }
        else {
          NS_LOG_ERROR("Message unable to be parsed");
        }

        parse+=length;
      }

      // broadcast basique
      if (dst == 255) {

        InetSocketAddress remoteAddr(Ipv4Address("11.0.0.255"), portWavei);

        ROSHeader hdr(dst, src);

        Ptr<Packet> p = packet->Copy();
        p->AddHeader(hdr);

        waveSocketi->SendTo(p, 0, remoteAddr);

      }
      // unicast basique
      else if ((dst != 255) && (dst != 0)) {

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
