#include "rosvehsync.h"

using namespace std;

namespace ns3
{

  SimInfo simInfo;

  NS_LOG_COMPONENT_DEFINE("ROSVehSync");
  NS_OBJECT_ENSURE_REGISTERED(ROSVehSync);

  TypeId ROSVehSync::GetTypeId ()
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
    controlSocket = nullptr;
  }

  void ROSVehSync::DoDispose ()
  {
    NS_LOG_FUNCTION(this);
    NS_LOG_INFO("DoDispose RosvehSync");
    m_socket_from_rtmaps = nullptr;
    m_socketList.clear ();//vider le conteneur de socket
    Application::DoDispose ();
  }

  //Fonction qui lance l'appliction
  void ROSVehSync::StartApplication ()
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
    controlSocket = nullptr;

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
  ROSVehSync::StopApplication ()//cette fonction nous permet lors de l'arret de la simulation de premièrement fermer le socket créé vers RTmaps, et aussi ceux provenant de Rtmaps
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
      Ptr<Socket> acceptedSocket = m_socketList.front ();//pour envoyer la référence du premier element et la stocker dans une variable qui s'appelle acceptedsocket
      m_socketList.pop_front ();//supprime le premier element de la liste m_socketlist
      acceptedSocket->Close ();//ferme le socket
    }
    if (m_socket_from_rtmaps) //si on recoit toujours quelque chose de rtmaps alors on le ferme
    {
      m_socket_from_rtmaps->Close ();
      m_socket_from_rtmaps->SetRecvCallback (MakeNullCallback<void, Ptr<Socket> > ());
    }
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

      uint32_t size = packet->GetSize();
      std::vector<uint8_t> buffer(size);
      packet->CopyData(buffer.data(), size);

      std::ostringstream oss;
      oss << std::hex << std::setfill('0');

      for (uint8_t b : buffer)
      {
        oss << std::setw(2) << static_cast<int>(b) << " ";
      }

      NS_LOG_INFO("CONTROLLER SOCKET has received (hex): " << oss.str());

      uint32_t parse = 0;
      Ptr<Packet> responsePacket;
      const uint8_t* data = buffer.data();
      NodeContainer Globalnode = NodeContainer::GetGlobal();

      while (parse < size) {

        uint8_t type = data[parse];
        parse++;
        uint8_t length = data[parse];
        parse++;

        if ((type == 0) && (length == 1) && (data[parse] == 0)) { // hello ros
          std::string message = "hello_NS3";
          uint8_t *packet_content = new uint8_t[3];
          packet_content[0] = 0;
          packet_content[1] = 1;
          packet_content[2] = 1; // hello ns3
          responsePacket = Create<Packet> (packet_content, 3);
          socket->Send (responsePacket);
        }
        else if ((type == 1) && (length == 1)) {
          int src = data[parse];
          NS_LOG_INFO("Message has source " << src);
        }
        else if ((type == 2) && (length == 1)) {
          int dst = data[parse];
          NS_LOG_INFO("Message has destination " << dst);
        }
        // permet de lire les messages de position et de mettre à jour la position du noeud concerné
        else if ((type == 3) && (length == 16)) {
          float x, y, z;
          uint8_t pos_src = data[parse];
          Ptr<ConstantVelocityMobilityModel> mobility = Globalnode.Get(pos_src)->GetObject<ConstantVelocityMobilityModel>();
          std::memcpy(&x, data + parse + 4, sizeof(float));
          std::memcpy(&y, data + parse + 8, sizeof(float));
          std::memcpy(&z, data + parse + 12, sizeof(float));
          const Vector NODE_POSITION(x, y, z);
          mobility->SetPosition(NODE_POSITION); // permet au noeud de mettre à jour sa position
          NS_LOG_INFO("Node " << static_cast<unsigned int>(pos_src) << " now has position " << NODE_POSITION);
        }
        // permet de lire les messages de position et de mettre à jour la vitesse du noeud concerné
        else if ((type == 4) && (length == 16)) {
          float vx, vy, vz;
          uint8_t vel_src = data[parse];
          Ptr<ConstantVelocityMobilityModel> mobility = Globalnode.Get(vel_src)->GetObject<ConstantVelocityMobilityModel>();
          std::memcpy(&vx, data + parse + 4, sizeof(float));
          std::memcpy(&vy, data + parse + 8, sizeof(float));
          std::memcpy(&vz, data + parse + 12, sizeof(float));
          const Vector NODE_SPEED(vx, vy, vz);
          mobility->SetVelocity(NODE_SPEED); // permet au noeud de mettre à jour sa vitesse
          NS_LOG_INFO("Node " << static_cast<unsigned int>(vel_src) << " now has speed " << NODE_SPEED);
        }
        // requetes de ros
        else if ((type == 101) && (length == 1) && (data[parse] == 0)) { // request_duration
          Time::Unit unit = Time::Unit::S;
          uint16_t duration = simInfo.duration.ToInteger(unit);
          uint8_t *packet_content = new uint8_t[4];
          packet_content[0] = 201; // reponse duration
          packet_content[1] = 2;
          std::memcpy(&packet_content[2], &duration, sizeof(uint16_t));
          responsePacket = Create<Packet> (packet_content, 4);
          socket->Send (responsePacket);
        }
        else if ((type == 102) && (length == 1) && (data[parse] == 0)) { // request_node
          uint8_t node = simInfo.nodeCount;
          uint8_t *packet_content = new uint8_t[3];
          packet_content[0] = 202; // reponse node
          packet_content[1] = 1;
          std::memcpy(&packet_content[2], &node, sizeof(uint8_t));
          responsePacket = Create<Packet> (packet_content, 3);
          socket->Send (responsePacket);
        }
        else if ((type == 103) && (length == 1) && (data[parse] == 0)) { // request_animfile
          std::string filename = simInfo.filename;
          uint8_t *packet_content = new uint8_t[2+filename.length()];
          packet_content[0] = 203; // reponse file
          packet_content[1] = filename.length();
          std::memcpy(&packet_content[2], filename.data(), filename.length());
          responsePacket = Create<Packet> (packet_content, 2+filename.length());
          socket->Send (responsePacket);
        }
        else if ((type == 104) && (length == 1) && (data[parse] == 0)) { // request_time
          Time::Unit unit = Time::Unit::S;
          uint16_t time = Simulator::Now().ToInteger(unit);
          uint8_t *packet_content = new uint8_t[4];
          packet_content[0] = 204; // reponse time
          packet_content[1] = 2;
          std::memcpy(&packet_content[2], &time, sizeof(uint16_t));
          responsePacket = Create<Packet> (packet_content, 4);
          socket->Send (responsePacket);
        }
        else {
          NS_LOG_ERROR("Message unable to be parsed");
        }

        parse+=length;
      }
    }
  }

  void ROSVehSync::HandlePeerClose (Ptr<Socket> socket)
  {
    NS_LOG_INFO("HANDLE PEER CLOSE   NUMEROS 12  ");
    NS_LOG_FUNCTION (this << socket);
  }

  void ROSVehSync::HandlePeerError (Ptr<Socket> socket)
  {
    NS_LOG_INFO("HANDLE PEER Error   NUMEROS 13  ");
    NS_LOG_FUNCTION (this << socket);
  }

  void ROSVehSync::HandleAccept (Ptr<Socket> s, const Address& from)
  {
    NS_LOG_INFO("HANDLE PEER ACCEPT   NUMEROS 13  ");
    NS_LOG_FUNCTION (this << s << from);
    s->SetRecvCallback (MakeCallback (&ROSVehSync::HandleRead, this));
    m_socketList.push_back (s);
  }

}  