/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#include "rosheader.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("ROSHeader");
NS_OBJECT_ENSURE_REGISTERED(ROSHeader);

TypeId
ROSHeader::GetTypeId(void)
{
  static TypeId tid = TypeId("ns3::ROSHeader")
    .SetParent<Header>()
    .AddConstructor<ROSHeader>();
  return tid;
}

TypeId
ROSHeader::GetInstanceTypeId(void) const
{
  return GetTypeId();
}

ROSHeader::ROSHeader()
  : m_dstId(0), m_srcId(0), m_x(0), m_y(0), m_z(0)
{
}

ROSHeader::ROSHeader(uint8_t dstId, uint8_t srcId,
                                   int32_t x, int32_t y, int32_t z)
  : m_dstId(dstId), m_srcId(srcId),
    m_x(x), m_y(y), m_z(z)
{
}

uint32_t
ROSHeader::GetSerializedSize(void) const
{
  return 2 + 3 * 4;  // 2 uint8 + 3 int32
}

void
ROSHeader::Serialize(Buffer::Iterator i) const
{
  i.WriteU8(m_dstId);
  i.WriteU8(m_srcId);
  i.WriteHtonU32(m_x);
  i.WriteHtonU32(m_y);
  i.WriteHtonU32(m_z);
}

uint32_t
ROSHeader::Deserialize(Buffer::Iterator i)
{
  m_dstId = i.ReadU8();
  m_srcId = i.ReadU8();
  m_x = i.ReadNtohU32();
  m_y = i.ReadNtohU32();
  m_z = i.ReadNtohU32();
  return GetSerializedSize();
}

void
ROSHeader::Print(std::ostream &os) const
{
  os << "dst=" << uint32_t(m_dstId)
     << " src=" << uint32_t(m_srcId)
     << " pos=(" << m_x << "," << m_y << "," << m_z << ")";
}

uint8_t  ROSHeader::GetDstId() const { return m_dstId; }
uint8_t  ROSHeader::GetSrcId() const { return m_srcId; }
int32_t  ROSHeader::GetX() const { return m_x; }
int32_t  ROSHeader::GetY() const { return m_y; }
int32_t  ROSHeader::GetZ() const { return m_z; }

} // namespace ns3
