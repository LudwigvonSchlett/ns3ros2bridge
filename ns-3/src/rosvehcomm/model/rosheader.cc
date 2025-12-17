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
  : m_dstId(0), m_srcId(0)
{
}

ROSHeader::ROSHeader(uint8_t dstId, uint8_t srcId)
  : m_dstId(dstId), m_srcId(srcId)
{
}

uint32_t
ROSHeader::GetSerializedSize(void) const
{
  return 2;  // 2 uint8
}

void
ROSHeader::Serialize(Buffer::Iterator i) const
{
  i.WriteU8(m_dstId);
  i.WriteU8(m_srcId);
}

uint32_t
ROSHeader::Deserialize(Buffer::Iterator i)
{
  m_dstId = i.ReadU8();
  m_srcId = i.ReadU8();
  return GetSerializedSize();
}

void
ROSHeader::Print(std::ostream &os) const
{
  os << "dst=" << uint32_t(m_dstId)
     << " src=" << uint32_t(m_srcId);
}

uint8_t  ROSHeader::GetDstId() const { return m_dstId; }
uint8_t  ROSHeader::GetSrcId() const { return m_srcId; }

} // namespace ns3
