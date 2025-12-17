/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ROSHEADER_H
#define ROSHEADER_H

#include "ns3/header.h"
#include "ns3/ipv4-address.h"

namespace ns3 {

class ROSHeader : public Header
{
  public:
    ROSHeader();
    ROSHeader(uint8_t dstId, uint8_t srcId);

    static TypeId GetTypeId(void);
    virtual TypeId GetInstanceTypeId(void) const override;

    virtual void Serialize(Buffer::Iterator start) const override;
    virtual uint32_t Deserialize(Buffer::Iterator start) override;
    virtual uint32_t GetSerializedSize(void) const override;
    virtual void Print(std::ostream &os) const override;

    // Getters
    uint8_t  GetDstId() const;
    uint8_t  GetSrcId() const;

  private:
    uint8_t  m_dstId;
    uint8_t  m_srcId;
  };

} // namespace ns3

#endif