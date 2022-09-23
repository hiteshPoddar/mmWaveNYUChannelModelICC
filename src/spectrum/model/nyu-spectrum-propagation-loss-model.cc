/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2022 New York University and NYU WIRELESS
 * Permission is hereby granted, free of charge, to any person obtaining a 
 * copy of this software and associated documentation files (the “Software”),
 * to deal in the Software without restriction, including without limitation 
 * the rights to use, copy, modify, merge, publish, distribute, sublicense, 
 * and/or sell copies of the Software, and to permit persons to whom the 
 * Software is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software. Users shall cite 
 * NYU WIRELESS publications regarding this work.

 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUTWARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR 
 * OTHER LIABILITY, WHETHER INANACTION OF CONTRACT TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR 
 * OTHER DEALINGS IN THE SOFTWARE.
 * 
 * Author: Hitesh Poddar
 */

#include "ns3/log.h"
#include "nyu-spectrum-propagation-loss-model.h"
#include "ns3/net-device.h"
#include "ns3/phased-array-model.h"
#include "ns3/node.h"
#include "ns3/channel-condition-model.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/simulator.h"
#include "ns3/pointer.h"
#include <map>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NYUSpectrumPropagationLossModel");

NS_OBJECT_ENSURE_REGISTERED (NYUSpectrumPropagationLossModel);

NYUSpectrumPropagationLossModel::NYUSpectrumPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
  m_uniformRv = CreateObject<UniformRandomVariable> ();
}

NYUSpectrumPropagationLossModel::~NYUSpectrumPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

void
NYUSpectrumPropagationLossModel::DoDispose ()
{
  m_deviceAntennaMap.clear ();
  m_longTermMap.clear ();
  m_channelModel->Dispose ();
  m_channelModel = nullptr;
}

TypeId
NYUSpectrumPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NYUSpectrumPropagationLossModel")
    .SetParent<SpectrumPropagationLossModel> ()
    .SetGroupName ("Spectrum")
    .AddConstructor<NYUSpectrumPropagationLossModel> ()
    .AddAttribute("ChannelModel", 
                  "The channel model. It needs to implement the MatrixBasedChannelModel interface",
                  StringValue("ns3::NYUChannelModel"),
                  MakePointerAccessor (&NYUSpectrumPropagationLossModel::SetChannelModel,
                                       &NYUSpectrumPropagationLossModel::GetChannelModel),
                                       MakePointerChecker<MatrixBasedChannelModel> ())
    ;
  return tid;
}

void
NYUSpectrumPropagationLossModel::SetChannelModel (Ptr<MatrixBasedChannelModel> channel)
{
  m_channelModel = channel;
}

Ptr<MatrixBasedChannelModel>
NYUSpectrumPropagationLossModel::GetChannelModel () const
{
  return m_channelModel;
}

void
NYUSpectrumPropagationLossModel::AddDevice (Ptr<NetDevice> n, Ptr<const PhasedArrayModel> a)
{
  NS_ASSERT_MSG (m_deviceAntennaMap.find (n->GetNode ()->GetId ()) == m_deviceAntennaMap.end (), "Device is already present in the map");
  m_deviceAntennaMap.insert (std::make_pair (n->GetNode ()->GetId (), a));
}

double
NYUSpectrumPropagationLossModel::GetFrequency () const
{
  DoubleValue freq;
  m_channelModel->GetAttribute ("Frequency", freq);
  return freq.Get ();
}

void
NYUSpectrumPropagationLossModel::SetChannelModelAttribute (const std::string &name, const AttributeValue &value)
{
  m_channelModel->SetAttribute (name, value);
}

void
NYUSpectrumPropagationLossModel::GetChannelModelAttribute (const std::string &name, AttributeValue &value) const
{
  m_channelModel->GetAttribute (name, value);
}

PhasedArrayModel::ComplexVector
NYUSpectrumPropagationLossModel::CalcLongTerm (Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
                                                const PhasedArrayModel::ComplexVector &sW,
                                                const PhasedArrayModel::ComplexVector &uW) const
{
  NS_LOG_FUNCTION (this);

  uint16_t sAntenna = static_cast<uint16_t> (sW.size ());
  uint16_t uAntenna = static_cast<uint16_t> (uW.size ());

  NS_LOG_DEBUG ("CalcLongTerm with sAntenna " << sAntenna << " uAntenna " << uAntenna);
  //store the long term part to reduce computation load
  //only the small scale fading needs to be updated if the large scale parameters and antenna weights remain unchanged.
  PhasedArrayModel::ComplexVector longTerm;
  uint8_t numRays = static_cast<uint8_t> (params->m_channel[0][0].size ());

  for (uint8_t cIndex = 0; cIndex < numRays; cIndex++)
    {
      std::complex<double> txSum (0,0);
      for (uint16_t sIndex = 0; sIndex < sAntenna; sIndex++)
        {
          std::complex<double> rxSum (0,0);
          for (uint16_t uIndex = 0; uIndex < uAntenna; uIndex++)
            {
              rxSum = rxSum + uW[uIndex] * params->m_channel[uIndex][sIndex][cIndex];
            }
          txSum = txSum + sW[sIndex] * rxSum;
        }
      longTerm.push_back (txSum);
    }
  return longTerm;
}

Ptr<SpectrumValue>
NYUSpectrumPropagationLossModel::CalcBeamformingGain (Ptr<SpectrumValue> txPsd,
                                                      PhasedArrayModel::ComplexVector longTerm,
                                                      Ptr<const MatrixBasedChannelModel::ChannelMatrix> params,
                                                      const ns3::Vector &sSpeed, const ns3::Vector &uSpeed) const
{
  NS_LOG_FUNCTION (this);

  Ptr<SpectrumValue> tempPsd = Copy<SpectrumValue> (txPsd);

  //channel[rx][tx][cluster]
  uint8_t numRays = static_cast<uint8_t> (params->m_channel[0][0].size ());

  // compute the doppler term
  // NOTE the update of Doppler is simplified by only taking the center angle of
  // each cluster in to consideration.
  double slotTime = Simulator::Now ().GetSeconds ();
  PhasedArrayModel::ComplexVector doppler;
  for (uint8_t cIndex = 0; cIndex < numRays; cIndex++)
    {
      //cluster angle angle[direction][n],where, direction = 0(aoa), 1(zoa).
      double temp_doppler = 2 * M_PI * ((sin (params->m_angle[MatrixBasedChannelModel::ZOA_INDEX][cIndex]) * cos (params->m_angle[MatrixBasedChannelModel::AOA_INDEX][cIndex]) * uSpeed.x
                                         + sin (params->m_angle[MatrixBasedChannelModel::ZOA_INDEX][cIndex]) * sin (params->m_angle[MatrixBasedChannelModel::AOA_INDEX][cIndex] ) * uSpeed.y
                                         + cos (params->m_angle[MatrixBasedChannelModel::ZOA_INDEX][cIndex] ) * uSpeed.z)
                                         + (sin (params->m_angle[MatrixBasedChannelModel::ZOD_INDEX][cIndex] ) * cos (params->m_angle[MatrixBasedChannelModel::AOD_INDEX][cIndex]) * sSpeed.x
                                         + sin (params->m_angle[MatrixBasedChannelModel::ZOD_INDEX][cIndex] ) * sin (params->m_angle[MatrixBasedChannelModel::AOD_INDEX][cIndex] ) * sSpeed.y
                                         + cos (params->m_angle[MatrixBasedChannelModel::ZOD_INDEX][cIndex] ) * sSpeed.z))
                                        * slotTime * GetFrequency () / 3e8;
      doppler.push_back (exp (std::complex<double> (0, temp_doppler)));
    }

  // apply the doppler term and the propagation delay to the long term component
  // to obtain the beamforming gain
  auto vit = tempPsd->ValuesBegin (); // psd iterator
  auto sbit = tempPsd->ConstBandsBegin(); // band iterator
  while (vit != tempPsd->ValuesEnd ())
    {
      std::complex<double> subsbandGain (0.0,0.0);
      if ((*vit) != 0.00)
        {
          double fsb = (*sbit).fc; // center frequency of the sub-band
          for (uint8_t cIndex = 0; cIndex < numRays; cIndex++)
            {
              double delay = -2 * M_PI * fsb * (params->m_delay[cIndex]) * 1e-9;
              subsbandGain = subsbandGain + longTerm[cIndex] * doppler[cIndex] * exp (std::complex<double> (0, delay));
            }
          *vit = (*vit) * (norm (subsbandGain));
        }
      vit++;
      sbit++;
    }
  return tempPsd;
}

PhasedArrayModel::ComplexVector
NYUSpectrumPropagationLossModel::GetLongTerm (uint32_t aId, uint32_t bId,
                                                Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix,
                                                const PhasedArrayModel::ComplexVector &aW,
                                                const PhasedArrayModel::ComplexVector &bW) const
{
  PhasedArrayModel::ComplexVector longTerm; // vector containing the long term component for each cluster

  // check if the channel matrix was generated considering a as the s-node and
  // b as the u-node or viceversa
  PhasedArrayModel::ComplexVector sW, uW;
  if (!channelMatrix->IsReverse (aId, bId))
  {
    sW = aW;
    uW = bW;
  }
  else
  {
    sW = bW;
    uW = aW;
  }

  // compute the long term key, the key is unique for each tx-rx pair
  uint32_t x1 = std::min (aId, bId);
  uint32_t x2 = std::max (aId, bId);
  uint32_t longTermId = MatrixBasedChannelModel::GetKey (x1, x2);

  bool update = false; // indicates whether the long term has to be updated
  bool notFound = false; // indicates if the long term has not been computed yet

  // look for the long term in the map and check if it is valid
  if (m_longTermMap.find (longTermId) != m_longTermMap.end ())
  {
    NS_LOG_DEBUG ("found the long term component in the map");
    longTerm = m_longTermMap[longTermId]->m_longTerm;

    // check if the channel matrix has been updated
    // or the s beam has been changed
    // or the u beam has been changed
    update = (m_longTermMap[longTermId]->m_channel->m_generatedTime != channelMatrix->m_generatedTime
              || m_longTermMap[longTermId]->m_sW != sW
              || m_longTermMap[longTermId]->m_uW != uW);

  }
  else
  {
    NS_LOG_DEBUG ("long term component NOT found");
    notFound = true;
  }

  if (update || notFound)
    {
      NS_LOG_DEBUG ("compute the long term");
      // compute the long term component
      longTerm = CalcLongTerm (channelMatrix, sW, uW);

      // store the long term
      Ptr<LongTerm> longTermItem = Create<LongTerm> ();
      longTermItem->m_longTerm = longTerm;
      longTermItem->m_channel = channelMatrix;
      longTermItem->m_sW = sW;
      longTermItem->m_uW = uW;

      m_longTermMap[longTermId] = longTermItem;
    }

  return longTerm;
}

Ptr<SpectrumValue>
NYUSpectrumPropagationLossModel::DoCalcRxPowerSpectralDensity (Ptr<const SpectrumValue> txPsd,
                                                                Ptr<const MobilityModel> a,
                                                                Ptr<const MobilityModel> b) const
{
  NS_LOG_FUNCTION (this);
  uint32_t aId = a->GetObject<Node> ()->GetId (); // id of the node a
  uint32_t bId = b->GetObject<Node> ()->GetId (); // id of the node b

  NS_ASSERT (aId != bId);
  NS_ASSERT_MSG (a->GetDistanceFrom (b) > 0.0, "The position of a and b devices cannot be the same");

  Ptr<SpectrumValue> rxPsd = Copy<SpectrumValue> (txPsd);

  // retrieve the antenna of device a
  NS_ASSERT_MSG (m_deviceAntennaMap.find (aId) != m_deviceAntennaMap.end (), "Antenna not found for node " << aId);
  Ptr<const PhasedArrayModel> aAntenna = m_deviceAntennaMap.at (aId);
  NS_LOG_DEBUG ("a node " << a->GetObject<Node> () << " antenna " << aAntenna);

  // retrieve the antenna of the device b
  NS_ASSERT_MSG (m_deviceAntennaMap.find (bId) != m_deviceAntennaMap.end (), "Antenna not found for device " << bId);
  Ptr<const PhasedArrayModel> bAntenna = m_deviceAntennaMap.at (bId);
  NS_LOG_DEBUG ("b node " << bId << " antenna " << bAntenna);

  Ptr<const MatrixBasedChannelModel::ChannelMatrix> channelMatrix = m_channelModel->GetChannel (a, b, aAntenna, bAntenna);

  // get the precoding and combining vectors
  PhasedArrayModel::ComplexVector aW = aAntenna->GetBeamformingVector ();
  PhasedArrayModel::ComplexVector bW = bAntenna->GetBeamformingVector ();

  // retrieve the long term component
  PhasedArrayModel::ComplexVector longTerm = GetLongTerm (aId, bId, channelMatrix, aW, bW);

  // apply the beamforming gain
  rxPsd = CalcBeamformingGain (rxPsd, longTerm, channelMatrix, a->GetVelocity (), b->GetVelocity ());

  return rxPsd;
}


}  // namespace ns3
