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

#include "nyu-propagation-loss-model.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/channel-condition-model.h"
#include "ns3/double.h"
#include "ns3/boolean.h"
#include "ns3/pointer.h"
#include <cmath>
#include "ns3/node.h"
#include "ns3/simulator.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NYUPropagationLossModel");

static const double M_C = 3.0e8; // propagation velocity in free space
static const double ref_dist = 1; // in meters

// ------------------------------------------------------------------------- //

NS_OBJECT_ENSURE_REGISTERED (NYUPropagationLossModel);

TypeId
NYUPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NYUPropagationLossModel")
    .SetParent<PropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddAttribute ("Frequency", "The centre frequency in Hz.",
                   DoubleValue (500.0e6),
                   MakeDoubleAccessor (&NYUPropagationLossModel::SetFrequency,
                                       &NYUPropagationLossModel::GetFrequency),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("FoliageLoss", "The Foilage Loss in dB",
				   DoubleValue (0.4),
				   MakeDoubleAccessor (&NYUPropagationLossModel::SetFoliageloss,
								       &NYUPropagationLossModel::GetFoliageloss),
				   MakeDoubleChecker<double> ())  
    .AddAttribute ("ShadowingEnabled", "Enable/disable shadowing.",
                   BooleanValue (true),
                   MakeBooleanAccessor (&NYUPropagationLossModel::m_shadowingEnabled),
                   MakeBooleanChecker ())
    .AddAttribute ("O2ILosstype", "Outdoor to indoor (O2I) penetration loss type - Low Loss / High Loss.",
				   StringValue("Low Loss"),
				   MakeStringAccessor (&NYUPropagationLossModel::SetO2Ilosstype,
								       &NYUPropagationLossModel::GetO2Ilosstype),
				   MakeStringChecker())
    .AddAttribute ("FoliageLossEnabled", "Enable/disable foilage loss.",
				   BooleanValue (false),
				   MakeBooleanAccessor (&NYUPropagationLossModel::m_foilagelossEnabled),
				   MakeBooleanChecker ()) 
    .AddAttribute ("ChannelConditionModel", "Pointer to the channel condition model.",
                   PointerValue (),
                   MakePointerAccessor (&NYUPropagationLossModel::SetChannelConditionModel,
                                        &NYUPropagationLossModel::GetChannelConditionModel),
                   MakePointerChecker<ChannelConditionModel> ())
  ;
  return tid;
}

NYUPropagationLossModel::NYUPropagationLossModel ()
  : PropagationLossModel ()
{
  NS_LOG_FUNCTION (this);

  // initialize the normal random variable
  m_normRandomVariable = CreateObject<NormalRandomVariable> ();
  m_normRandomVariable->SetAttribute ("Mean", DoubleValue (0));
  m_normRandomVariable->SetAttribute ("Variance", DoubleValue (1));
}

NYUPropagationLossModel::~NYUPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}

void
NYUPropagationLossModel::DoDispose ()
{
  m_channelConditionModel->Dispose ();
  m_channelConditionModel = nullptr;
  m_shadowingMap.clear ();
}

void
NYUPropagationLossModel::SetChannelConditionModel (Ptr<ChannelConditionModel> model)
{
  NS_LOG_FUNCTION (this);
  m_channelConditionModel = model;
}

Ptr<ChannelConditionModel>
NYUPropagationLossModel::GetChannelConditionModel () const
{
  NS_LOG_FUNCTION (this);
  return m_channelConditionModel;
}

void
NYUPropagationLossModel::SetFrequency (double f)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (f >= 500.0e6 && f <= 150.0e9, "Frequency should be between 0.5 and 100 GHz but is " << f);
  m_frequency = f;
}

double
NYUPropagationLossModel::GetFrequency () const
{
  NS_LOG_FUNCTION (this);
  return m_frequency;
}

void
NYUPropagationLossModel::SetFoliageloss (double foliageloss)
{
	NS_LOG_FUNCTION (this);
	NS_ASSERT_MSG (foliageloss >=0 && foliageloss<= 10, "foilage loss should be between 0 and 10 dB/m but is " << foliageloss);
	m_foliageloss = foliageloss;
}

double
NYUPropagationLossModel::GetFoliageloss (void) const
{
  	NS_LOG_FUNCTION (this);
  	return m_foliageloss;
}

void
NYUPropagationLossModel::SetO2Ilosstype(const std::string &value)
{
	NS_LOG_FUNCTION (this);
	NS_ASSERT_MSG (value != "Low Loss" || value != "High Loss", "O2ILossType should be Low Loss or High Loss but is " << value);
	m_O2Ilosstype = value;
}

std::string
NYUPropagationLossModel::GetO2Ilosstype () const
{
  	NS_LOG_FUNCTION (this);
  	return m_O2Ilosstype ;
}

double
NYUPropagationLossModel::DoCalcRxPower (double txPowerDbm,
                                        Ptr<MobilityModel> a,
                                        Ptr<MobilityModel> b) const
{
  NS_LOG_FUNCTION(this);

  // check if the model is initialized
  NS_ASSERT_MSG (m_frequency != 0.0, "First set the centre frequency");
 
  // retrieve the channel condition
  NS_ASSERT_MSG (m_channelConditionModel, "First set the channel condition model");
  Ptr<ChannelCondition> cond = m_channelConditionModel->GetChannelCondition (a, b);
 
  // compute the 2D distance between a and b
  double distance2d = Calculate2dDistance (a->GetPosition (), b->GetPosition ());
 
  // compute the 3D distance between a and b
  double distance3d = CalculateDistance (a->GetPosition (), b->GetPosition ());
 
  // compute hUT and hBS
  std::pair<double, double> heights = GetUtAndBsHeights (a->GetPosition ().z, b->GetPosition ().z);
  
  double rxPow = txPowerDbm;
  NS_LOG_DEBUG("Tx power in Dbm:" << txPowerDbm);
  double PL = 0;

  PL = GetLoss (cond, distance2d, distance3d, heights.first, heights.second);
  NS_LOG_DEBUG ("Path Loss: " << PL);

  if (m_shadowingEnabled)
  {
    PL += GetShadowing (a, b,cond->GetLosCondition ());
    NS_LOG_DEBUG ("PL with SF " << PL);
  }
  if (cond->GetO2iCondition() == ChannelCondition::O2I)
  {
    PL += GetO2ILoss (m_O2Ilosstype,m_frequency);
    NS_LOG_DEBUG ("PL with O2I " << PL);
  }
  if (m_foilagelossEnabled)
  {    
    PL += GetTotalFoliageLoss(distance2d);
    NS_LOG_DEBUG ("PL with Foliage Loss " << PL);
  }
  
  rxPow -= PL;
  
  // The recieved power with 0 dB gain at Tx and Rx.
  NS_LOG_DEBUG ("Rcvd Power : " << rxPow);
  return rxPow;
}

double
NYUPropagationLossModel::RxPower_threshold (double txPowerDbm,
                                          Ptr<MobilityModel> a,
                                          Ptr<MobilityModel> b) const
{
  // compute Received power threshold (dBm). Dynamic range indicates the maximum possible omnidirectional
  // path loss in dB which can be measuremed by the 2D T-R distance.
  
  // compute the 2D distance between a and b
  double distance2d = Calculate2dDistance (a->GetPosition (), b->GetPosition ());
  
  double rxPow_th = txPowerDbm - dynamic_range(distance2d);
  NS_LOG_DEBUG ("Rcvd Power threshold: " << rxPow_th);
  return rxPow_th;
}

double
NYUPropagationLossModel::GetLoss (Ptr<ChannelCondition> cond, double distance2d, double distance3d, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);
 
  double loss = 0;
  if (cond->GetLosCondition () == ChannelCondition::LosConditionValue::LOS)
    {
      loss = GetLossLos (distance2d, distance3d, hUt, hBs);
    }
  else if (cond->GetLosCondition () == ChannelCondition::LosConditionValue::NLOS)
    {
      loss = GetLossNlos (distance2d, distance3d, hUt, hBs);
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }
  return loss;
}

double
NYUPropagationLossModel::GetShadowing (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);

  double shadowingValue;

  // compute the channel key
  uint32_t key = GetKey (a, b);

  bool notFound = false; // indicates if the shadowing value has not been computed yet
  bool newCondition = false; // indicates if the channel condition has changed
  Vector newDistance; // the distance vector, that is not a distance but a difference
  auto it = m_shadowingMap.end (); // the shadowing map iterator
  if (m_shadowingMap.find (key) != m_shadowingMap.end ())
    {
      // found the shadowing value in the map
      it = m_shadowingMap.find (key);
      newDistance = GetVectorDifference (a, b);
      newCondition = (it->second.m_condition != cond); // true if the condition changed
    }
  else
    {
      notFound = true;

      // add a new entry in the map and update the iterator
      ShadowingMapItem newItem;
      it = m_shadowingMap.insert (it, std::make_pair (key, newItem));
    }

  if (notFound || newCondition)
    {
      // generate a new independent realization
      shadowingValue = m_normRandomVariable->GetValue () * GetShadowingStd (cond);
    }
  else
    {
      // compute a new correlated shadowing loss
      Vector2D displacement (newDistance.x - it->second.m_distance.x, newDistance.y - it->second.m_distance.y);
      double R = exp (-1 * displacement.GetLength () / GetShadowingCorrelationDistance (cond));
      shadowingValue =  R * it->second.m_shadowing + sqrt (1 - R * R) * m_normRandomVariable->GetValue () * GetShadowingStd (cond);
    }

  // update the entry in the map
  it->second.m_shadowing = shadowingValue;
  it->second.m_distance = newDistance; // Save the (0,0,0) vector in case it's the first time we are calculating this value
  it->second.m_condition = cond;

  return shadowingValue;
}

std::pair<double, double>
NYUPropagationLossModel::GetUtAndBsHeights (double za, double zb) const
{
  // The default implementation assumes that the tallest node is the BS and the
  // smallest is the UT.
  double hUt = std::min (za, zb);
  double hBs = std::max (za, zb);

  return std::pair<double, double> (hUt, hBs);
}

int64_t
NYUPropagationLossModel::DoAssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this);

  m_normRandomVariable->SetStream (stream);
  return 1;
}

double
NYUPropagationLossModel::Calculate2dDistance (Vector a, Vector b)
{
  double x = a.x - b.x;
  double y = a.y - b.y;
  double distance2D = sqrt (x * x + y * y);

  return distance2D;
}

uint32_t
NYUPropagationLossModel::GetKey (Ptr<MobilityModel> a, Ptr<MobilityModel> b)
{
  // use the nodes ids to obtain an unique key for the channel between a and b
  // sort the nodes ids so that the key is reciprocal
  uint32_t x1 = std::min (a->GetObject<Node> ()->GetId (), b->GetObject<Node> ()->GetId ());
  uint32_t x2 = std::max (a->GetObject<Node> ()->GetId (), b->GetObject<Node> ()->GetId ());

  // use the cantor function to obtain the key
  uint32_t key = (((x1 + x2) * (x1 + x2 + 1)) / 2) + x2;

  return key;
}

Vector
NYUPropagationLossModel::GetVectorDifference (Ptr<MobilityModel> a, Ptr<MobilityModel> b)
{
  uint32_t x1 = a->GetObject<Node> ()->GetId ();
  uint32_t x2 = b->GetObject<Node> ()->GetId ();

  if (x1 < x2)
    {
      return b->GetPosition () - a->GetPosition ();
    }
  else
    {
      return a->GetPosition () - b->GetPosition ();
    }
}

double
NYUPropagationLossModel::calPar(double ple1, double ple2, double frequency) const
{
  NS_LOG_FUNCTION(this << ple1 << ple2 << frequency);
  frequency = frequency/1e9; // freq in GHz
  double output = 0;
  if (frequency < 28)
  {
    output = ple1;
  }
  else if (frequency > 150)
  {
    output = ple2;
  }
  else
  {
    output = frequency * (ple2 - ple1) / (140 - 28) + (5 *ple1 - ple2)/4;
  }
  return output;
}

// Dyamic range of NYU Channel Sounder
double
NYUPropagationLossModel::dynamic_range(double distance2D) const
{
  // distance is in meters and dynamic_range is in dB
  double dynamic_range = 0;
  if (distance2D <= 500)
  {
    dynamic_range = 190;
  }
  else
  {
    dynamic_range = 220;
  }
  return dynamic_range;
}

double
NYUPropagationLossModel::GetO2ILoss (const std::string &o2ilosstype, double frequency) const
{
  double o2iloss = 0;
  if(o2ilosstype.compare("Low Loss") == 0)
  {
    o2iloss = 10*log10(5 + 0.03*pow(frequency/1e9,2)) + 4*m_normRandomVariable->GetValue();
  }
  else if (o2ilosstype.compare("High Loss") == 0)
  {
    o2iloss = 10*log10(10 + 5*pow(frequency/1e9,2)) + 6*m_normRandomVariable->GetValue();
  }
  else
  {
    NS_FATAL_ERROR ("Unknown O2I Loss Type");
  }
  NS_LOG_DEBUG("O2I Loss Type: "<< o2ilosstype << " O2I Loss Value: " << o2iloss);
  return o2iloss;
}

double 
NYUPropagationLossModel::GetTotalFoliageLoss (double distance2d) const
{
  double TotalFoliageLoss = 0;
  TotalFoliageLoss = m_foliageloss * m_uniformVar->GetValue(0,distance2d);
  NS_LOG_DEBUG("Total FoliageLoss: "<< TotalFoliageLoss);
  return TotalFoliageLoss;
}

// ------------------------------------------------------------------------- //
 
NS_OBJECT_ENSURE_REGISTERED (NYUUMiPropagationLossModel);

TypeId
NYUUMiPropagationLossModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NYUUMiPropagationLossModel")
    .SetParent<NYUPropagationLossModel> ()
    .SetGroupName ("Propagation")
    .AddConstructor<NYUUMiPropagationLossModel> ()
  ;
  return tid;
}
NYUUMiPropagationLossModel::NYUUMiPropagationLossModel ()
  : NYUPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
 
  // set a default channel condition model
  m_channelConditionModel = CreateObject<NYUUMiChannelConditionModel> ();
}
NYUUMiPropagationLossModel::~NYUUMiPropagationLossModel ()
{
  NS_LOG_FUNCTION (this);
}
double
NYUUMiPropagationLossModel::GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);

  double lambda; // wavelength in meters
  double PLref; // Free Space Path Loss (FSPL)
  double PL_LOS = 0; // Path loss without SF (dB)
  double Ple = calPar(2,2,m_frequency); //Path Loss Exponent UMi LOS
  
  lambda = M_C/(m_frequency);

  PLref = 20*log10(4*M_PI*ref_dist/lambda); 

  PL_LOS = PLref + 10 * Ple * log10 (distance2D);
  
  NS_LOG_DEBUG ("m_frequency: "<<m_frequency << " 2d-distance: "<<distance2D << " labmda: " << lambda << " FSPL: " <<PLref << " PL_LOS: " << PL_LOS);
  return PL_LOS;
}

double
NYUUMiPropagationLossModel::GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const
{
  NS_LOG_FUNCTION (this);

  double lambda; // wavelength in meters
  double PLref; // Free Space Path Loss (FSPL)
  double PL_NLOS = 0; // Path loss without SF (dB)
  double Ple = calPar(3.2,3.2,m_frequency); //Path Loss Exponent UMi NLOS
  
  // once 140 GHz channel model also incorporated we need to perform a calibration to fetch the correct PLE
  // Ple = calpar(val1,val2,m_frequency); 

  lambda = M_C/(m_frequency);

  PLref = 20*log10(4*M_PI*ref_dist/lambda); 

  PL_NLOS = PLref + 10 * Ple * log10 (distance2D);
  
  NS_LOG_DEBUG ("m_frequency: "<<m_frequency << " 2d-distance: "<<distance2D << " labmda: " << lambda << " FSPL: " <<PLref << " PL_NLOS: " << PL_NLOS);

  return PL_NLOS;
}

double
NYUUMiPropagationLossModel::GetShadowingStd (ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  double shadowingStd;
  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      shadowingStd = calPar(4.0,4.0,m_frequency);
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      shadowingStd = calPar(7.0,7.0,m_frequency);
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }
  
  NS_LOG_DEBUG ("shadowingStd " << shadowingStd);
  return shadowingStd;
}

double
NYUUMiPropagationLossModel::GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const
{
  NS_LOG_FUNCTION (this);
  double correlationDistance;

  // See 3GPP TR 38.901, Table 7.5-6
  if (cond == ChannelCondition::LosConditionValue::LOS)
    {
      correlationDistance = 10;
    }
  else if (cond == ChannelCondition::LosConditionValue::NLOS)
    {
      correlationDistance = 13;
    }
  else
    {
      NS_FATAL_ERROR ("Unknown channel condition");
    }

  return correlationDistance;
}

}