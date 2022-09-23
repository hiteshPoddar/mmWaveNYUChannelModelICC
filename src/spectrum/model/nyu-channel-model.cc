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


#include "nyu-channel-model.h"
#include "ns3/log.h"
#include "ns3/phased-array-model.h"
#include "ns3/node.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/integer.h"
#include <algorithm>
#include <random>
#include "ns3/log.h"
#include <ns3/simulator.h>
#include "ns3/mobility-model.h"
#include "ns3/pointer.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("NYUChannelModel");

NS_OBJECT_ENSURE_REGISTERED (NYUChannelModel);

NYUChannelModel::NYUChannelModel ()
{
  NS_LOG_FUNCTION (this);
  m_uniformRv = CreateObject<UniformRandomVariable> ();

  m_normalRv = CreateObject<NormalRandomVariable> ();
  m_normalRv->SetAttribute ("Mean", DoubleValue (0.0));
  m_normalRv->SetAttribute ("Variance", DoubleValue (1.0));

  m_expRv  = CreateObject<ExponentialRandomVariable> ();
  
}

NYUChannelModel::~NYUChannelModel ()
{
  NS_LOG_FUNCTION (this);
}

void
NYUChannelModel::DoDispose ()
{
  NS_LOG_FUNCTION (this);
  m_channelMap.clear ();
  if (m_channelConditionModel)
    {
      m_channelConditionModel->Dispose ();
    }
  m_channelConditionModel = nullptr;
}

TypeId
NYUChannelModel::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::NYUChannelModel")
    .SetParent<Object> ()
    .SetGroupName ("Spectrum")
    .SetParent<MatrixBasedChannelModel> ()
    .AddConstructor<NYUChannelModel> ()
    .AddAttribute ("Frequency",
                   "The operating Frequency in Hz",
                   DoubleValue (500.0e6),
                   MakeDoubleAccessor (&NYUChannelModel::SetFrequency,
                                       &NYUChannelModel::GetFrequency),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RFBandwidth",
                   "The Bandwidth in Hz",
                   DoubleValue (18e6),
                   MakeDoubleAccessor (&NYUChannelModel::SetRFBandWidth,
                                       &NYUChannelModel::GetRFBandWidth),
                   MakeDoubleChecker<double> ()) 
    .AddAttribute ("Scenario",
                   "The NYU scenario (RMa, UMa, UMi, InH, InF)",
                   StringValue ("UMi"),
                   MakeStringAccessor (&NYUChannelModel::SetScenario,
                                       &NYUChannelModel::GetScenario),
                   MakeStringChecker ())
    .AddAttribute ("ChannelConditionModel",
                   "Pointer to the channel condition model",
                   PointerValue (),
                   MakePointerAccessor (&NYUChannelModel::SetChannelConditionModel,
                                        &NYUChannelModel::GetChannelConditionModel),
                   MakePointerChecker<ChannelConditionModel> ())
    .AddAttribute ("UpdatePeriod",
                   "Specify the channel coherence time",
                   TimeValue (MilliSeconds (0)),
                   MakeTimeAccessor (&NYUChannelModel::m_updatePeriod),
                   MakeTimeChecker ())
    // attributes for the blockage model
    .AddAttribute ("Blockage",
                   "Enable NYU blockage model",
                   BooleanValue (false),
                   MakeBooleanAccessor (&NYUChannelModel::m_blockage),
                   MakeBooleanChecker ())
  ;
  return tid;
}

void
NYUChannelModel::SetChannelConditionModel (Ptr<ChannelConditionModel> model)
{
  NS_LOG_FUNCTION (this);
  m_channelConditionModel = model;
}

Ptr<ChannelConditionModel>
NYUChannelModel::GetChannelConditionModel () const
{
  NS_LOG_FUNCTION (this);
  return m_channelConditionModel;
}

void
NYUChannelModel::SetFrequency (double f)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (f >= 500.0e6 && f <= 150.0e9, "Frequency should be between 0.5 and 100 GHz but is " << f);
  m_frequency = f;
}

double
NYUChannelModel::GetFrequency () const
{
  NS_LOG_FUNCTION (this);
  return m_frequency;
}

void
NYUChannelModel::SetRFBandWidth (double rf_bw)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (rf_bw >= 0 && rf_bw <= 1000e6, "Bandwidth should be between 0 and 1000 MHz GHz but is " << rf_bw);
  m_rfbw = rf_bw;
}

double
NYUChannelModel::GetRFBandWidth () const
{
  NS_LOG_FUNCTION (this);
  return m_rfbw;
}

void
NYUChannelModel::SetScenario (const std::string &scenario)
{
  NS_LOG_FUNCTION (this);
  NS_ASSERT_MSG (scenario == "RMa" || scenario == "UMa" || scenario == "UMi"
                 || scenario == "InH" || scenario == "InF",
                 "Unknown scenario, choose between RMa, UMa, UMi,"
                 "InH, InF");
  m_scenario = scenario;
}

std::string
NYUChannelModel::GetScenario () const
{
  NS_LOG_FUNCTION (this);
  return m_scenario;
}

/* API which does a linear interpolation of channel parameters between 0.5 GHz - 150 GHz */
double
NYUChannelModel::calpar(double val1, double val2, double frequency) const
{
  NS_LOG_FUNCTION (this << val1 << val2 << frequency);
  double output = 0;
  if(frequency < 28)
  {
    output = val1;
  }
  else if (frequency > 150)
  {
    output = val2;
  }
  else
  {
    output = frequency*(val2-val1)/(140-28)+(5*val1-val2)/4;
  }
  NS_LOG_DEBUG("Interpolation Value:" << output << std::endl);
  return output;
}

Ptr<const NYUChannelModel::ParamsTable>
NYUChannelModel::GetNYUTable (Ptr<const ChannelCondition> channelCondition) const
{
  NS_LOG_FUNCTION (this);

  // Frequency in GHz
  double freq = m_frequency / 1e9;

  Ptr<ParamsTable> tablenyu = Create<ParamsTable> ();

  bool los = channelCondition->IsLos ();
  NS_LOG_DEBUG("Channel Condition is LOS: " << los << " Frequency:" << freq << " Bandwidth:" << m_rfbw << " Scenario:" << m_scenario);

  // XPD Values generated from NYU Channel Model is not based on scenario.
  if (los)
  {
    tablenyu->XPD_Mean = 11.5 + (freq) * 0.10; // frequency dependent XPD Mean value
    tablenyu->XPD_Sd = 1.6; // XPD standard deviation
  }
  else
  {
    tablenyu->XPD_Mean = 5.5 + (freq) * 0.13; // frequency dependent XPD Mean value
    tablenyu->XPD_Sd = 1.6; // XPD standard deviation  
  }

  if (m_scenario == "UMi" && los)
    {
        /* once 140 outdoor is implemented tablenyu->mu_AOD = calpar(1.9,v2,fcGHz) */
        /* Currently values used are for 28-73 GHz */
        tablenyu->max_c = 6;
        tablenyu->max_s = 30;
        tablenyu->mu_AOD = 1.9; // number of AOD spatial Lobes
        tablenyu->mu_AOA = 1.8; // number of AOA spatial Lobes
        tablenyu->mu_rho = 0.2; // in ns
        tablenyu->mu_tau = 123; // in ns
        tablenyu->minVoidInterval = 25;//in ns
        tablenyu->sigmaCluster = 1; // in dB
        tablenyu->tc_gamma = 25.9; // in ns
        tablenyu->sigmaSubpath = 6; // in dB
        tablenyu->sp_gamma = 16.9; // in ns
        tablenyu->mean_ZOD = -12.6; // in degree
        tablenyu->sigma_ZOD = 5.9; // in degree
        tablenyu->std_AOD_RMSLobeAzimuthSpread = 8.5; // degree
        tablenyu->AOD_RMSLobeAzimuthSpread = "Gaussian";
        tablenyu->std_AOD_RMSLobeElevationSpread = 2.5; // degree
        tablenyu->AOD_RMSLobeElevationSpread = "Gaussian";
        tablenyu->mean_ZOA = 10.8; // in degree
        tablenyu->sigma_ZOA = 5.3; // in degree
        tablenyu->std_AOA_RMSLobeAzimuthSpread = 10.5; // in degree
        tablenyu->AOA_RMSLobeAzimuthSpread = "Gaussian";
        tablenyu->std_AOA_RMSLobeElevationSpread = 11.5; // in degree
        tablenyu->AOA_RMSLobeElevationSpread = "Laplacian";
        tablenyu->los = true;// Flag indicating LOS/NLOS. true implies LOS and false implies NLOS
        tablenyu->distributionType_AOA = "Laplacian";// distribution type -- to be removed
        tablenyu->distributionType_AOD = "Gaussian";// distribution type -- to be removed - apis need to be updated
    }
  else if (m_scenario == "UMi" && !los)
    {
        tablenyu->max_c = 6;
        tablenyu->max_s = 30;
        tablenyu->mu_AOD = 1.5; // number of AOD spatial Lobes
        tablenyu->mu_AOA = 2.1; // number of AOA spatial Lobes
        tablenyu->mu_rho = 0.5; // in ns
        tablenyu->mu_tau = 83; // in ns
        tablenyu->minVoidInterval = 25;//in ns
        tablenyu->sigmaCluster = 3; // in dB
        tablenyu->tc_gamma = 51; // in ns
        tablenyu->sigmaSubpath = 6; // in dB
        tablenyu->sp_gamma = 15.5; // in ns
        tablenyu->mean_ZOD = -4.9; // in degree
        tablenyu->sigma_ZOD = 4.5; // in degree
        tablenyu->std_AOD_RMSLobeAzimuthSpread = 11.0; // degree
        tablenyu->AOD_RMSLobeAzimuthSpread = "Gaussian";
        tablenyu->std_AOD_RMSLobeElevationSpread = 3.0; // degree
        tablenyu->AOD_RMSLobeElevationSpread = "Gaussian";
        tablenyu->mean_ZOA = 3.6; // in degree
        tablenyu->sigma_ZOA = 4.8; // in degree
        tablenyu->std_AOA_RMSLobeAzimuthSpread = 7.5; // in degree
        tablenyu->AOA_RMSLobeAzimuthSpread = "Gaussian";
        tablenyu->std_AOA_RMSLobeElevationSpread = 6.0; // in degree
        tablenyu->AOA_RMSLobeElevationSpread = "Laplacian";
        tablenyu->los = false; // Flag indicating LOS/NLOS. LOS->1, NLOS->0

        tablenyu->distributionType_AOD = "Gaussian"; // distribution type
        tablenyu->distributionType_AOA = "Laplacian"; // distribution type  
    }
    else
    {
        NS_FATAL_ERROR ("Unknown channel condition");
    }

  return tablenyu;
}

bool
NYUChannelModel::ChannelMatrixNeedsUpdate (Ptr<const NYUChannelMatrix> channelMatrix, Ptr<const ChannelCondition> channelCondition) const
{
  NS_LOG_FUNCTION (this);

  bool update = false;

  // if the channel condition is different the channel has to be updated
  if (!channelMatrix->m_channelCondition->IsEqual (channelCondition))
    {
      NS_LOG_DEBUG ("Update the channel condition");
      update = true;
    }

  // if the coherence time is over the channel has to be updated
  if (!m_updatePeriod.IsZero () && Simulator::Now () - channelMatrix->m_generatedTime > m_updatePeriod)
    {
      NS_LOG_DEBUG ("Generation time " << channelMatrix->m_generatedTime.As (Time::NS) << " now " << Now ().As (Time::NS));
      update = true;
    }

  return update;
}

Ptr<const MatrixBasedChannelModel::ChannelMatrix>
NYUChannelModel::GetChannel (Ptr<const MobilityModel> aMob,
                            Ptr<const MobilityModel> bMob,
                            Ptr<const PhasedArrayModel> aAntenna,
                            Ptr<const PhasedArrayModel> bAntenna)
{
  NS_LOG_FUNCTION (this);

  // Compute the channel key. The key is reciprocal, i.e., key (a, b) = key (b, a)
  uint32_t x1 = std::min (aMob->GetObject<Node> ()->GetId (), bMob->GetObject<Node> ()->GetId ());
  uint32_t x2 = std::max (aMob->GetObject<Node> ()->GetId (), bMob->GetObject<Node> ()->GetId ());
  uint32_t channelId = GetKey (x1, x2);

  // retrieve the channel condition
  Ptr<const ChannelCondition> condition = m_channelConditionModel->GetChannelCondition (aMob, bMob);

  // Check if the channel is present in the map and return it, otherwise
  // generate a new channel
  bool update = false;
  bool notFound = false;
  Ptr<NYUChannelMatrix> channelMatrix;
  if (m_channelMap.find (channelId) != m_channelMap.end ())
    {
      // channel matrix present in the map
      NS_LOG_DEBUG ("channel matrix present in the map");
      channelMatrix = m_channelMap[channelId];

      // check if it has to be updated
      update = ChannelMatrixNeedsUpdate (channelMatrix, condition);
    }
  else
    {
      NS_LOG_DEBUG ("channel matrix not found");
      notFound = true;
    }

  // If the channel is not present in the map or if it has to be updated
  // generate a new realization
  if (notFound || update)
    {
      // channel matrix not found or has to be updated, generate a new one
      Angles txAngle (bMob->GetPosition (), aMob->GetPosition ());
      Angles rxAngle (aMob->GetPosition (), bMob->GetPosition ());

      double x = aMob->GetPosition ().x - bMob->GetPosition ().x;
      double y = aMob->GetPosition ().y - bMob->GetPosition ().y;
      double distance2D = sqrt (x * x + y * y);

      channelMatrix = GetNewChannel (condition, aAntenna, bAntenna, rxAngle, txAngle, distance2D);
      channelMatrix->m_nodeIds = std::make_pair (aMob->GetObject<Node> ()->GetId (), bMob->GetObject<Node> ()->GetId ());

      // store or replace the channel matrix in the channel map
      m_channelMap[channelId] = channelMatrix;
    }

  return channelMatrix;
}

Ptr<NYUChannelModel::NYUChannelMatrix>
NYUChannelModel::GetNewChannel (Ptr<const ChannelCondition> channelCondition,
                                Ptr<const PhasedArrayModel> sAntenna,
                                Ptr<const PhasedArrayModel> uAntenna,
                                Angles &uAngle, Angles &sAngle,
                                double dis2D) const
{
  NS_LOG_FUNCTION (this);

  double distance2D = dis2D;
  double pwrthreshold = dynamic_range(distance2D);
  // get the nyu parameters
  Ptr<const ParamsTable> tablenyu = GetNYUTable (channelCondition);

  // create a channel matrix instance
  Ptr<NYUChannelMatrix> channelParams = Create<NYUChannelMatrix> ();
  channelParams->m_channelCondition = channelCondition; // set the channel condition
  channelParams->m_generatedTime = Simulator::Now ();

  // Generate channel params and matrix in this API
  // Step 1: Generate number of time clusters N, spatial AOD Lobes and spatial AOA Lobes, and subpaths in each time cluster
    channelParams->numTC = GetNumTC(tablenyu->lambda_c);
    channelParams->numAODLobes = GetNumAODLobes(tablenyu->mu_AOD);
    channelParams->numAOALobes = GetNumAOALobes(tablenyu->mu_AOA);
    channelParams->numSPinTC = GetNumSPinTC(channelParams->numTC);

    // Step 2: Generate the intra-cluster subpath delays i.e. Delay of each Subpath within a Time Cluster {rho_mn (ns)}
    channelParams->SPdelayinTC = GetIntraClusterDelays(channelParams->numSPinTC,tablenyu->mu_rho);
    
    // Step 3: Generate the phases (rad) for each Supath in a time cluster. 4 phases are generated for each Subpath one for each
    // polarization. Rows represent subpaths and col1,col2,col3,col4 represent the polarizations
    channelParams->SPPhases = GetSubpathPhases(channelParams->numSPinTC);

    // Step 4: Generate the cluster excess time delays tau_n (ns)
    channelParams->TCExcessDelay = GetClusterExcessTimeDelays(tablenyu->mu_tau,channelParams->SPdelayinTC,tablenyu->minVoidInterval);

    // Step 5: Generate temporal cluster powers (mW)
    channelParams->TCPowers = GetClusterPowers(channelParams->TCExcessDelay,tablenyu->sigmaCluster,tablenyu->tc_gamma);

    // Step 6: Generate the cluster subpath powers (mW)
    channelParams->SPPowers = GetSubPathPowers(channelParams->SPdelayinTC,channelParams->TCPowers,tablenyu->sigmaSubpath,tablenyu->sp_gamma,
                                               tablenyu->los);
    
    // step 7: Recover absolute propagation times t_mn (ns) of each subpath component in a time cluster
    channelParams->absoluteSPdelayinTC = getAbsolutePropTimes(distance2D,channelParams->TCExcessDelay,channelParams->SPdelayinTC);

    // Step 8: Recover AODs and AOAs of the multipath components
    channelParams->AOD_cluster_subpath_lobe_az_elev_angles = GetSubpathMappingandAngles(channelParams->numAODLobes,
                                                                                        channelParams->numSPinTC,
                                                                                        tablenyu->mean_ZOD,
                                                                                        tablenyu->sigma_ZOD,
                                                                                        tablenyu->std_AOD_RMSLobeAzimuthSpread, 
                                                                                        tablenyu->std_AOD_RMSLobeElevationSpread, 
                                                                                        tablenyu->distributionType_AOD);

    channelParams->AOA_cluster_subpath_lobe_az_elev_angles = GetSubpathMappingandAngles(channelParams->numAOALobes,
                                                                                        channelParams->numSPinTC,
                                                                                        tablenyu->mean_ZOA,
                                                                                        tablenyu->sigma_ZOA,
                                                                                        tablenyu->std_AOA_RMSLobeAzimuthSpread, 
                                                                                        tablenyu->std_AOA_RMSLobeElevationSpread, 
                                                                                        tablenyu->distributionType_AOA); 
    // Step 9: Construct the multipath parameters (AOA,ZOD,AOA,ZOA)
    channelParams->PowerSpectrumOld = GetPowerSpectrum(channelParams->numSPinTC,
                                                      channelParams->absoluteSPdelayinTC,
                                                      channelParams->SPPowers,
                                                      channelParams->SPPhases,
                                                      channelParams->AOD_cluster_subpath_lobe_az_elev_angles,
                                                      channelParams->AOA_cluster_subpath_lobe_az_elev_angles);
    
    // Step 10: Adjust the multipath parameters (AOA,ZOD,AOA,ZOA) based on LOS/NLOS and
    // combine the Subpaths which cannot be resolved.
    channelParams->PowerSpectrum = GetBWAdjustedtedPowerSpectrum(channelParams->PowerSpectrumOld,m_rfbw,tablenyu->los);

    // All subpaths whose power is above threshold is considered. The threshold is defined as Max power of the subpath - 30 dB.
    channelParams->PowerSpectrum = GetValidSubapths(channelParams->PowerSpectrum,pwrthreshold);

    // Step 11: Generate XPD values for each ray in PowerSpectrum
    channelParams->XPD = GetXPDperRay(channelParams->PowerSpectrum.size(),tablenyu->XPD_Mean,tablenyu->XPD_Sd);

    // The AOD,ZOD,AOA,ZOA generated by NYU channel model is in degrees and the cordinate system used
    // is phi w.r.t to y axis and theta w.r.t xy plane. This is different when compared to the GCS where
    // phi is w.r.t to x axis and theta is w.r.t z axis. This API converts NYU cordinate system to GCS
    // and also saves the angles in radians. So AOD,ZOD,AOA,ZOA are saved in radians. m_angle is inherited from
    // matrix-based-channel-model and is used in CalcBeamformingGain()
    channelParams->m_angle = NYUCStoGCS(channelParams->PowerSpectrum);

    channelParams->rayAodRadian.resize(channelParams->PowerSpectrum.size());
    channelParams->rayZodRadian.resize(channelParams->PowerSpectrum.size());
    channelParams->rayAoaRadian.resize(channelParams->PowerSpectrum.size());
    channelParams->rayZoaRadian.resize(channelParams->PowerSpectrum.size());

    // debug m_angle array and store the AOD,ZOD,AOA,ZOA in the rayAod,rayZod,rayAoa and rayZoa vectors
    for (int i = 0; i < (int)channelParams->m_angle.size(); i++)
    {
      for (int j = 0; j < (int)channelParams->m_angle[i].size(); j++)
      {
        if (i == 0)
        {
          NS_LOG_DEBUG("m_angle sp id:" << j << " aoa:" << channelParams->m_angle[i][j]);
          channelParams->rayAoaRadian[j] = channelParams->m_angle[i][j];
        }
        else if (i == 1)
        {
          NS_LOG_DEBUG("m_angle sp id:" << j << " zoa:" << channelParams->m_angle[i][j]);
          channelParams->rayZoaRadian[j]  = channelParams->m_angle[i][j];
        }
        else if (i == 2)
        {
          NS_LOG_DEBUG("m_angle sp id:" << j << " aod:" << channelParams->m_angle[i][j]);
          channelParams->rayAodRadian[j]  = channelParams->m_angle[i][j];
        }
        else if (i == 3)
        {
          NS_LOG_DEBUG("m_angle sp id:" << j << " zod:" << channelParams->m_angle[i][j]);
          channelParams->rayZodRadian[j] = channelParams->m_angle[i][j];
        }
      }
    }
    
    // Save the delay of SP in m_delay. This is used later in CalcBeamformingGain() api present in nyu-spectrum-propagation-loss-model.cc
    for (int i = 0; i < (int) channelParams->PowerSpectrum.size(); i++)
    {
      channelParams->m_delay.push_back(channelParams->PowerSpectrum[i][0]);
    }

    // debug ray delay stored in mdelay - same as PowerSpectrum[i][0]
    for (int i = 0; i < (int)channelParams->m_delay.size(); i++)
    {
      NS_LOG_DEBUG(" Subpath id:" << i << " delay:"<< channelParams->m_delay[i]);
    }

    // Stores the total number of subpaths after BW adjustment and excluding weak subpaths
    channelParams->totalSubpaths = channelParams->PowerSpectrum.size();
    
    NS_LOG_DEBUG("Total Number of SP is:" << channelParams->totalSubpaths);

  Complex3DVector H_usn; 

  uint64_t uSize = uAntenna->GetNumberOfElements ();
  uint64_t sSize = sAntenna->GetNumberOfElements ();

  H_usn.resize (uSize);
  for (uint64_t uIndex = 0; uIndex < uSize; uIndex++)
    {
      H_usn[uIndex].resize (sSize);
      for (uint64_t sIndex = 0; sIndex < sSize; sIndex++)
        {
          H_usn[uIndex][sIndex].resize (channelParams->totalSubpaths);
        }
    }

      // The following for loops computes the channel coefficients
  for (uint64_t uIndex = 0; uIndex < uSize; uIndex++)
    {
      Vector uLoc = uAntenna->GetElementLocation (uIndex);
      for (uint64_t sIndex = 0; sIndex < sSize; sIndex++)
        {
          Vector sLoc = sAntenna->GetElementLocation (sIndex);
          for (uint8_t nIndex = 0; nIndex < channelParams->totalSubpaths; nIndex++)
            {
              std::complex<double> rays (0,0);
              NS_LOG_DEBUG("Subpath id:" << nIndex << " tx ant:" << uIndex << " rx ant:" << sIndex);
              double rxPhaseDiff = 2 * M_PI * (sin (channelParams->rayZoaRadian[nIndex]) * cos (channelParams->rayAoaRadian[nIndex]) * uLoc.x
                                          + sin (channelParams->rayZoaRadian[nIndex]) * sin (channelParams->rayAoaRadian[nIndex]) * uLoc.y
                                          + cos (channelParams->rayZoaRadian[nIndex]) * uLoc.z);

              double txPhaseDiff = 2 * M_PI * (sin (channelParams->rayZodRadian[nIndex]) * cos (channelParams->rayAodRadian[nIndex]) * sLoc.x
                                          + sin (channelParams->rayZodRadian[nIndex]) * sin (channelParams->rayAodRadian[nIndex]) * sLoc.y
                                          + cos (channelParams->rayZodRadian[nIndex]) * sLoc.z);
              NS_LOG_DEBUG("rxPhaseDiff:" << rxPhaseDiff);
              NS_LOG_DEBUG("txPhaseDiff:" << txPhaseDiff);

              double rxFieldPatternPhi, rxFieldPatternTheta, txFieldPatternPhi, txFieldPatternTheta;
              std::tie (rxFieldPatternPhi, rxFieldPatternTheta) = uAntenna->GetElementFieldPattern (Angles (channelParams->rayAoaRadian[nIndex],channelParams->rayZoaRadian[nIndex]));
              std::tie (txFieldPatternPhi, txFieldPatternTheta) = sAntenna->GetElementFieldPattern (Angles (channelParams->rayAodRadian[nIndex],channelParams->rayZodRadian[nIndex]));
              NS_LOG_DEBUG("rxFieldPatternPhi:" << rxFieldPatternPhi << " rxFieldPatternTheta:" << rxFieldPatternTheta);
              NS_LOG_DEBUG("txFieldPatternPhi:" << txFieldPatternPhi << " txFieldPatternTheta:" << txFieldPatternTheta);

              rays = (std::complex<double> (cos (channelParams->SPPhases[nIndex][0]), sin (channelParams->SPPhases[nIndex][0])) * rxFieldPatternTheta * txFieldPatternTheta +
                std::complex<double> (cos (channelParams->SPPhases[nIndex][1]), sin (channelParams->SPPhases[nIndex][1])) * std::sqrt (1 / Getdb2pow(channelParams->XPD[nIndex][1])) * rxFieldPatternTheta * txFieldPatternPhi +
                std::complex<double> (cos (channelParams->SPPhases[nIndex][2]), sin (channelParams->SPPhases[nIndex][2])) * std::sqrt (1 / Getdb2pow(channelParams->XPD[nIndex][2])) * rxFieldPatternPhi * txFieldPatternTheta +
                std::complex<double> (cos (channelParams->SPPhases[nIndex][3]), sin (channelParams->SPPhases[nIndex][3])) * std::sqrt(1 / Getdb2pow(channelParams->XPD[nIndex][0])) * rxFieldPatternPhi * txFieldPatternPhi)
                * std::complex<double> (cos (rxPhaseDiff), sin (rxPhaseDiff))
                * std::complex<double> (cos (txPhaseDiff), sin (txPhaseDiff));          
              NS_LOG_DEBUG("RAY excluding amplitude:" << rays << " Power Spectrum Power value:" << channelParams->PowerSpectrum[nIndex][1]);
              rays *= sqrt(channelParams->PowerSpectrum[nIndex][1]);
              NS_LOG_DEBUG("Ray including ammplitude:" << rays);
              H_usn[uIndex][sIndex][nIndex] = rays;
            }
        }
    }

  NS_LOG_INFO ("size of coefficient matrix =[" << H_usn.size () << "][" << H_usn[0].size () << "][" << H_usn[0][0].size () << "]");
   for (auto& i:H_usn)
    {
      for (auto& j:i)
        {
          for (auto& k:j)
            {
              NS_LOG_DEBUG (" " << k << ",");
            }
        }
    }
  channelParams->m_channel = H_usn;
  
  return channelParams;
}

int64_t
NYUChannelModel::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_normalRv->SetStream (stream);
  m_uniformRv->SetStream (stream + 1);
  m_expRv->SetStream (stream + 2);
  return 3;
}

/********************************************************************************************/

/* Distribution APIs - Poisson, Discrete Uniform, Uniform  */

int 
NYUChannelModel::GetPoissionDist (double lambda) const
{
  NS_LOG_FUNCTION (this << lambda);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::poisson_distribution<int> distribution(lambda);
  double value = distribution (gen);
  NS_LOG_DEBUG(" Value in Pois Dist is:" << value);
  return value;
}

int 
NYUChannelModel::GetDUniformDist (const double min, const double max) const
{
  NS_LOG_FUNCTION (this << min << max);
  int value = m_uniformRv->GetInteger (min, max);
  NS_LOG_DEBUG(" Valus in Uniform Dist is:" << (double) value <<",min is:"<<min<<", max is:"<<max);
  return value;
}

double 
NYUChannelModel::GetUniformDist (const double min, const double max) const
{
  NS_LOG_FUNCTION (this << min << max);
  double value = m_uniformRv->GetValue (min, max);
  NS_LOG_DEBUG(" Valus in Uniform Dist is:" << (double) value <<",min is:"<<min<<", max is:"<<max);
  return value;
}

double 
NYUChannelModel::GetExpDist (double lambda) const
{
  NS_LOG_FUNCTION (this << lambda);
  m_expRv->SetAttribute ("Mean", DoubleValue (lambda));
  double value = m_expRv->GetValue();
  NS_LOG_DEBUG("Value in Exp Dist is:" << value);
  return value;
}

/***********************************************/
double
NYUChannelModel::GetMinValue (double val1, double val2) const
{
  NS_LOG_FUNCTION (this << val1 << val2);
  double output = 0;
  if (val1 > val2)
  {
    output = val2;
  }
  else
  {
    output = val1;
  }
  NS_LOG_DEBUG("Min Value is:" <<output << std::endl);
  return output;
}

double
NYUChannelModel::GetMaxValue (double val1, double val2) const
{
  NS_LOG_FUNCTION (this << val1 << val2);
  double output = 0;
  if (val1 > val2)
  {
    output = val1;
  }
  else
  {
    output = val2;
  }
  NS_LOG_DEBUG("Max Value is:" <<output << std::endl);
  return output;
}

int
NYUChannelModel::GetSignum (double value) const
{
  NS_LOG_FUNCTION (this << value);
  int output = 0;
  if (value > 0)
  {
    output = 1;
  }
  else if (value < 0)
  {
    output = -1;
  }
  else
  {
    output = 0;
  }
  NS_LOG_DEBUG("Signum Function output value is:" <<output << std::endl);
  return output;
}

double
NYUChannelModel::Getdb2pow(double pwr_dB) const
{
  double pwr_lin = 0;
  pwr_lin = std::pow(10,(pwr_dB * 0.10));
  return pwr_lin;
}

/********************************* Channel Parameters API  ********************************************************/
int
NYUChannelModel::GetNumTC (double lambda_c) const
{
  NS_LOG_FUNCTION (this << lambda_c);
  
  int numTC = 0;
  if (m_scenario.compare("InH") == 0)
  {
    numTC = GetPoissionDist(lambda_c) + 1; 
  }
  else if (m_scenario.compare("RMa") == 0)
  {
    numTC = 1;
  }
  else
  {
    // UMa or UMi scenario
    numTC = GetDUniformDist(1,6);
  }
  NS_LOG_DEBUG(" Scenario:" << m_scenario << " numTC is:" << numTC);
  return numTC;
}

int 
NYUChannelModel::GetNumAOALobes (double mu_AOA) const
{
    NS_LOG_FUNCTION (this << mu_AOA);

    int numAOALobes = 0;
    if (m_scenario.compare("InH") == 0)
    {
        numAOALobes = GetDUniformDist(1,mu_AOA);
    }
    else if (m_scenario.compare("RMa") == 0)
    {
        numAOALobes = 1;
    }
    else
    {
        /* UMi or UMa */
        int aoa_instance = 0;
        aoa_instance = GetPoissionDist(mu_AOA);
        numAOALobes = GetMaxValue(1,GetMinValue(5,aoa_instance));
    }
    NS_LOG_DEBUG(" Scenario:" << m_scenario << " number of AOA Spatial Lobes is:" << numAOALobes);
    return numAOALobes;
}

int
NYUChannelModel::GetNumAODLobes (double mu_AOD) const
{
    NS_LOG_FUNCTION (this << mu_AOD);
    
    int numAODLobes = 0;
    if (m_scenario.compare("InH") == 0)
    {
        numAODLobes = GetDUniformDist(1,mu_AOD);
    }
    else if (m_scenario.compare("RMa") == 0)
    {
        numAODLobes = 1;
    }
    else
    {
        /* UMi or UMa */
        int aod_instance = 0;
        aod_instance = GetPoissionDist(mu_AOD);
        numAODLobes = GetMaxValue(1,GetMinValue(5,aod_instance));
    }
    NS_LOG_DEBUG(" Scenario:" << m_scenario << " number of AOD Spatial Lobes is:" << numAODLobes);
    return numAODLobes;
}

MatrixBasedChannelModel::DoubleVector
NYUChannelModel::GetNumSPinTC (int num_TC) const
{
    NS_LOG_FUNCTION (this << num_TC);

    int i;
    MatrixBasedChannelModel::DoubleVector SPperTC;

    if(m_scenario.compare("RMa") == 0)
    {
        for(i = 0; i < num_TC; i++)
        {
            SPperTC.push_back(GetDUniformDist(1,2));
        }
    }
    else
    {
        for(i = 0; i < num_TC; i++)
        {
            SPperTC.push_back(GetDUniformDist(1,30));
        }
    }

    for(long unsigned int i = 0; i < SPperTC.size(); i++)
    {
        NS_LOG_DEBUG("Time Cluster:"<< i <<" Number of Subpaths:" << SPperTC[i] << std::endl);
    }
    return SPperTC;
}

MatrixBasedChannelModel::Double2DVector 
NYUChannelModel::GetIntraClusterDelays(MatrixBasedChannelModel::DoubleVector numSPinTC, double mu_rho) const
{
    NS_LOG_FUNCTION (this << mu_rho << m_rfbw);

    int i,j = 0;
    int numSP = 0;
    double tmp = 0;
    double min_delay = 0;
    double x = 0;
    MatrixBasedChannelModel::DoubleVector arrayTemp;
    MatrixBasedChannelModel::Double2DVector SPdelaysinTC;

    if (m_scenario.compare("InH") == 0)
    {
        /* To be implemented later */
    }
    else
    {
        /* UMi,UMa,RMa scenarios */
        for(i = 0; i < (int) numSPinTC.size(); i++)
        {
            // SP in each time cluster
            numSP = numSPinTC[i];

            // generating delay in ns for each SP in a TC. rfbw is in Hz.
            for(j = 0; j < numSP; j++)
            {
              tmp = (1/(m_rfbw/2))*1e9*(j+1); 
              arrayTemp.push_back(tmp);
            }

            // finding min_delay
            min_delay = *min_element(arrayTemp.begin(),arrayTemp.end());
            
            for(j = 0; j < (int) arrayTemp.size(); j++)
            {
              arrayTemp[j] = arrayTemp[j] - min_delay;
            }

            // sorting the delay generated
            sort(arrayTemp.begin(), arrayTemp.end());
            
            // multiplying each delay with mu_rho
            
            x = mu_rho * GetUniformDist(0,1);

            for(j = 0; j < (int) arrayTemp.size(); j++)
            {
                arrayTemp[j] = pow(arrayTemp[j],(1+x));
            }
   
            SPdelaysinTC.push_back(arrayTemp);

            // clear vector arrayTemp for each TC
            arrayTemp.clear();

        }

        // Displaying the subpath delay generated for each time cluster for debugging
        for (i = 0; i < (int) SPdelaysinTC.size(); i++) 
        {
            for (j = 0; j < (int) SPdelaysinTC[i].size(); j++)
            {
                NS_LOG_DEBUG("Time Cluster: " << i << " Subpath:" << j << " Delay:" << SPdelaysinTC[i][j] << std::endl);
            }
        }
    }
    return SPdelaysinTC;
}

MatrixBasedChannelModel::Double2DVector 
NYUChannelModel::GetSubpathPhases(MatrixBasedChannelModel::DoubleVector numSPinTC) const
{
  NS_LOG_FUNCTION (this);

  int i,j,k;
  int numSP = 0;
  double SubpathPhases = 0;

  MatrixBasedChannelModel::DoubleVector polarizationPhases;
  MatrixBasedChannelModel::Double2DVector SubpathPhases_db;

  // Number of TC is the size of the vector numSPinTC
  for (i = 0; i < (int) numSPinTC.size(); i++)
  {
    // SP in each time cluster
    numSP = numSPinTC[i];  
    NS_LOG_DEBUG("TC" << i << "numSP:" << numSP);

    for (j = 0; j < numSP; j++)
    {
      NS_LOG_DEBUG("TC" << i << "SP:" << j);
      // generate four phases for each ray. One phase for each polarization
      // phases: {theta_theta (V-V), theta_phi (V-H), phi_theta (H-V), phi-phi (H-H)}
      for (k = 0; k < 4; k++)
      {
        SubpathPhases = GetUniformDist(-1 * M_PI, M_PI);
        polarizationPhases.push_back(SubpathPhases);
      }
      SubpathPhases_db.push_back(polarizationPhases);
      polarizationPhases.clear();
    }
  }
  return SubpathPhases_db;
}


MatrixBasedChannelModel::DoubleVector 
NYUChannelModel::GetClusterExcessTimeDelays(double mu_tau, 
                                            MatrixBasedChannelModel::Double2DVector SPdelayinTC,
                                            double minVoidInterval) const
{
  NS_LOG_FUNCTION (this<<mu_tau << minVoidInterval);
  
  int i,numSPinTC;
  int numTC = 0;
  double min_delay = 0;
  double clusterVoidInterval = 0;
  double LastSPTC = 0;
  double delay = 0;

  MatrixBasedChannelModel::DoubleVector tau_n_prime;
  MatrixBasedChannelModel::DoubleVector tau_n;

  tau_n.resize(1);

  numTC = SPdelayinTC.size();
  clusterVoidInterval = minVoidInterval + (1/(m_rfbw/2))*1e9;

  // For each TC generate a delay based on an exponential distribution
  for (i = 0; i < numTC; i++)
  {
    tau_n_prime.push_back(GetExpDist(mu_tau));
  }

  min_delay = *min_element(tau_n_prime.begin(),tau_n_prime.end());

  for(i = 0; i < (int) tau_n_prime.size(); i++)
  {
    tau_n_prime[i] = tau_n_prime[i] - min_delay;
  }

  sort(tau_n_prime.begin(), tau_n_prime.end());

  // Fetch the delay of the last SP of TC1.
  numSPinTC = SPdelayinTC[0].size();
  LastSPTC = SPdelayinTC[0][numSPinTC-1];

  // First TC delay is 0 ns. For the other TC need to compute the excess delay
  for ( i = 1; i < numTC; i++)
  {
    delay = tau_n_prime[i] + LastSPTC + clusterVoidInterval;
    tau_n.push_back(delay);

    numSPinTC = SPdelayinTC[i].size();
    LastSPTC = tau_n[i] + SPdelayinTC[i][numSPinTC-1];
  }
  
  // display the computed excess delay values for each time cluster
  for ( i = 0; i < (int) tau_n.size(); i++)
  {
    NS_LOG_DEBUG("Mean Excess Delay of TC " << i << " is:" << tau_n[i] << std::endl);
  }

  return tau_n;
}

MatrixBasedChannelModel::DoubleVector 
NYUChannelModel::GetClusterPowers(MatrixBasedChannelModel::DoubleVector tau_n, 
                                  double sigmaCluster,
                                  double tc_gamma) const
{
  NS_LOG_FUNCTION (this << sigmaCluster << tc_gamma);
  
  // Note: This returns the normalized cluster power. Need to multiply it with the Rx power from Large scale later.
  int i;
  int numTC = 0; // num of time clusters
  double shadowing = 0; // shadowing in each time cluster
  double Pwr = 0; // power in time cluster
  double sum_of_cluster_pwr = 0; // sum of powers in all time clusters
  double NormalizedPwr = 0; // each cluster power divided by sum of all cluster power
 
  MatrixBasedChannelModel::DoubleVector z; // Vector storing shadowing in each Time Cluster
  MatrixBasedChannelModel::DoubleVector ClusterPwr; // Vector storing power of each Time Cluster
  MatrixBasedChannelModel::DoubleVector NormalizedClusterPwr; // Vector storing normalized power of each Time Cluster

  numTC = tau_n.size();

  for (i = 0; i < numTC; i++)
  {
    shadowing = sigmaCluster * m_normalRv->GetValue();
    z.push_back(shadowing);
  }

  // debugging: to check shadowing power in each time cluster
  for ( i = 0; i < (int)z.size(); i++)
  {
    NS_LOG_DEBUG("Shadowing power in TC: " << i << " is:" << z[i]);
  }

  for (i = 0; i < numTC; i++)
  {
    Pwr = exp(-tau_n[i]/tc_gamma)*(pow(10,(z[i]/10)));
    ClusterPwr.push_back(Pwr);
  }

  // debugging: to check power distribution as per exponential distribution in each time cluster 
  for ( i = 0; i < (int)ClusterPwr.size(); i++)
  {
    NS_LOG_DEBUG("Exponential Power distrubution in TC: " << i << " is:" << ClusterPwr[i]);
  }

  // sum cluster power
  sum_of_cluster_pwr = std::accumulate(ClusterPwr.begin(), ClusterPwr.end(), 0.0);

  //debugging: Sum of the total power of all Time Cluster
  NS_LOG_DEBUG("Sum of Powers in all TC is:" << sum_of_cluster_pwr);

  // normalize cluster ratios
  for ( i = 0; i < (int)ClusterPwr.size(); i++)
  {
    NormalizedPwr = ClusterPwr[i]/sum_of_cluster_pwr;
    NormalizedClusterPwr.push_back(NormalizedPwr);
  }

  //debugging: check the normalized cluster power
  for ( i = 0; i < (int)ClusterPwr.size(); i++)
  {
    NS_LOG_DEBUG("Normalized Cluster Power for TC " << i << " is:" << NormalizedClusterPwr[i]);
  }
  return NormalizedClusterPwr;
}

MatrixBasedChannelModel::Double2DVector 
NYUChannelModel::GetSubPathPowers(MatrixBasedChannelModel::Double2DVector SPdelayinTC, 
                                  MatrixBasedChannelModel::DoubleVector TCPowers,
                                  double sigmaSubpath,
                                  double sp_gamma,
                                  bool los) const
{
  NS_LOG_FUNCTION (this << sigmaSubpath << sp_gamma << los);

  int i,j; // loop iteration
  int numTC = 0; // Number of Time Clusters
  int numSPinTC = 0; //Number of SP in each Time Cluster
  int maxElementIndex = 0; // Find the index of the strongest subpath power in time cluster one for LOS

  double shadowing = 0; // shadowing power for each subpath
  double subPathRatios_tmp = 0; // subpath power w.r.t distributions
  double maxElement = 0; // maximum value of subpath power for TC1 in LOS
  double tmp; // used for swapping powers for TC1 in LOS condition
  double sum_of_sp_pwr = 0; // sum of subpath powers in a particular time cluster
  double subPathRatios = 0; // final power of each subpath in a time cluster

  numTC = TCPowers.size(); // Number of Time Clusters
  MatrixBasedChannelModel::DoubleVector u; // shadowing values for all SP in a one TC
  MatrixBasedChannelModel::DoubleVector SubPathRatios_tmp_vector; // Subpath power from distribution
  MatrixBasedChannelModel::DoubleVector subPathRatios_vect;// normalized subpath for each time cluster
  MatrixBasedChannelModel::Double2DVector subPathPowers; // Vector storing the final computed power of all subpaths in a time cluster

  // Each time cluster
  for ( i = 0; i < numTC; i++)
  {
    numSPinTC = SPdelayinTC[i].size();

    // Shadowing values for all SP in a TC
    for (j = 0; j < numSPinTC; j++)
    {
      shadowing = sigmaSubpath * m_normalRv->GetValue();
      u.push_back(shadowing);
    }

    // debugging: to shadowing power for all SP in a time cluster 
    for ( j = 0; j < (int)u.size(); j++)
    {
      NS_LOG_DEBUG("TC:"<< i <<" Shadowing Power for SP:" << j << " is:" << u[j]);
    }

    // Each SP of a time cluster store the power
    for (j = 0; j < numSPinTC; j++)
    {
      subPathRatios_tmp = exp(-SPdelayinTC[i][j]/sp_gamma)*(pow(10,u[j]/10));
      SubPathRatios_tmp_vector.push_back(subPathRatios_tmp);
    }

    // debugging: exponential power distribution for SPs in a TC 
    for ( j = 0; j < (int)SubPathRatios_tmp_vector.size(); j++)
    {
      NS_LOG_DEBUG("TC:"<< i <<" Exponential Distribued Power for SP:" << j << " is:" << SubPathRatios_tmp_vector[j]);
    } 

    // For 1st Time Cluster the First SP is the SP with the strongest power
    if (i == 1 && los)
    {
      NS_LOG_DEBUG("In LOS condition first SP of 1st TC has the strongest power");
      maxElementIndex = std::max_element(SubPathRatios_tmp_vector.begin(),SubPathRatios_tmp_vector.end()) - SubPathRatios_tmp_vector.begin();
      maxElement = *std::max_element(SubPathRatios_tmp_vector.begin(), SubPathRatios_tmp_vector.end());
      tmp = SubPathRatios_tmp_vector[0];
      SubPathRatios_tmp_vector[0] = maxElement;
      SubPathRatios_tmp_vector[maxElementIndex] = tmp;
    }

    // sum supath powers in one time cluster
    sum_of_sp_pwr = std::accumulate(SubPathRatios_tmp_vector.begin(), SubPathRatios_tmp_vector.end(), 0.0);
    NS_LOG_DEBUG("Sum of SP Power in TC" << i << " is:" << sum_of_sp_pwr);
    
    for(j = 0; j < numSPinTC; j++)
    {
      subPathRatios = (SubPathRatios_tmp_vector[j]/sum_of_sp_pwr)*TCPowers[i];
      subPathRatios_vect.push_back(subPathRatios);
    }

    subPathPowers.push_back(subPathRatios_vect);
    u.clear();
    SubPathRatios_tmp_vector.clear();
    subPathRatios_vect.clear();
  }

  // Displaying the subpath powers generated for each time cluster for debugging
  for (i = 0; i < (int) subPathPowers.size(); i++) 
  {
    for (j = 0; j < (int) subPathPowers[i].size(); j++)
    {
      NS_LOG_DEBUG("Time Cluster: " << i << " Subpath:" << j << " Power:" << subPathPowers[i][j] << std::endl);
    }
  }

  return subPathPowers;
}

MatrixBasedChannelModel::Double2DVector 
NYUChannelModel::getAbsolutePropTimes(double distance2D,
                                      MatrixBasedChannelModel::DoubleVector TCExcessDelay,
                                      MatrixBasedChannelModel::Double2DVector SPdelayinTC) const
{
  NS_LOG_FUNCTION (this << distance2D);

  int numTC = 0;
  int i,j;
  double time = 0;
  double absdelay_tmp = 0;

  MatrixBasedChannelModel::DoubleVector absdelay;
  MatrixBasedChannelModel::Double2DVector absSPdelayinTC;

  numTC = TCExcessDelay.size();
  //time = (distance2D/M_C)*1e9;

  // setting time = 0, removing the absolute delay
  time = 0;

  NS_LOG_DEBUG("Absolute Propgation is:" << time);

  for(i = 0; i < numTC; i++)
  {
    for ( j = 0; j < (int) SPdelayinTC[i].size(); j++)
    {
      absdelay_tmp = time + TCExcessDelay[i] + SPdelayinTC[i][j];
      absdelay.push_back(absdelay_tmp);
    }
    absSPdelayinTC.push_back(absdelay);
    absdelay.clear();
  }
  
  // Displaying the absolute subpath delays generated for each time cluster for debugging
  for (i = 0; i < (int) absSPdelayinTC.size(); i++) 
  {
    for (j = 0; j < (int) absSPdelayinTC[i].size(); j++)
    {
      NS_LOG_DEBUG("Time Cluster: " << i << " Subpath:" << j << " Delay:" << absSPdelayinTC[i][j] << std::endl);
    }
  }
  return absSPdelayinTC;
}

MatrixBasedChannelModel::Double2DVector 
NYUChannelModel::GetSubpathMappingandAngles(int numLobes,
                                            MatrixBasedChannelModel::DoubleVector numSPinTC,
                                            double mean,
                                            double sigma,
                                            double std_RMSLobeElevationSpread,
                                            double std_RMSLobeAzimuthSpread,
                                            std::string distributionType) const
{
  NS_LOG_FUNCTION (this << numLobes << mean << sigma << std_RMSLobeElevationSpread << std_RMSLobeAzimuthSpread << distributionType);

  int i,j;
  int numTC = 0; // Number of Time Clusters
  int numSP = 0; // Number of SubPaths
  int randomLobeIndex = 0; // Assign SP to a Spatial Lobe

  double az_min,az_max; // Splitting azimuth planes
  double tmp_mean_azi_angle,tmp_mean_elev_angle; // Assign angles to lobes and Subpaths in elevation and azimuth
  double deltaElev,deltaAzi; // standard deviation of subpath w.r.t to mean angles of lobes in azimuth and elevation
  double subpathAzi,subpathElev; // computed azimuth and elevation angles of the subpath
  double z,b;

  MatrixBasedChannelModel::DoubleVector lobeindices; // contains lobe index from 1 to numLobes
  MatrixBasedChannelModel::DoubleVector theta_min_array; // min splitting angles for azimuth plane. 
  MatrixBasedChannelModel::DoubleVector theta_max_array; // max splitting angles for azimuth plane.
  MatrixBasedChannelModel::DoubleVector mean_ElevationAngles; // mean elevation angles for spatial lobes
  MatrixBasedChannelModel::DoubleVector mean_AzimuthAngles; // mean azimuth angles for spatial lobes
  MatrixBasedChannelModel::Double2DVector cluster_subpath_lobe_az_elev_angles; // stores the sp->tc->lobe mapping and Azimuth and Elevation angles of each SP

  numTC = numSPinTC.size();

  // Skip the first index i.e. index 0 as Lobe indices start from 1 to numLobes. 
  mean_ElevationAngles.resize(1);
  mean_AzimuthAngles.resize(1);
  
  // Lobe indices
  for (i = 0; i < numLobes; i++)
  {
    lobeindices.push_back(i+1);
  }

  // debugging: Lobe indices
  for (i = 0; i < (int)lobeindices.size(); i++)
  {
    NS_LOG_DEBUG("Lobe index generated is:" << lobeindices[i] << std::endl);
  }

  // Discretize azimuth plane    
  for(i = 0; i < (int) lobeindices.size(); i++)
  {
    az_min = 360*(lobeindices[i] - 1)/numLobes;
    az_max = 360*(lobeindices[i])/numLobes;
    theta_min_array.push_back(az_min);
    theta_max_array.push_back(az_max);
  }

  // debugging: theta min and theta max values
  for (i = 0; i < (int)theta_min_array.size(); i++)
  {
    NS_LOG_DEBUG("Theta min value:" << theta_min_array[i] << std::endl);
    NS_LOG_DEBUG("Theta max value:" << theta_max_array[i] << std::endl);
  }

  // compute mean elevation and azimuth angles
  for (i = 0; i < numLobes; i++)
  {
    tmp_mean_elev_angle = mean + sigma * m_normalRv->GetValue();
    tmp_mean_azi_angle = theta_min_array[i] + (theta_max_array[i] - theta_min_array[i])*GetUniformDist(0,1);
    mean_ElevationAngles.push_back(tmp_mean_elev_angle);
    mean_AzimuthAngles.push_back(tmp_mean_azi_angle);
  }

  for (i = 1; i < (int)mean_ElevationAngles.size(); i++)  
  {
    NS_LOG_DEBUG("Mean Elevation Angle:" << mean_ElevationAngles[i] << std::endl);
    NS_LOG_DEBUG("Mean Azimuth Angle:" << mean_AzimuthAngles[i] << std::endl);
  }

  // main code to compute SP angles and do mapping
  for (i = 0; i < numTC; i++)
  {
    numSP = numSPinTC[i];   
    for (j = 0; j < numSP; j++)
    {
      randomLobeIndex = GetDUniformDist(1,numLobes);  
      tmp_mean_elev_angle = mean_ElevationAngles[randomLobeIndex];
      tmp_mean_azi_angle = mean_AzimuthAngles[randomLobeIndex];
      deltaAzi = std_RMSLobeAzimuthSpread * m_normalRv->GetValue();
            
      if(distributionType.compare("Gaussian") == 0)
      {
        deltaElev = std_RMSLobeElevationSpread * m_normalRv->GetValue();
      }
      else if(distributionType.compare("Laplacian") == 0)
      {
        z = -0.5 + GetUniformDist(0,1);
        b = std_RMSLobeElevationSpread/sqrt(2);
        deltaElev = -b*GetSignum(z) * log(1 - 2*abs(z));
      }
      else
      {
        NS_FATAL_ERROR ("Invalid Distribution");
      }

      subpathAzi = WrapTo360(tmp_mean_azi_angle + deltaAzi);
      subpathElev = GetMinValue((GetMaxValue(tmp_mean_elev_angle + deltaElev, -60)),60);

      cluster_subpath_lobe_az_elev_angles.push_back({(double)i,(double)j,(double)randomLobeIndex,subpathAzi,subpathElev});
    }   
  }

  // For debugging the Generated Subpaths Azimuth and Elevation angles
  for(i = 0; i < (int) cluster_subpath_lobe_az_elev_angles.size(); i++)
  {
    for (j = 0; j < (int)cluster_subpath_lobe_az_elev_angles[i].size(); j++)
    {
      if (j == 0)
      {
        NS_LOG_DEBUG("TC Id:" << cluster_subpath_lobe_az_elev_angles[i][j]);
      }
      else if (j == 1)
      {
        NS_LOG_DEBUG("SP Id:" << cluster_subpath_lobe_az_elev_angles[i][j]);
      }
      else if (j == 2)
      {
        NS_LOG_DEBUG("Lobe Id:" << cluster_subpath_lobe_az_elev_angles[i][j]);
      }
      else if (j == 3)
      {
        NS_LOG_DEBUG("azimuth:" << cluster_subpath_lobe_az_elev_angles[i][j]);
      }
      else if (j == 4)
      {
        NS_LOG_DEBUG("elevation:" << cluster_subpath_lobe_az_elev_angles[i][j]);
      }
      else
      {
        NS_FATAL_ERROR ("Invalid Index Accessed");
      }
    }
  }
  return cluster_subpath_lobe_az_elev_angles;
}

MatrixBasedChannelModel::Double2DVector
NYUChannelModel::GetPowerSpectrum(MatrixBasedChannelModel::DoubleVector numSPinTC,
                                  MatrixBasedChannelModel::Double2DVector absoluteSPdelayinTC,
                                  MatrixBasedChannelModel::Double2DVector SPPowers,
                                  MatrixBasedChannelModel::Double2DVector SPPhases,
                                  MatrixBasedChannelModel::Double2DVector AOD_cluster_subpath_lobe_az_elev_angles,
                                  MatrixBasedChannelModel::Double2DVector AOA_cluster_subpath_lobe_az_elev_angles)const
{
  int i,j;
  int numTC = 0;
  int numSP = 0;

  double subpathDelay,subpathPower,subpathPhase,subpath_AOD_Azi,subpath_AOD_EL,subpath_AOA_Azi,subpath_AOA_EL;
  double subpath_AOD_Lobe,subpath_AOA_Lobe;

  MatrixBasedChannelModel::Double2DVector PowerSpectrum;

  numTC = numSPinTC.size();
  
  for (i = 0; i < numTC; i++)
  {
    for (j = 0; j < numSPinTC[i]; j++)
    {
      subpathDelay = absoluteSPdelayinTC[i][j];
      subpathPower = SPPowers[i][j];
      subpathPhase = SPPhases[numSP][0];
      subpath_AOD_Azi = AOD_cluster_subpath_lobe_az_elev_angles[j][3];
      subpath_AOD_EL = AOD_cluster_subpath_lobe_az_elev_angles[j][4];
      subpath_AOA_Azi = AOA_cluster_subpath_lobe_az_elev_angles[j][3];
      subpath_AOA_EL = AOA_cluster_subpath_lobe_az_elev_angles[j][4];
      subpath_AOD_Lobe = AOD_cluster_subpath_lobe_az_elev_angles[j][2];
      subpath_AOA_Lobe = AOA_cluster_subpath_lobe_az_elev_angles[j][2];

      PowerSpectrum.push_back({subpathDelay,subpathPower,subpathPhase,subpath_AOD_Azi,subpath_AOD_EL,
                              subpath_AOA_Azi,subpath_AOA_EL,subpath_AOD_Lobe, subpath_AOA_Lobe});
      numSP++;
    }
  }

  NS_LOG_DEBUG("Total Number of SP is:" << numSP);

  // Displaying the absolute subpath delays generated for each time cluster for debugging
  for (i = 0; i < (int) PowerSpectrum.size(); i++) 
  {
    NS_LOG_DEBUG("Subpath id:" << i << std::endl);
    for (j = 0; j < (int) PowerSpectrum[i].size(); j++)
    {
      if (j == 0)
      {
        NS_LOG_DEBUG("SubpathDelay:" << PowerSpectrum[i][j]);
      }
      else if (j == 1)
      {
        NS_LOG_DEBUG("SubpathPower:" << PowerSpectrum[i][j]);
      }
      else if (j == 2)
      {
        NS_LOG_DEBUG("SubpathPhase:" << PowerSpectrum[i][j]);
      }
      else if (j == 3)
      {
        NS_LOG_DEBUG("Subpath_AOD_Azi:" << PowerSpectrum[i][j]);
      }
      else if (j == 4)
      {
        NS_LOG_DEBUG("subpath_AOD_EL:" << PowerSpectrum[i][j]);
      }
      else if (j == 5)
      {
        NS_LOG_DEBUG("subpath_AOA_Azi:" << PowerSpectrum[i][j]);
      }
      else if (j == 6)
      {
        NS_LOG_DEBUG("subpath_AOA_EL:" << PowerSpectrum[i][j]);
      }
      else if (j == 7)
      {
        NS_LOG_DEBUG("subpath_AOD_Lobe:" << PowerSpectrum[i][j]);
      }
      else if (j == 8)
      {
        NS_LOG_DEBUG("subpath_AOA_Lobe:" << PowerSpectrum[i][j]);
      }
      else
      {
        NS_FATAL_ERROR ("Invalid Index Accessed");
      }
    }
  } 
  return PowerSpectrum;
}

MatrixBasedChannelModel::Double2DVector
NYUChannelModel::GetBWAdjustedtedPowerSpectrum(MatrixBasedChannelModel::Double2DVector PowerSpectrumOld, 
                                              double RFBandwidth,
                                              bool los) const
{

  NS_LOG_FUNCTION(this << RFBandwidth << los);
  int numSP = 0; // number of Subpaths
  int i,j;
  int NumSPcombined = 0; // Number of Supbaths combined togther in one boundary time
  int index = 0;

  double MinTimeSP = 0; // Subpaths within this duration cannot be resolved. They appear as one subpath.
  double BoundaryTime = 0; //All Subpaths <= boundary time are combined together
  double SPcombinedPwr = 0; // Combined complex power of the SP

  std::complex <double> sum_sp = 0; // add the subpath amplitude and phase together

  bool SetBoundaryTime = true; // indicated a new boundary time is being set. 

  MatrixBasedChannelModel::Double2DVector PowerSpectrum;
      
  MinTimeSP = (1/(m_rfbw/2))*1e9;
  NS_LOG_DEBUG("SP Resolution Time:" << MinTimeSP);

  numSP = PowerSpectrumOld.size();
  
  for( i = 0; i < numSP; i++)
  {
    if(SetBoundaryTime)
    {
      BoundaryTime = PowerSpectrumOld[i][0] + MinTimeSP;
      NS_LOG_DEBUG("Subpath id:" << i << " Boundary Time:" << BoundaryTime << std::endl);
      SetBoundaryTime = false;
      PowerSpectrum.push_back(PowerSpectrumOld[i]);
      index = i;
    }
    if(PowerSpectrumOld[i][0] <= BoundaryTime)
    {
      NumSPcombined++;
      sum_sp = sum_sp + sqrt(PowerSpectrumOld[i][1]) * exp(std::complex <double> (0,PowerSpectrumOld[i][2]));
    }
    else
    {
      NS_LOG_DEBUG("Total Number of SP combined:" << NumSPcombined);
      SPcombinedPwr = pow(abs(sum_sp),2);
      PowerSpectrum[index][1] =  SPcombinedPwr; 
      NS_LOG_DEBUG("Combined SP Power:" << PowerSpectrum[index][1] << std::endl);
      SetBoundaryTime = true;
      sum_sp = 0;
      NumSPcombined = 0;
      i = i - 1; // one index gets skipped so go back and recalculate the boundary
      index = 0;
    }
    if (i == numSP - 1)
    {
      NS_LOG_DEBUG("Total Number of SP combined:" << NumSPcombined);
      SPcombinedPwr = pow(abs(sum_sp),2);
      PowerSpectrum[i][1] =  SPcombinedPwr;
      NS_LOG_DEBUG("Combined SP Power:" << PowerSpectrum[i][1] << std::endl);
    }
  }

  PowerSpectrum = GetLosAlignedPowerSpectrum(PowerSpectrum,los);

  NS_LOG_DEBUG("Final PowerSpectrum values after BW Adjustment, Total SP:" << PowerSpectrum.size());

  // Displaying the absolute subpath delays generated for each time cluster for debugging
  for (i = 0; i < (int) PowerSpectrum.size(); i++) 
  {
    NS_LOG_DEBUG("Subpath ID:" << i);
    for (j = 0; j < (int) PowerSpectrum[i].size(); j++)
    {
      if (j == 0)
      {
        NS_LOG_DEBUG("SubpathDelay:" << PowerSpectrum[i][j]);
      }
      else if (j == 1)
      {
        NS_LOG_DEBUG("SubpathPower:" << PowerSpectrum[i][j]);
      }
      else if (j == 2)
      {
        NS_LOG_DEBUG("SubpathPhase:" << PowerSpectrum[i][j]);
      }
      else if (j == 3)
      {
        NS_LOG_DEBUG("Subpath_AOD_Azi:" << PowerSpectrum[i][j]);
      }
      else if (j == 4)
      {
        NS_LOG_DEBUG("subpath_AOD_EL:" << PowerSpectrum[i][j]);
      }
      else if (j == 5)
      {
        NS_LOG_DEBUG("subpath_AOA_Azi:" << PowerSpectrum[i][j]);
      }
      else if (j == 6)
      {
        NS_LOG_DEBUG("subpath_AOA_EL:" << PowerSpectrum[i][j]);
      }
      else if (j == 7)
      {
        NS_LOG_DEBUG("subpath_AOD_Lobe:" << PowerSpectrum[i][j]);
      }
      else if (j == 8)
      {
        NS_LOG_DEBUG("subpath_AOA_Lobe:" << PowerSpectrum[i][j]);
      }      
      else
      {
        NS_FATAL_ERROR ("Invalid Index Accessed");
      }
    }
  } 
    
  return PowerSpectrum;
}

MatrixBasedChannelModel::Double2DVector 
NYUChannelModel::GetLosAlignedPowerSpectrum(MatrixBasedChannelModel::Double2DVector &PowerSpectrum, 
                                            bool los) const
{
  NS_LOG_FUNCTION(this << los << m_scenario);
  
  double correctAzAOA = 0;
  double diffAzAOA = 0;
  double correctELAOA = 0;
  double diffElAOA = 0;

  int i;

  // In LOS the first Subpath i.e. Subpath 0 in AOD and AOA , ZOD and ZOA should be aligned.
  if (los)
  {
    // Subpath 0 - Azimuth AOD
    if (PowerSpectrum[0][3] - 180 > 0)
    {
      correctAzAOA = PowerSpectrum[0][3] - 180;
    }
    else
    {
      correctAzAOA = PowerSpectrum[0][3] + 180;
    }
    NS_LOG_DEBUG("Corrected Az AOA is:" << correctAzAOA << std::endl);

    // Calculate the difference between generated azimuth AOA and correct Azimuth AOA.
    diffAzAOA = PowerSpectrum[0][5] - correctAzAOA;
    NS_LOG_DEBUG("Diff between generated Az AOA and corrected Az AOA is:" << diffAzAOA << std::endl);

    // Correct all AOA w.r.t to the AOA of the first LOS subpath
    for (i = 0; i < (int) PowerSpectrum.size(); i++)
    {
      PowerSpectrum[i][5] = PowerSpectrum[i][5] - diffAzAOA;
      PowerSpectrum[i][5] = WrapTo360(PowerSpectrum[i][5]);
    }

    // Debug SP AOA alignment
    for (i = 0; i < (int) PowerSpectrum.size(); i++)
    {
      NS_LOG_DEBUG("Adjusted AOA for Subpath" << i <<" is:"<< PowerSpectrum[i][5] << std::endl);
    }

    // Fetch the ZOD elevation
    correctELAOA = -PowerSpectrum[0][4];
    NS_LOG_DEBUG("Corrected Az ZOA is:" << correctELAOA << std::endl);

    // Calculate the difference between generated ZOA and correct Azimuth ZOD.
    diffElAOA = PowerSpectrum[0][6] - correctELAOA;
    NS_LOG_DEBUG("Diff between generated Az ZOA and corrected Az ZOA is:" << diffElAOA << std::endl);

    // Correct all ZOA w.r.t to the ZOA of the first LOS subpath
    for (i = 0; i < (int) PowerSpectrum.size(); i++)
    {
      PowerSpectrum[i][6] = PowerSpectrum[i][6] - diffElAOA;
      if (PowerSpectrum[i][6] > 90)
      {
        PowerSpectrum[i][6] = 180 - PowerSpectrum[i][6];
      }
      else if (PowerSpectrum[i][6] < -90)
      {
        PowerSpectrum[i][6] = -180 - PowerSpectrum[i][6];
      }
    }

    // Debug SP AOA alignment
    for (i = 0; i < (int) PowerSpectrum.size(); i++)
    {
      NS_LOG_DEBUG("Adjusted ZOA for Subpath" << i <<" is:"<< PowerSpectrum[i][6] << std::endl);
    }
  }
  else
  {
    NS_LOG_DEBUG("PowerSpectrum alignement not needed, scnario is:" << m_scenario << std::endl);
  }
  return PowerSpectrum;
}

MatrixBasedChannelModel::Double2DVector
NYUChannelModel::GetValidSubapths(MatrixBasedChannelModel::Double2DVector PowerSpectrum, double pwrthreshold) const
{
  NS_LOG_FUNCTION(this);

  MatrixBasedChannelModel::Double2DVector PowerSpectrumOptimized;
  double maxSubpathPower = 0;
  double maxSubpathPowerID = 500; // 500 is a dummy subpath id
  double threshold = 0; // in dB
  double subpathPower = 0; // subpath Power in dB

  for (int i = 0; i < (int) PowerSpectrum.size(); i++)
  {
    if (PowerSpectrum[i][1] > maxSubpathPower)
    {
      maxSubpathPower = PowerSpectrum[i][1];
      maxSubpathPowerID = i;
    }  
  }

  threshold = 10*log10(maxSubpathPower) - pwrthreshold;
  NS_LOG_DEBUG("Max Subpath Power lin_scale:" << maxSubpathPower << " Max Subpath Power ID:" << maxSubpathPowerID << " threshold:" << threshold);

  // for all subpaths above the threshold save the Power spectrum
  for (int i = 0; i < (int)PowerSpectrum.size(); i++)
  {
    subpathPower = 10*log10(PowerSpectrum[i][1]);
    if (subpathPower > threshold)
    {
      PowerSpectrumOptimized.push_back(PowerSpectrum[i]);
    }
  }

  NS_LOG_DEBUG("Total Number of Subpath after removing weak subpaths is: " << PowerSpectrumOptimized.size());

  return PowerSpectrumOptimized;
}

MatrixBasedChannelModel::Double2DVector 
NYUChannelModel::GetXPDperRay(double totalNumSP,
                              double XPD_Mean,
                              double XPD_Sd) const
{
  MatrixBasedChannelModel::Double2DVector XPD;
  int i,j;
  // Polarization values for HH (phi_phi), VH(theta_phi), HV (phi_theta)
  double phi_phi,theta_phi,phi_theta;

  for (i = 0; i < totalNumSP; i++)
  {
    phi_phi = m_normalRv->GetValue() * XPD_Sd;
    theta_phi = XPD_Mean;
    phi_theta = XPD_Mean + m_normalRv->GetValue() * XPD_Sd;
    XPD.push_back({phi_phi,theta_phi,phi_theta});
  }
  
  //debugging XPD values for each Ray
  for (i = 0; i < (int)XPD.size(); i++)
  {
    for ( j = 0; j < (int)XPD[i].size(); j++)
    {
      if (j == 0)
      {
        NS_LOG_DEBUG(" HH XPD value for ray" << i << " is:" << XPD[i][j]);
      }
      else if (j == 1)
      {
        NS_LOG_DEBUG(" VH XPD value for ray" << i << " is:" << XPD[i][j]);
      }
      else if (j == 2)
      {
        NS_LOG_DEBUG(" HV XPD value for ray" << i << " is:" << XPD[i][j]);
      }
    }
  }
  return XPD;
}


MatrixBasedChannelModel::Double2DVector
NYUChannelModel::NYUCStoGCS(MatrixBasedChannelModel::Double2DVector PowerSpectrum) const
{
  MatrixBasedChannelModel::DoubleVector rayAodDegree;
  MatrixBasedChannelModel::DoubleVector rayZodDegree;
  MatrixBasedChannelModel::DoubleVector rayAoaDegree;
  MatrixBasedChannelModel::DoubleVector rayZoaDegree;

  MatrixBasedChannelModel::DoubleVector rayAodRadian;
  MatrixBasedChannelModel::DoubleVector rayZodRadian;
  MatrixBasedChannelModel::DoubleVector rayAoaRadian;
  MatrixBasedChannelModel::DoubleVector rayZoaRadian;

  MatrixBasedChannelModel::Double2DVector m_angle;
  
  // Stroring the AOD,ZOD,AOA,ZOA values and changing it from NYU cordinates to 3GPP GCS
  // Col 3 - AOD , Col 4 - ZOD, Col 5 - AOA, Col 6 - ZOA (all values in degrees)
  for(int i = 0; i < (int) PowerSpectrum.size(); i++)
  {
    rayAodDegree.push_back(PowerSpectrum[i][3]);
    rayZodDegree.push_back(PowerSpectrum[i][4]);
    rayAoaDegree.push_back(PowerSpectrum[i][5]);
    rayZoaDegree.push_back(PowerSpectrum[i][6]);
  }

  // Transforming NYU Cordinate system to GCS. Subtract (90-theta) for elevation and (90-phi)%360 for azimuth 
  // to change NYU measurement cordinate system to GCS.
  for (int i = 0; i < (int) PowerSpectrum.size(); i++)
  {
    rayAodDegree[i] = WrapTo360(90 - rayAodDegree[i]);
    rayZodDegree[i] = 90 - rayZodDegree[i];
    rayAoaDegree[i] = WrapTo360(90 - rayAoaDegree[i]);
    rayZoaDegree[i] = 90 - rayZoaDegree[i];
  }
    
  // Debug for NYU to GCS converted Ray characteristics in degrees (AOD,ZOD,AOA,ZOA)
  for (int i = 0; i < (int) PowerSpectrum.size(); i++)
  {
    NS_LOG_DEBUG("Subpath:"<< i << " GCS AOD:" << rayAodDegree[i] << " degree");
    NS_LOG_DEBUG("Subpath:"<< i << " GCS ZOD:" << rayZodDegree[i] << " degree");
    NS_LOG_DEBUG("Subpath:"<< i << " GCS AOA:" << rayAoaDegree[i] << " degree");
    NS_LOG_DEBUG("Subpath:"<< i << " GCS ZOA:" << rayZoaDegree[i] << " degree");
  }

  // Store the AOD,ZOD,AOA,ZOA in radians for each ray according to GCS.
  rayAodRadian = DegreesToRadians(rayAodDegree);
  rayZodRadian = DegreesToRadians(rayZodDegree);
  rayAoaRadian = DegreesToRadians(rayAoaDegree);
  rayZoaRadian = DegreesToRadians(rayZoaDegree);

  // Debug for NYU to GCS converted Ray characteristics in radians (AOD,ZOD,AOA,ZOA)
  for (int i = 0; i < (int) PowerSpectrum.size(); i++)
  {
    NS_LOG_DEBUG("Subpath:"<< i << " GCS AOD:" << rayAodRadian[i] << " radian");
    NS_LOG_DEBUG("Subpath:"<< i << " GCS ZOD:" << rayZodRadian[i] << " radian");
    NS_LOG_DEBUG("Subpath:"<< i << " GCS AOA:" << rayAoaRadian[i] << " radian");
    NS_LOG_DEBUG("Subpath:"<< i << " GCS ZOA:" << rayZoaRadian[i] << " radian");
  }

  // m_angle is in matrix-based-channel-model.h and we populate the value in radians 
  // for AOD,ZOD,AOA,ZOA in m_angle. This is then used in CalcBeamformingGain() in nyu-spectrum-propagation-loss-model.cc
  // m_angle row 0 - aoa , row 1 - zoa, row 2 -aod, row 3 -zod (for all SP)
  m_angle.push_back(rayAoaRadian);
  m_angle.push_back(rayZoaRadian);
  m_angle.push_back(rayAodRadian);
  m_angle.push_back(rayZodRadian);

  return m_angle;
}

// Dyamic range of NYU Channel Sounder
double
NYUChannelModel::dynamic_range(double distance2D) const
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


}  // namespace ns3
