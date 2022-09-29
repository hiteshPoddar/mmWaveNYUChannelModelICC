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

#ifndef NYU_CHANNEL_H
#define NYU_CHANNEL_H

#include  <complex.h>
#include "ns3/angles.h"
#include <ns3/object.h>
#include <ns3/nstime.h>
#include <ns3/random-variable-stream.h>
#include <ns3/boolean.h>
#include <unordered_map>
#include <ns3/channel-condition-model.h>
#include <ns3/matrix-based-channel-model.h>

namespace ns3 {

class MobilityModel;

/**
 * \ingroup spectrum
 * \brief Channel Matrix Generation
 *
 * The class implements the channel matrix generation procedure
 *
 * \see GetChannel
 */
class NYUChannelModel : public MatrixBasedChannelModel
{
public:
  /**
   * Constructor
   */
  NYUChannelModel ();

  /**
   * Destructor
   */
  ~NYUChannelModel ();
  
  void DoDispose () override;

  /**
   * Get the type ID
   * \return the object TypeId
   */
  static TypeId GetTypeId ();


  /**
   * Set the channel condition model
   * \param a pointer to the ChannelConditionModel object
   */
  void SetChannelConditionModel (Ptr<ChannelConditionModel> model);

  /**
   * Get the associated channel condition model
   * \return a pointer to the ChannelConditionModel object
   */
  Ptr<ChannelConditionModel> GetChannelConditionModel () const;

  /**
   * Sets the center frequency of the model
   * \param f the center frequency in Hz
   */
  void SetFrequency (double freq);

  /**
   * Returns the center frequency
   * \return the center frequency in Hz
   */
  double GetFrequency (void) const;

  /**
   * Sets the RF Bandwidth of the model
   * \param rfBandwidth the RF Bandwidth in Hz
   */
  void SetRfBandwidth (double rfBandwidth);
  
  /**
   * Returns the RF Bandwidth of the model
   * \return the RF Bandwidth in Hz
   */
  double GetRfBandwidth (void) const;

  /**
   * Sets the propagation scenario
   * \param scenario the propagation scenario
   */
  void SetScenario (const std::string &scenario);

  /**
   * Returns the propagation scenario
   * \return the propagation scenario
   */
  std::string GetScenario (void) const;

  /**
   * Looks for the channel matrix associated to the aMob and bMob pair in m_channelMap.
   * If found, it checks if it has to be updated. If not found or if it has to
   * be updated, it generates a new uncorrelated channel matrix using the
   * method GetNewChannel and updates m_channelMap.
   *
   * \param aMob mobility model of the a device
   * \param bMob mobility model of the b device
   * \param aAntenna antenna of the a device
   * \param bAntenna antenna of the b device
   * \return the channel matrix
   */
  Ptr<const ChannelMatrix> GetChannel (Ptr<const MobilityModel> aMob,
                                       Ptr<const MobilityModel> bMob,
                                       Ptr<const PhasedArrayModel> aAntenna,
                                       Ptr<const PhasedArrayModel> bAntenna) override;
  /**
   * \brief Assign a fixed random variable stream number to the random variables
   * used by this model.
   *
   * \param stream first stream index to use
   * \return the number of stream indices assigned by this model
   */
  int64_t AssignStreams (int64_t stream);

  /**
   * The measurements conducted by NYU are at 28,73 and 140 GHz. For other
   * frequencies a linear intrerpolation is done.
   * \param val1 the value of a parameter at 28 GHz
   * \param val2 the value of a parameter at 140 GHz
   * \param frequency the centrer frequency of operation in GHz
   * \return the value of a parameter at any frequency between 28-150 GHz
   */
  double GetCalibratedParameter (double val1, double val2, double frequency) const;

  /**
   * Find maximum value between two given values
   * \param val1 the first number
   * \param val2 the second number
   * \return the Maximum number between first and second number
   */
  double GetMaximumValue (double val1, double val2) const;

  /**
   * Find minimum value among two given values
   * \param val1 the first number
   * \param val2 the second number
   * \return the Minimum number between first and second number
   */
  double GetMinimumValue (double val1, double val2) const;

  /**
   * Generate a value as per signum function
   * \param value input value of signum function
   * \return 1 if value > 0, 0 if value 0 and -1 if value is < 0
   */
  int GetSignum(double value) const;

  /**
   * Generate a random value following a poisson distribution
   * \param lambda mean of the poisson distribution
   * \return a random integer value from a poisson distribution
   */

  int GetPoissionDist (double lambda) const;

  /**
   * Generate a random value following a discrete uniform distribution
   * \param min the lower bound of the discrete uniform distribution
   * \param max the upper bound of the discrete uniform distribution
   * \return a random integer value from a discrete uniform distribution
   */
  int GetDiscreteUniformDist (const double min, const double max) const;

  /**
   * Generate a random value (integer/double) following a uniform distribution
   * \param min lower bound of the uniform distribution
   * \param max upper bound of the uniform distribution
   * \return a random value(integer/double) from a uniform distribution
   */  
  double GetUniformDist (const double min, const double max) const;

  /**
   * Generate a random value following an exponential distribution
   * \param lambda the mean of the exponential distribution
   * \return a random value from an exponential distribution
   */
  double GetExponentialDist (double lambda) const;

  /**
   * Get the number of Time Clusters
   * \param maxNumberOfTimeCluster the maximum number of Time Cluster for UMi,UMa and RMa
   * \param lambdaC mean value of the number of time cluster for InH
   * \return the number of Time Clusters
   */
  int GetNumberOfTimeClusters (double maxNumberOfTimeCluster, double lambdaC) const;

  /**
   * Get the number of Angle of Arrival (AOA) Spatial Lobes i.e. the Rx Spatial Lobes
   * \param muAoa the mean value of the number of Angle of Arrival (AOA) Spatial Lobes
   * \return the number of Angle of Arrival (AOA) Spatial Lobes
   */
  int GetNumberOfAoaSpatialLobes (double muAoa) const;

  /**
   * Get the number of Angle of Departure (AOD) Spatial Lobes i.e. the Tx Spatial Lobes
   * \param muAod the mean value of the number of Angle of Departure (AOD) Spatial Lobes
   * \return the number of Angle of Departure (AOD) Spatial Lobes
   */
  int GetNumberOfAodSpatialLobes (double muAod) const;

  /**
   * Get the number of Subpaths/Multipaths/rays in each Time Cluster
   * \param numberOfTimeClusters the number of Time Clusters in UMi,UMa and RMa
   * \param maxNumberOfSubpaths the maximum number of Subpaths in UMi,UMa and RMa
   * \param betaS the scaling factor for InH and InF
   * \param muS the mean of the exponential distribution for InH
   * \param sigmaS the scale of the Pareto distribution for InF
   * \param kS the shape of the Pareto distribution for InF
   * \param thethaS the bound of the Pareto distribution for InF
   * \return the number of Subpaths in each Time Cluster
   */
  MatrixBasedChannelModel::DoubleVector
  GetNumberOfSubpathsInTimeCluster (int numberOfTimeClusters, 
                                    double maxNumberOfSubpaths, 
                                    double betaS, 
                                    double muS,
                                    double sigmaS,
                                    double kS,
                                    double thethaS) const;

   /**
   * Get the Subpath delay in each Time Cluster (in ns)
   * \param numberOfSubpathInTimeCluster the number of subpaths in each time cluster
   * \param muRho the mean subpath delay in each time cluster (in ns) 
   * \param alphaRho 
   * \param betaRho 
   * \return the delay of each Subpath in each Time Cluster (in ns)
   */
  MatrixBasedChannelModel::Double2DVector GetIntraClusterDelays (MatrixBasedChannelModel::DoubleVector numberOfSubpathInTimeCluster,
                                                                double muRho,
                                                                double alphaRho,
                                                                double betaRho) const;

  /**
   * Get the Subpath phases of each Subapath in each Time Cluster
   * \param numberOfSubpathInTimeCluster the number of subpath in each Time Cluster
   * \return the phases of each subpath in each Time Cluster
   */
  MatrixBasedChannelModel::Double2DVector GetSubpathPhases (MatrixBasedChannelModel::DoubleVector numberOfSubpathInTimeCluster) const;

  /**
   * Get the Delay of each Time Cluster (in ns)
   * \param muTau the mean excess delay of each Time Cluster (in ns) for UMi,UMa, RMa and InH
   * \param subpathDelayInTimeCluster the subpath delay in each Time Cluster (in ns)
   * \param minimumVoidInterval the mimumum time in ns by which two Time Clusters are seperated (in ns)
   * \param aplhaTau the alpha value of the gamma distribution for Time Cluster delay (in ns) for InF  
   * \param betaTau the beta value of the gamma distribution for Time Cluster delay (in ns) for InF  
   * \return the delay of each Time Cluster (in ns)
   */
  MatrixBasedChannelModel::DoubleVector GetClusterExcessTimeDelays (double muTau,
                                                                    MatrixBasedChannelModel::Double2DVector subpathDelayInTimeCluster,
                                                                    double minimumVoidInterval,
                                                                    double aplhaTau,
                                                                    double betaTau) const;

  /**
   * Get the Normalized Power of each Time Cluster (in Watts)
   * \param getClusterExcessTimeDelays the mean excess delay of each Time Cluster (in ns)
   * \param sigmaCluster the shadowing value in each Time Cluster (in dB)
   * \param timeClusterGamma the Time Cluster decay constant (in ns)
   * \return the Normalized Power in each Time Cluster (in Watts)
   */
  MatrixBasedChannelModel::DoubleVector GetClusterPowers (MatrixBasedChannelModel::DoubleVector getClusterExcessTimeDelays,
                                                          double sigmaCluster, 
                                                          double timeClusterGamma) const;

  /**
   * Get the Normalized Power of each Subpath in a Time Cluster (in Watts)
   * \param subpathDelayInTimeCluster number of SubPaths in each Time Cluster (in ns)
   * \param timeClusterPowers Normalized Power of the Time Clusters (in Watts)
   * \param sigmaSubpath shadowing of each Subpath (in dB)
   * \param subpathGamma the decay constant of each Subpath (in ns)
   * \param los the value holding if the channel condition is Los or Nlos
   * \return the Normalized Power of each Subpath in a Time Cluster (in Watts)
   */
  MatrixBasedChannelModel::Double2DVector GetSubpathPowers (MatrixBasedChannelModel::Double2DVector subpathDelayInTimeCluster,
                                                            MatrixBasedChannelModel::DoubleVector timeClusterPowers, 
                                                            double sigmaSubpath,
                                                            double subpathGamma, 
                                                            bool los) const;

  /**
   * Get the Absolute propagation time of each subpath
   * \param distance2D the 2D distance between Tx and Rx nodes
   * \param delayOfTimeCluster the delay of each Time Cluster (in ns)
   * \param subpathDelayInTimeCluster the subpath delay in each Time Cluster (in ns)
   * \return the absolute propagation time of each subpath in a Time Cluster (in ns)
   */
  MatrixBasedChannelModel::Double2DVector GetAbsolutePropagationTimes (double distance2D, 
                                                                      MatrixBasedChannelModel::DoubleVector delayOfTimeCluster,
                                                                      MatrixBasedChannelModel::Double2DVector subpathDelayInTimeCluster) const;

/**
   * Get the Mapping of each Subpath and the Azimuth and Elevation angles w.r.t to the Spatial Lobe
   * \param numberOfSpatialLobes the number of Spatial Lobes
   * \param numberOfSubpathInTimeCluster the number of subpaths in each Time Cluster
   * \param mean the mean angle of the Spatial Lobe (in degrees)
   * \param sigma the standard deviation of the mean of the Spatial Lobe (in degrees)
   * \param stdRMSLobeElevationSpread the standard deviation of the elevation offset from the lobe centroid (in degrees)
   * \param stdRMSLobeAzimuthSpread the standard deviation of the azimuth offset from the lobe centroid (in degrees)
   * \param azimuthDistributionType the distribution of azimuth angles of Subpaths
   * \param elevationDistributionType the distribution of elevations angles of Subpaths
   * \return the Time Cluster ID, Subpath ID, Spatial Lobe ID, Azimuth angle of the Subpaths, Elevation angle of the Subpath
   */
  MatrixBasedChannelModel::Double2DVector GetSubpathMappingAndAngles (int numberOfSpatialLobes,
                                                                      MatrixBasedChannelModel::DoubleVector numberOfSubpathInTimeCluster,
                                                                      double mean, 
                                                                      double sigma,
                                                                      double stdRMSLobeElevationSpread,
                                                                      double stdRMSLobeAzimuthSpread, 
                                                                      std::string azimuthDistributionType, 
                                                                      std::string elevationDistributionType) const;
  /**
   * Create a database for the Subpath characteristics :- Time (in ns), Phase (in degrees), Power (in Watts), AOD (in degree), ZOD (in degree), AOA (in degree) and ZOA (in degree)
   * \param numberOfSubpathInTimeCluster the number of Subpaths in each Time Cluster
   * \param absoluteSubpathdelayinTimeCluster the abosulute delay of each Subpath (in ns)
   * \param subpathPower the normalized subpath power (in Watts)
   * \param subpathPhases the subpath phases
   * \param subpathAodZod the AOD and ZOD of the subpath
   * \param subpathAoaZoa the AOA and ZOA of the subpath
   * \return SP Absolute Delay(in ns), Power (rel to 1mW), Phase (radians), AOD, ZOD, AOA, ZOA (all in degrees), AOD Spatial Lobe, AOA Spatial Lobe
   */
  MatrixBasedChannelModel::Double2DVector GetPowerSpectrum (MatrixBasedChannelModel::DoubleVector numberOfSubpathInTimeCluster,
                                                            MatrixBasedChannelModel::Double2DVector absoluteSubpathdelayinTimeCluster,
                                                            MatrixBasedChannelModel::Double2DVector subpathPower,
                                                            MatrixBasedChannelModel::Double2DVector subpathPhases,
                                                            MatrixBasedChannelModel::Double2DVector subpathAodZod,
                                                            MatrixBasedChannelModel::Double2DVector subpathAoaZoa) const;
  
  /**
   * Combine generated subpaths depending on the RF Bandwidth. Wider bands have greater subpath resolution when compared to narrow bands.
   * \param powerSpectrumOld the subpath charactersitcs - Absolute Delay(in ns), Power (rel to 1mW), Phase (radians), AOD, ZOD, AOA, ZOA (all in degrees), AOD Spatial Lobe, AOA Spatial Lobe
   * \param rfBandwidth the RF Bandwidth of operation
   * \param los the channel condition is either Los/Nlos
   * \return the final number of resolvable subpaths
   */
  MatrixBasedChannelModel::Double2DVector GetBWAdjustedtedPowerSpectrum (MatrixBasedChannelModel::Double2DVector powerSpectrumOld,
                                                                        double rfBandwidth, 
                                                                        bool los) const;

  /**
   * The first subpath in LOS is aligned - this implies that AOD and AOA are aligned , ZOD and ZOA are aligned.
   * \param powerSpectrum the subpath charactersitcs after bandwidth adjustment
   * \param los the value indicating if channel is Los/Nlos
   * \return the PowerSpectrum aligned for LOS
   */
  MatrixBasedChannelModel::Double2DVector GetLosAlignedPowerSpectrum (MatrixBasedChannelModel::Double2DVector &powerSpectrum,
                                                                      bool los) const;
  
  /**
   * Remove the subpaths with weak power
   * \param powerSpectrum the subpath charactersitcs adjusted as per RF Bandwidth
   * \param pwrthreshold the miminum detectable subpath power
   * \return PowerSpectrum having only the strong subpaths
   */
  MatrixBasedChannelModel::Double2DVector GetValidSubapths (MatrixBasedChannelModel::Double2DVector powerSpectrum,
                                                            double pwrthreshold) const;

  /**
   * Get the XPD for each ray in the final PowerSpectrum
   * \param totalNumberOfSubpaths the number of subpath in each Time Cluster
   * \param xpdMean the mean value of the XPD
   * \param xpdSd the standard deviation of the XPD
   * \return the XPD value of each subpath in each Time Cluster
   */
  MatrixBasedChannelModel::Double2DVector GetXpdPerSubpath (double totalNumberOfSubpaths,
                                                            double xpdMean, 
                                                            double xpdSd) const;
  
  /**
   * Convert Power in dB scale to linear scale
   * \param pwrdB the power in dB scale
   * \return the power in linear scale
   */
  double GetDbToPow (double pwrdB) const;

  /**
   * Convert the Subpath AOD,ZOD,AOA,ZOA generated in degrees using the NYU Cordinate System (NYUCS) to Global Cordinate System (GCS)
   * in degrees and transform the subpath AOD,ZOD,AOA,ZOA from degrees to radians.
   * \param powerSpectrum the databse used to fetch the AOD,ZOD,AOA,ZOA in degrees for each Subpath
   * \return SP AOD,ZOD,AOA,ZOA in radians w.r.t GCS 
   */
  MatrixBasedChannelModel::Double2DVector NYUCordinateSystemToGlobalCordinateSystem (
      MatrixBasedChannelModel::Double2DVector powerSpectrum) const;

  /**
   * Fetch the minimum detectable power in dB
   * \param distance2D the 2d distance between the TX and RX
   * \return the minimum power that can be detected by the NYU Channel Sounder
   */
  double DynamicRange (double distance2D) const;

private:
  
  /**
   * Extends the struct ChannelMatrix by including information that are used 
   * within the class NYUChannelModel
   */
  struct NYUChannelMatrix : public MatrixBasedChannelModel::ChannelMatrix
  {
    Ptr<const ChannelCondition> m_channelCondition; //!< the channel condition
    int numberOfTimeClusters = 0; //!< value containing the number of Time Clusters
    int numberOfAoaSpatialLobes = 0; //!< value containing the number of AOA Spatial Lobes
    int numberOfAodSpatialLobes = 0; //!< value containing the number of AOD Spatial Lobes
    int totalSubpaths = 0; //!< value containing the total number of Subpaths
    MatrixBasedChannelModel::DoubleVector numberOfSubpathInTimeCluster; //!< value containing the number of Subpaths in each time cluster
    MatrixBasedChannelModel::DoubleVector delayOfTimeCluster; //!< value containing the delay of each time cluster
    MatrixBasedChannelModel::DoubleVector timeClusterPowers; //!< value containing the power of each time cluster
    MatrixBasedChannelModel::DoubleVector rayAodRadian; //!< the vector containing AOD angles
    MatrixBasedChannelModel::DoubleVector rayAoaRadian; //!< the vector containing AOA angles
    MatrixBasedChannelModel::DoubleVector rayZodRadian; //!< the vector containing ZOD angles
    MatrixBasedChannelModel::DoubleVector rayZoaRadian; //!< the vector containing ZOA angles
    MatrixBasedChannelModel::Double2DVector subpathDelayInTimeCluster; //!< value containing delay of each subpath in each time cluster
    MatrixBasedChannelModel::Double2DVector subpathPhases; //!< value containig the Subpath phases of each each SP in each time cluster
    MatrixBasedChannelModel::Double2DVector subpathPowers; //!< value containing the power of each Subpath in each time cluster
    MatrixBasedChannelModel::Double2DVector absoluteSubpathDelayinTimeCluster; //!< value containing the absolute delay of each subpath in each time cluster
    MatrixBasedChannelModel::Double2DVector subpathAodZod; //!< value containing the mapping(SP,TC,Lobe) and Subpath angles(Azimuth,Elevation) of AOD Lobe
    MatrixBasedChannelModel::Double2DVector subpathAoaZoa; //!< value containing the mapping(SP,TC,Lobe) and Subpath angles(Azimuth,Elevation) of AOA Lobe
    MatrixBasedChannelModel::Double2DVector powerSpectrumOld; //!< value containing SP characteristics: AbsoluteDelay(in ns),Power (relative to 1mW),Phases (radians),AOD (in degrees),ZOD (in degrees),AOA (in degrees),ZOA (in degrees)
    MatrixBasedChannelModel::Double2DVector powerSpectrum; //!<value containg SP characteristics - Adjusted according to RF bandwidth
    MatrixBasedChannelModel::Double2DVector xpd; //!< value containing the XPD (Cross Polarization Discriminator) in dB for each Ray
  };

  /**
   * Data structure that stores the parameters of 3GPP TR 38.901, Table 7.5-6,
   * for a certain scenario
   */
  struct ParamsTable : public SimpleRefCount<ParamsTable>
  {
    /******** NYU Channel Parameters ************/
    // common parameters for UMi,UMa,RMa,InH and InF
    double muAod = 0; //!<Max num of AOD Spatial Lobes
    double muAoa = 0; //!<Max num of AOA Spatial Lobes
    double minimumVoidInterval = 0; //!<minVoidInterval Time in ns
    double sigmaCluster = 0; //!<Per-cluster shadowing in dB
    double timeClusterGamma = 0; //!<Time cluster decay constant in ns
    double sigmaSubpath = 0; //!<per subpath shadowing in dB
    double subpathGamma = 0; //!<subpath decay constant in ns
    double meanZod = 0; //!<Mean zenith angle of departure (ZOD) in degrees
    double sigmaZod = 0; //!<Standard deviation of the ZOD distribution in degrees
    double sdOfAodRmsLobeAzimuthSpread = 0; //!<Standard deviation of the azimuth offset from the lobe centroid in degrees
    double sdOfAodRmsLobeElevationSpread = 0; //!<Standard deviation of the elevation offset from the lobe centroid in degrees
    std::string aodRmsLobeAzimuthSpread; //!<string specifying which distribution to use: 'Gaussian' or 'Laplacian
    std::string aodRmsLobeElevationSpread; //!<string specifying which distribution to use: 'Gaussian' or 'Laplacian
    double meanZoa = 0; //!<Mean zenith angle of arrival (ZOA) in degrees
    double sigmaZoa = 0; //!<Standard deviation of the ZOA distribution in degrees
    double sdOfAoaRmsLobeAzimuthSpread = 0; //!<Standard deviation of the azimuth offset from the lobe centroid in degrees
    double sdOfAoaRmsLobeElevationSpread = 0; //!<Standard deviation of the elevation offset from the lobe centroid
    std::string aoaRmsLobeAzimuthSpread; //!<A string specifying which distribution to use: 'Gaussian' or 'Laplacian
    std::string aoaRmsLobeElevationSpread; //!<A string specifying which distribution to use: 'Gaussian' or 'Laplacian
    bool los; //!<boolean value indicating whether the channel condition is LOS or NLOS
    double xpdMean = 0; //!<Mean of XPD value
    double xpdSd = 0; //!< standard deviation of XPD value
    // common parameters for for UMi,UMa and RMa
    double maxNumberOfTimeCluster = 0; //!<Max number of Time Clusters
    double maxNumberOfSubpaths = 0; //!<Max number of Subpaths
    // common parameters for for UMi,UMa,RMa and InH
    double muTau = 0; //!<Mean excess Delay in ns
    double muRho = 0; //!<Intra cluster Delay in ns
    // common parameters for for InH, InF
    double lambdaC = 0; //!< Mean number of time clusters
    double betaS = 0; //!<Scaling factor for mean number of cluster sub-paths
    // parameters specific to InF
    double kS = 0;//!<the shape of the number of cluster sub-paths
    double sigmaS = 0;//!<the scale factor for the number of cluster sub-paths
    double thethaS = 0;//!<the bound for the number of cluster sub-paths
    double alphaTau = 0; 
    double betaTau = 0;
    double alphaRho = 0; //!<the alpha value for the gamma distribution for intra cluster subpath delay (in ns)
    double betaRho = 0; //!<the beta value for the gamma distribution for intra cluster subpath delay (in ns)
    // parameters specific to InH
    double muS = 0; //!<Mean number of cluster sub-paths
  };

  /**
   * Get the parameters needed to apply the channel generation procedure
   * \param channelCondition the channel condition
   * \return the parameters table
   */
  virtual Ptr<const ParamsTable> GetNYUTable (Ptr<const ChannelCondition> channelCondition) const;

  /**
   * Compute the channel matrix between two devices
   * \param channelCondition the channel condition
   * \param sAntenna the s node antenna array
   * \param uAntenna the u node antenna array
   * \param uAngle the u node angle
   * \param sAngle the s node angle
   * \param dis2D the 2D distance between tx and rx
   * \return the channel realization
   */
  Ptr<NYUChannelMatrix> GetNewChannel (Ptr<const ChannelCondition> channelCondition,
                                        Ptr<const PhasedArrayModel> sAntenna,
                                        Ptr<const PhasedArrayModel> uAntenna,
                                        Angles &uAngle, 
                                        Angles &sAngle,
                                        double dis2D) const;

  /**
   * Check if the channel matrix has to be updated
   * \param channelMatrix channel matrix
   * \param channelCondition the channel condition
   * \return true if the channel matrix has to be updated, false otherwise
   */
  bool ChannelMatrixNeedsUpdate (Ptr<const NYUChannelMatrix> channelMatrix, Ptr<const ChannelCondition> channelCondition) const;

  std::unordered_map<uint32_t, Ptr<NYUChannelMatrix> > m_channelMap; //!< map containing the channel realizations
  Time m_updatePeriod; //!< the channel update period
  double m_frequency; //!< the operating frequency
  double m_rfBandwidth; //!< the operating rf bandwidth in Hz 
  std::string m_scenario; //!< the 3GPP scenario
  Ptr<ChannelConditionModel> m_channelConditionModel; //!< the channel condition model
  Ptr<UniformRandomVariable> m_uniformRv; //!< uniform random variable
  Ptr<NormalRandomVariable> m_normalRv; //!< normal random variable
  Ptr<ExponentialRandomVariable> m_expRv;//!< exponential random variable
  // parameters for the blockage model
  bool m_blockage;//!< enables the blockage  
};
} // namespace ns3

#endif /* NYU_CHANNEL_H */
