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
  void SetFrequency (double f);

  /**
   * Returns the center frequency
   * \return the center frequency in Hz
   */
  double GetFrequency (void) const;

  /**
   * Sets the RF Bandwidth of the model
   * \param rf_bw the RF Bandwidth in Hz
   */
  void SetRFBandWidth (double rf_bw);
  
  /**
   * Returns the RF Bandwidth of the model
   * \return the RF Bandwidth in Hz
   */
  double GetRFBandWidth (void) const;

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
   * \param val1 value of a parameter at 28 GHz
   * \param val2 value of a parameter at 140 GHz
   * \return the value of a parameter at any frequency between 28-140 GHz
   */
  double calpar(double val1, double val2, double frequency) const;

  /**
   * Find maximum value between two given values
   * \param val1 first number
   * \param val2 second number
   * \return the Maximum number between first and second number
   */
  double GetMaxValue (double val1, double val2) const;

  /**
   * Find minimum value among two given values
   * \param val1 first number
   * \param val2 second number
   * \return the Minimum number between first and second number
   */
  double GetMinValue (double val1, double val2) const;

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
   * \param min lower bound of the discrete uniform distribution
   * \param max upper bound of the discrete uniform distribution
   * \return a random integer value from a discrete uniform distribution
   */  
  int GetDUniformDist (const double min, const double max) const;

  /**
   * Generate a random value (integer/double) following a uniform distribution
   * \param min lower bound of the uniform distribution
   * \param max upper bound of the uniform distribution
   * \return a random value(integer/double) from a uniform distribution
   */  
  double GetUniformDist (const double min, const double max) const;

  /**
   * Generate a random value following an exponential distribution
   * \param lambda mean of the exponential distribution
   * \return a random value(int/double) value from an exponential distribution
   */  
  double GetExpDist (double lambda) const;

  /**
   * Get the number of Time Clusters. Time clusters follow a Poisson Distribution
   * \param lambda_c mean value of the number of time cluster 
   * \return the number of time clusters
   */  
  int GetNumTC (double lambda_c) const;

  /**
   * Get the number of AOA Spatial Lobes
   * \param mu_AOA mean value of the number of AOA Spatial Lobes
   * \return the number of AOA Spatial Lobes
   */  
  int GetNumAOALobes (double mu_AOA) const;

  /**
   * Get the number of AOD Spatial Lobes
   * \param mu_AOD mean value of the number of AOD Spatial Lobes
   * \return the number of AOD Spatial Lobes
   */  
  int GetNumAODLobes (double mu_AOD) const;

  /**
   * Get the number of Subpaths in each Time Cluster
   * \param num_TC number of Time Clusters
   * \return the number of Subpaths in each Time Cluster
   */ 
  MatrixBasedChannelModel::DoubleVector GetNumSPinTC (int num_TC) const;

  /**
   * Get the Subpath delay in each Time Cluster
   * \param numSPinTC number of subpath in each time cluster
   * \param mu_rho mean subpath delay in each time cluster (ns)
   * \return the number of Subpaths in each Time Cluster
   */  
  MatrixBasedChannelModel::Double2DVector GetIntraClusterDelays(MatrixBasedChannelModel::DoubleVector numSPinTC, double mu_rho) const;

  /**
   * Get the Subpath phases in each Time Cluster
   * \param numSPinTC number of subpath in each time cluster
   * \return the phases of each subpath in each Time Cluster
   */ 
  MatrixBasedChannelModel::Double2DVector GetSubpathPhases(MatrixBasedChannelModel::DoubleVector numSPinTC) const;

  /**
   * Get the Delay of each Time Cluster
   * \param mu_tau mean excess delay of each time cluster
   * \param SPdelayinTC subpath delay in each time cluster
   * \param minVoidInterval mimumum time in ns by which two time clusters are seperated
   * \return the delay of each Time Cluster
   */ 
  MatrixBasedChannelModel::DoubleVector GetClusterExcessTimeDelays(double mu_tau, 
                                                                  MatrixBasedChannelModel::Double2DVector SPdelayinTC,
                                                                  double minVoidInterval) const;

  /**
   * Get the Power of each Time Cluster
   * \param GetClusterExcessTimeDelays mean excess delay of each time cluster (in ns)
   * \param Pr_dBm total received omnidirectional power (dBm)
   * \param sigmaCluster shadowing in each time cluster (dB)
   * \param tc_gamma time cluster decay constant (ns)
   * \return the power in each time cluster
   */ 
  MatrixBasedChannelModel::DoubleVector GetClusterPowers(MatrixBasedChannelModel::DoubleVector GetClusterExcessTimeDelays, 
                                                         double sigmaCluster,
                                                         double tc_gamma) const;

  /**
   * Get the Power of each Time Cluster
   * \param numSPinTC number of SubPaths in each Time Cluster
   * \param TCPowers Power of the Time Clusters
   * \param sigmaSubpath shadowing of each Subpath (dB)
   * \param sp_gamma the decay constant of each Subpath (ns)
   * \return the power in each time cluster
   */ 
  MatrixBasedChannelModel::Double2DVector GetSubPathPowers(MatrixBasedChannelModel::Double2DVector SPdelayinTC, 
                                                           MatrixBasedChannelModel::DoubleVector TCPowers,
                                                           double sigmaSubpath,
                                                           double sp_gamma,
                                                           bool los) const;

  /**
   * Get the Power of each Time Cluster
   * \param distance2D 2D distance between Tx and Rx nodes. Tx is fixed and Rx is moving
   * \param TCExcessDelay Delay of each time cluster
   * \param SPdelayinTC Subpath delay in each time cluster
   * \return the absolute delay time of each subpath in a time cluster (in ns)
   */ 
  MatrixBasedChannelModel::Double2DVector getAbsolutePropTimes(double distance2D,
                                                               MatrixBasedChannelModel::DoubleVector TCExcessDelay,
                                                               MatrixBasedChannelModel::Double2DVector SPdelayinTC) const;

  /**
   * Get the Mapping of each Subpath and the Azimuth and Elevation angles w.r.t to the Spatial Lobe
   * \param numLobes Number of Spatial Lobes
   * \param numSPinTC Number of Subpaths in each Time Cluster
   * \param mean mean angle of the spatial lobe (in degrees)
   * \param sigma standard deviation of the mean of the spatial lobe (in degrees)
   * \param std_RMSLobeElevationSpread standard deviation of the elevation offset from the lobe centroid (in degrees)
   * \param std_RMSLobeAzimuthSpread standard deviation of the azimuth offset from the lobe centroid (in degrees)
   * \param distributionType Distribution type of SP in Spatial Lobes
   * \return the Time Cluster ID, Subpath ID, Spatial Lobe ID, Azimuth angle of the SP, Elevation angle of the SP
   */ 
  MatrixBasedChannelModel::Double2DVector GetSubpathMappingandAngles(int numLobes,
                                                                    MatrixBasedChannelModel::DoubleVector numSPinTC,
                                                                    double mean,
                                                                    double sigma,
                                                                    double std_RMSLobeElevationSpread,
                                                                    double std_RMSLobeAzimuthSpread,
                                                                    std::string distributionType) const;
  /**
   * Get the Mapping of each Subpath and the Azimuth and Elevation angles w.r.t to the Spatial Lobe
   * \param numSPinTC Number of Subpaths in each Time Cluster
   * \param absoluteSPdelayinTC 
   * \param SPPowers 
   * \param SPPhases 
   * \param AOD_cluster_subpath_lobe_az_elev_angles 
   * \param AOA_cluster_subpath_lobe_az_elev_angles 
   * \return SP Absolute Delay(in ns), Power (rel to 1mW), Phase (radians), AOD, ZOD, AOA, ZOA (all in degrees), AOD Spatial Lobe, AOA Spatial Lobe
   */                                                                   
  MatrixBasedChannelModel::Double2DVector GetPowerSpectrum(MatrixBasedChannelModel::DoubleVector numSPinTC,
                                                          MatrixBasedChannelModel::Double2DVector absoluteSPdelayinTC,
                                                          MatrixBasedChannelModel::Double2DVector SPPowers,
                                                          MatrixBasedChannelModel::Double2DVector SPPhases,
                                                          MatrixBasedChannelModel::Double2DVector AOD_cluster_subpath_lobe_az_elev_angles,
                                                          MatrixBasedChannelModel::Double2DVector AOA_cluster_subpath_lobe_az_elev_angles)const;
  
  /**
   * For a given RF BW not all Subpaths can be distinctly resolved. Fetch the SP characteristics of the combined SubPaths
   * \param PowerSpectrumOld SP charactersitcs - Absolute Delay(in ns), Power (rel to 1mW), Phase (radians), AOD, ZOD, AOA, ZOA (all in degrees), AOD Spatial Lobe, AOA Spatial Lobe
   * \param RFBandwidth RF Banwidth
   * \param los set if channel is LOS
   * \return SP characteristics of merged Subpaths
   */
  MatrixBasedChannelModel::Double2DVector GetBWAdjustedtedPowerSpectrum(MatrixBasedChannelModel::Double2DVector PowerSpectrumOld, 
                                                                        double RFBandwidth,
                                                                        bool los) const;

  /**
   * Align the SP for LOS
   * \param PowerSpectrum SP charactersitcs adjusted as per RF Bandwidth
   * \param Scenario indicates whether we are in UMi,UMa,RMa,InH, Factory
   * \return PowerSpectrum aligned for LOS
   */
  MatrixBasedChannelModel::Double2DVector GetLosAlignedPowerSpectrum(MatrixBasedChannelModel::Double2DVector &PowerSpectrum, 
                                                                    bool los) const;
  
  /**
   * Remove the subpaths with weak power
   * \param PowerSpectrum SP charactersitcs adjusted as per RF Bandwidth
   * \param pwrthreshold miminum detectable subpath power
   * \return PowerSpectrum having only the strong subpaths
   */
  MatrixBasedChannelModel::Double2DVector GetValidSubapths(MatrixBasedChannelModel::Double2DVector PowerSpectrum, 
                                                          double pwrthreshold) const;

  /**
   * Get the XPD for each ray in the final PowerSpectrum
   * \param totalNumSP number of subpath in each time cluster
   * \param XPD_Mean mean value of the XPD
   * \param XPD_Sd standard deviation of the XPD
   * \return the XPD value of each subpath in each Time Cluster
   */ 
  MatrixBasedChannelModel::Double2DVector GetXPDperRay(double totalNumSP,
                                                      double XPD_Mean,
                                                      double XPD_Sd) const;
  
  /**
   * Convert Power in dB scale to linear scale
   * \param pwr_dB power in dB scale
   * \return the power in linear scale
   */   
  double Getdb2pow(double pwr_dB) const;

  /**
   * Convert the Subpath AOD,ZOD,AOA,ZOA generated in degrees using the NYU Cordinate System (NYUCS) to Global Cordinate System (GCS)
   * in degrees and transform the subpath AOD,ZOD,AOA,ZOA from degrees to radians.
   * \param PowerSpectrum used to fetch the AOD,ZOD,AOA,ZOA in degrees for each Subpath
   * \return SP AOD,ZOD,AOA,ZOA in radians w.r.t GCS 
   */
  MatrixBasedChannelModel::Double2DVector  NYUCStoGCS(MatrixBasedChannelModel::Double2DVector PowerSpectrum) const;

  /**
   * Fetch the minimum detectable power in dB
   * \param distance 2d distance between the TX and RX
   * \return the minimum power that can be detected by the NYU Channel Sounder
   */
  double dynamic_range (double distance2D) const;

private:
  
  /**
   * Extends the struct ChannelMatrix by including information that are used 
   * within the class NYUChannelModel
   */
  struct NYUChannelMatrix : public MatrixBasedChannelModel::ChannelMatrix
  {
    Ptr<const ChannelCondition> m_channelCondition; //!< the channel condition
    int numTC = 0; //!< value containing the number of Time Clusters
    int numAOALobes = 0; //!< value containing the number of AOA Spatial Lobes
    int numAODLobes = 0; //!< value containing the number of AOD Spatial Lobes
    int totalSubpaths = 0; //!< value containing the total number of Subpaths

    MatrixBasedChannelModel::DoubleVector numSPinTC; //!< value containing the number of Subpaths in each time cluster
    MatrixBasedChannelModel::DoubleVector TCExcessDelay; //!< value containing the delay of each time cluster
    MatrixBasedChannelModel::DoubleVector TCPowers;//!< value containing the power of each time cluster
    MatrixBasedChannelModel::DoubleVector rayAodRadian; //!< the vector containing AOD angles
    MatrixBasedChannelModel::DoubleVector rayAoaRadian; //!< the vector containing AOA angles
    MatrixBasedChannelModel::DoubleVector rayZodRadian; //!< the vector containing ZOD angles
    MatrixBasedChannelModel::DoubleVector rayZoaRadian; //!< the vector containing ZOA angles

    MatrixBasedChannelModel::Double2DVector SPdelayinTC; //!< value containing delay of each subpath in each time cluster
    MatrixBasedChannelModel::Double2DVector SPPhases; //!< value containig the Subpath phases of each each SP in each time cluster
    MatrixBasedChannelModel::Double2DVector SPPowers; //!< value containing the power of each Subpath in each time cluster
    MatrixBasedChannelModel::Double2DVector absoluteSPdelayinTC;//!< value containing the absolute delay of each subpath in each time cluster
    MatrixBasedChannelModel::Double2DVector AOD_cluster_subpath_lobe_az_elev_angles; //!< value containing the mapping(SP,TC,Lobe) and Subpath angles(Azimuth,Elevation) of AOD Lobe
    MatrixBasedChannelModel::Double2DVector AOA_cluster_subpath_lobe_az_elev_angles; //!< value containing the mapping(SP,TC,Lobe) and Subpath angles(Azimuth,Elevation) of AOA Lobe
    MatrixBasedChannelModel::Double2DVector PowerSpectrumOld;//!< value containing SP characteristics: AbsoluteDelay(in ns),Power (relative to 1mW),Phases (radians),AOD (in degrees),ZOD (in degrees),AOA (in degrees),ZOA (in degrees)
    MatrixBasedChannelModel::Double2DVector PowerSpectrum;//!<value containg SP characteristics - Adjusted according to RF bandwidth
    MatrixBasedChannelModel::Double2DVector XPD; //!< value containing the XPD (Cross Polarization Discriminator) in dB for each Ray
  };

  /**
   * Data structure that stores the parameters of 3GPP TR 38.901, Table 7.5-6,
   * for a certain scenario
   */
  struct ParamsTable : public SimpleRefCount<ParamsTable>
  {
    /******** NYU Channel Parameters ************/
    // Common parameters for UMi,UMa,RMa,InH and InF
    double mu_AOD = 0;//!<Max num of AOD Spatial Lobes
    double mu_AOA = 0;//!<Max num of AOA Spatial Lobes
    double minVoidInterval = 0; //!<minVoidInterval Time in ns 
    double sigmaCluster = 0;//!<Per-cluster shadowing in dB
    double tc_gamma = 0;//!<Time cluster decay constant in ns
    double sigmaSubpath = 0;//!<per subpath shadowing in dB
    double sp_gamma = 0;//!<subpath decay constant in ns
    double mean_ZOD = 0;//!<Mean zenith angle of departure (ZOD) in degrees
    double sigma_ZOD = 0;//!<Standard deviation of the ZOD distribution in degrees
    double std_AOD_RMSLobeAzimuthSpread = 0;//!<Standard deviation of the azimuth offset from the lobe centroid in degrees
    double std_AOD_RMSLobeElevationSpread = 0;//!<Standard deviation of the elevation offset from the lobe centroid in degrees
    std::string AOD_RMSLobeAzimuthSpread;//!<string specifying which distribution to use: 'Gaussian' or 'Laplacian
    std::string AOD_RMSLobeElevationSpread;//!<string specifying which distribution to use: 'Gaussian' or 'Laplacian
    double mean_ZOA = 0;//!<Mean zenith angle of arrival (ZOA) in degrees
    double sigma_ZOA = 0;//!<Standard deviation of the ZOA distribution in degrees
    double std_AOA_RMSLobeAzimuthSpread = 0;//!<Standard deviation of the azimuth offset from the lobe centroid in degrees
    double std_AOA_RMSLobeElevationSpread = 0;//!<Standard deviation of the elevation offset from the lobe centroid
    std::string AOA_RMSLobeAzimuthSpread;//!<A string specifying which distribution to use: 'Gaussian' or 'Laplacian
    std::string AOA_RMSLobeElevationSpread;//!<A string specifying which distribution to use: 'Gaussian' or 'Laplacian
    bool los; //!<boolean value indicating whether the channel condition is LOS or NLOS
    double XPD_Mean = 0;//!<Mean of XPD value 
    double XPD_Sd = 0; //!< standard deviation of XPD value
    // common for UMi,UMa and RMa
    double max_c = 0;//!<Max number of Time Clusters
    double max_s = 0;//!<Max number of Subpaths
    double mu_rho = 0;//!<Intra cluster Delay in ns
    double mu_tau = 0;//!<Mean excess Delay in ns
    // common for InH, InF
    double lambda_c = 0;  //!< Mean number of time clusters 
    double beta_s = 0;//!<Scaling factor for mean number of cluster sub-paths
    // specific to InF
    double k_s = 0;
    double sigma_s = 0;
    double thetha_s = 0;
    double alpha_tau = 0;
    double beta_tau = 0;
    double alpha_rho = 0;
    double beta_rho = 0;
    // specific to InH
    double mu_s = 0;//!<Mean number of cluster sub-paths


    // to be removed
    std::string distributionType_AOA;
    std::string distributionType_AOD;
  };

  /**
   * Get the parameters needed to apply the channel generation procedure
   * \param channelCondition the channel condition
   * \return the parameters table
   */
  virtual Ptr<const ParamsTable> GetNYUTable (Ptr<const ChannelCondition> channelCondition) const;

  /**
   * Compute the channel matrix between two devices
   * \param locUT the location of the UT
   * \param channelCondition the channel condition
   * \param sAntenna the s node antenna array
   * \param uAntenna the u node antenna array
   * \param uAngle the u node angle
   * \param sAngle the s node angle
   * \param dis2D the 2D distance between tx and rx
   * \param hBS the height of the BS
   * \param hUT the height of the UT
   * \return the channel realization
   */
  Ptr<NYUChannelMatrix> GetNewChannel (Ptr<const ChannelCondition> channelCondition,
                                        Ptr<const PhasedArrayModel> sAntenna,
                                        Ptr<const PhasedArrayModel> uAntenna,
                                        Angles &uAngle, Angles &sAngle,
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
  double m_rfbw; //!< the operating rf bandwidth in Hz 
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
