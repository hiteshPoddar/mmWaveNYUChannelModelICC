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

#ifndef NYU_PROPAGATION_LOSS_MODEL_H
#define NYU_PROPAGATION_LOSS_MODEL_H

#include "ns3/propagation-loss-model.h"
#include "ns3/channel-condition-model.h"
#include "ns3/string.h"

namespace ns3 {

/**
* \brief Base class for the NYU propagation models
*/
class NYUPropagationLossModel : public PropagationLossModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * Constructor
   */
  NYUPropagationLossModel ();

  /**
   * Destructor
   */
  virtual ~NYUPropagationLossModel () override;

  /**
   * \brief Set the channel condition model used to determine the channel
   *        state (e.g., the LOS/NLOS condition)
   * \param model pointer to the channel condition model
   */
  void SetChannelConditionModel (Ptr<ChannelConditionModel> model);

  /**
   * \brief Returns the associated channel condition model
   * \return the channel condition model
   */
  Ptr<ChannelConditionModel> GetChannelConditionModel (void) const;

  /**
   * \brief Set the central frequency of the model
   * \param f the central frequency in the range in Hz, between 500.0e6 and 100.0e9 Hz
   */
  void SetFrequency (double f);

  /**
   * \brief Return the current central frequency
   * \return The current central frequency
   */
  double GetFrequency (void) const;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  NYUPropagationLossModel (const NYUPropagationLossModel &) = delete;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \returns the NYUPropagationLossModel instance
   */
  NYUPropagationLossModel & operator = (const NYUPropagationLossModel &) = delete;

  // foliage loss 
  void SetFoliageloss (double foli_loss);
  double GetFoliageloss (void) const;

  // O2I Loss type
  void SetO2Ilosstype(const std::string &value);
  std::string GetO2Ilosstype (void) const;

  // Total PL for O2I depending on High Loss or Low Loss NYU O2I Model
  double GetO2ILoss (const std::string &o2ilosstype, double frequency) const;
  // Total PL for FoliageLoss (per_meter_foliage_loss * total_distance = db)
  double GetTotalFoliageLoss (double distance2d) const;

  // Max measurable PL by NYU Channel Sounder
  virtual double RxPower_threshold (double txPowerDbm,
									Ptr<MobilityModel> a,
									Ptr<MobilityModel> b) const;  
	
  double calPar(double ple1, double ple2, double frequency) const;
	

private:
  /**
   * Computes the received power by applying the pathloss models
   *
   * \param txPowerDbm tx power in dBm
   * \param a tx mobility model
   * \param b rx mobility model
   * \return the rx power in dBm
   */
  virtual double DoCalcRxPower (double txPowerDbm,
                                Ptr<MobilityModel> a,
                                Ptr<MobilityModel> b) const override;

  /**
   * If this  model uses objects of type RandomVariableStream,
   * set the stream numbers to the integers starting with the offset
   * 'stream'. Return the number of streams (possibly zero) that
   * have been assigned.
   *
   * \param stream
   * \return the number of stream indices assigned by this model
   */
  virtual int64_t DoAssignStreams (int64_t stream) override;
  
  double GetLoss (Ptr<ChannelCondition> cond, double distance2D, double distance3D, double hUt, double hBs) const;
  
  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is not obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  virtual double GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const = 0;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  virtual double GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const = 0;
  
  /**
   * \brief Determines hUT and hBS. The default implementation assumes that
   *        the tallest node is the BS and the smallest is the UT. The derived classes
   * can change the default behavior by overriding this method.
   * \param za the height of the first node in meters
   * \param zb the height of the second node in meters
   * \return std::pair of heights in meters, the first element is hUt and the second is hBs
   */
  virtual std::pair<double, double> GetUtAndBsHeights (double za, double zb) const;

  /**
   * \brief Retrieves the shadowing value by looking at m_shadowingMap.
   *        If not found or if the channel condition changed it generates a new
   *        independent realization and stores it in the map, otherwise it correlates
   *        the new value with the previous one using the autocorrelation function
   *        defined in 3GPP TR 38.901, Sec. 7.4.4.
   * \param a tx mobility model
   * \param b rx mobility model
   * \param cond the LOS/NLOS channel condition
   * \return shadowing loss in dB
   */
  double GetShadowing (Ptr<MobilityModel> a, Ptr<MobilityModel> b, ChannelCondition::LosConditionValue cond) const;

  /**
   * \brief Returns the shadow fading standard deviation
   * \param a tx mobility model
   * \param b rx mobility model
   * \param cond the LOS/NLOS channel condition
   * \return shadowing std in dB
   */
  virtual double GetShadowingStd (ChannelCondition::LosConditionValue cond) const = 0;

  /**
   * \brief Returns the shadow fading correlation distance
   * \param cond the LOS/NLOS channel condition
   * \return shadowing correlation distance in meters
   */
  virtual double GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const = 0;

  /**
   * \brief Returns an unique key for the channel between a and b.
   *
   * The key is the value of the Cantor function calculated by using as
   * first parameter the lowest node ID, and as a second parameter the highest
   * node ID.
   *
   * \param a tx mobility model
   * \param b rx mobility model
   * \return channel key
   */
  static uint32_t GetKey (Ptr<MobilityModel> a, Ptr<MobilityModel> b);

  /**
   * \brief Get the difference between the node position
   *
   * The difference is calculated as (b-a) if Id(a) < Id (b), or
   * (a-b) if Id(b) <= Id(a).
   *
   * \param a First node
   * \param b Second node
   * \return the difference between the node vector position
   */
  static Vector GetVectorDifference (Ptr<MobilityModel> a, Ptr<MobilityModel> b);

  double dynamic_range (double distance2D) const;

protected:
  virtual void DoDispose () override; 
  
  /**
  * \brief Computes the 2D distance between two 3D vectors
  * \param a the first 3D vector
  * \param b the second 3D vector
  * \return the 2D distance between a and b
  */
  static double Calculate2dDistance (Vector a, Vector b);

  Ptr<ChannelConditionModel> m_channelConditionModel; //!< pointer to the channel condition model
  double m_frequency; 
  double m_foliageloss;
  std::string m_O2Ilosstype;
  bool m_shadowingEnabled;  
  bool m_foilagelossEnabled;

  Ptr<UniformRandomVariable> m_uniformVar;
  Ptr<NormalRandomVariable> m_normRandomVariable;

  /** Define a struct for the m_shadowingMap entries */
  struct ShadowingMapItem
  {
    double m_shadowing; //!< the shadowing loss in dB
    ChannelCondition::LosConditionValue m_condition; //!< the LOS/NLOS condition
    Vector m_distance; //!< the vector AB
  };

  mutable std::unordered_map<uint32_t, ShadowingMapItem> m_shadowingMap; //!< map to store the shadowing values
};

/**
 * \ingroup propagation
 *
 * \brief Implements the pathloss model defined for the UMi scenario.
 */
class NYUUMiPropagationLossModel : public NYUPropagationLossModel
{
public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);

  /**
   * Constructor
   */
  NYUUMiPropagationLossModel ();

  /**
   * Destructor
   */
  virtual ~NYUUMiPropagationLossModel () override;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   */
  NYUUMiPropagationLossModel (const NYUUMiPropagationLossModel &) = delete;

  /**
   * \brief Copy constructor
   *
   * Defined and unimplemented to avoid misuse
   * \returns the NYUUMiPropagationLossModel instance
   */
  NYUUMiPropagationLossModel & operator = (const NYUUMiPropagationLossModel &) = delete;

private:
  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is not obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  virtual double GetLossLos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Computes the pathloss between a and b considering that the line of
   *        sight is obstructed
   * \param distance2D the 2D distance between tx and rx in meters
   * \param distance3D the 3D distance between tx and rx in meters
   * \param hUt the height of the UT in meters
   * \param hBs the height of the BS in meters
   * \return pathloss value in dB
   */
  virtual double GetLossNlos (double distance2D, double distance3D, double hUt, double hBs) const override;

  /**
   * \brief Returns the shadow fading standard deviation
   * \param a tx mobility model
   * \param b rx mobility model
   * \param cond the LOS/NLOS channel condition
   * \return shadowing std in dB
   */
  virtual double GetShadowingStd (ChannelCondition::LosConditionValue cond) const override;

  /**
   * \brief Returns the shadow fading correlation distance
   * \param cond the LOS/NLOS channel condition
   * \return shadowing correlation distance in meters
   */
  virtual double GetShadowingCorrelationDistance (ChannelCondition::LosConditionValue cond) const override;
  
};


} //namespace ns3
#endif