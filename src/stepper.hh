/*
 * Copyright 2011,
 * Florent Lamiraux
 *
 * CNRS
 *
 * This file is part of sot-test.
 * sot-test is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * sot-test is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with sot-test.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DYNAMICGRAPH_SOT_DYNAMIC_TEST_STEPPER_HH
#define DYNAMICGRAPH_SOT_DYNAMIC_TEST_STEPPER_HH

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/entity.h>
//#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <sot/core/matrix-homogeneous.hh>

namespace dynamicgraph {
  namespace sot {
    namespace dynamic {
      class Stepper : public Entity
      {
      public:
	static const std::string CLASS_NAME;
	virtual const std::string& getClassName (void) const {
	  return CLASS_NAME;
	}

	Stepper(const std::string& inName);
	/// \brief Provide left ankle position
	void setLeftAnklePosition(const Matrix& inLeftAnklePosition);
	/// \brief Provide right ankle position
	void setRightAnklePosition(const Matrix& inRightAnklePosition);
	/// \brief Set position of center of mass
	void setCenterOfMass(const Vector& inCom);
	/// \brief Set width of each foot
	void setFootWidth(const double& inWidth);
	/// \brief Set left foot position before stepping
	void setLeftFootCenter(const Vector& inFootPosition);
	/// \brief Set right foot position before stepping
	void setRightFootCenter(const Vector& inFootPosition);
	/// \brief Start the stepping motion
	void start();

      protected:
	// Signal update
	double& computeComGain(double& comGain, const int& inTime);
	Vector& computeComRef(Vector& comRef, const int& inTime);
	Vector& computeZmpRef(Vector& zmpRef, const int& inTime);
	MatrixHomogeneous& computeLeftAnkle(MatrixHomogeneous&
					    leftAnklePosition,
					   const int& inTime);
	MatrixHomogeneous& computeRightAnkle(MatrixHomogeneous&
					    rightAnklePosition,
					    const int& inTime);

      private:
	double computeFootHeight(const Vector& inFootCenter,
				 const Vector& inZmp);
	double computeMagnitude(double t);

	/// \brief control gain of the com task
	SignalTimeDependent<double, int> comGainSOUT;
	/// \brief Reference of the center of mass
	SignalTimeDependent<Vector, int> comReferenceSOUT;
	/// \brief Reference of the center of pressure
	SignalTimeDependent<Vector, int> zmpReferenceSOUT;
	/// \brief Reference of the left ankle
	SignalTimeDependent<MatrixHomogeneous, int> leftAnkleReferenceSOUT;
	/// \brief Reference of the right ankle
	SignalTimeDependent<MatrixHomogeneous, int> rightAnkleReferenceSOUT;

	/// Position of the left ankle before stepping
	MatrixHomogeneous leftAnklePosition_;
	/// Position of the right ankle before stepping
	MatrixHomogeneous rightAnklePosition_;
	/// Left foot center position before stepping
	Vector leftFootCenter_;
	/// Right foot center position before stepping
	Vector rightFootCenter_;
	// Position of the center of mass before stepping
	Vector centerOfMass_;
	// Half-width of each ankle
	double halfFootWidth_;
	// Time since beginning of stepping
	int startTime_;
	// Whether the motion being executed
	bool stepping_;
	// Magnitude of the center of mass oscillation
	double magnitude_;
	// Pulsation of the center of mass oscillation
	double omega_;
	// Period of the center of mass oscillation
	double comPeriod_;
	// Maximal step height when stepping
	double stepHeight_;
	// Time sampling period
	double timePeriod_;
	// distance between foot centers
	double distance_;
	// unit vector of direction right foot to left foot centers
	Vector unit_;

      }; // class Stepper
    } // namespace dynamic
  } // namespace sot
} // namespace dynamicgraph
#endif // DYNAMICGRAPH_SOT_DYNAMIC_TEST_STEPPER_HH
