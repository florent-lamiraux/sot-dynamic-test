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

#include <math.h>
#include "stepper.hh"
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/factory.h>

namespace dynamicgraph {
  namespace sot {
    namespace dynamic {
      static const double gravity = 9.81;
      namespace command {
	namespace stepper {
	  using dynamicgraph::command::Value;
	  class Start : public dynamicgraph::command::Command
	  {
	  public:
	    Start(Stepper& entity, const std::string& docstring) :
	      Command(entity, std::vector<Value::Type>(), docstring)
	    {
	    }
	    virtual Value doExecute()
	    {
	      Stepper& st = static_cast<Stepper&>(owner());
	      st.start();
	      return Value();
	    }
	  }; // Class Start
	} // namespace stepper
      } // namespace command
      
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(Stepper, "Stepper");
      Stepper::Stepper(const std::string& inName) :
	Entity(inName),
	comGainSOUT("Stepper("+inName+")::output(double)::comGain"),
	comReferenceSOUT("Stepper("+inName+")::output(Vector)::comReference"),
	comVelocitySOUT("Stepper("+inName+")::output(Vector)::comdot"),
	zmpReferenceSOUT("Stepper("+inName+")::output(Vector)::zmpReference"),
	leftAnkleReferenceSOUT("Stepper("+inName+
			      ")::output(Vector)::leftAnkleReference"),
	rightAnkleReferenceSOUT("Stepper("+inName+
			       ")::output(Vector)::rightAnkleReference"),
	leftAnklePosition_(MatrixHomogeneous()),
	rightAnklePosition_(MatrixHomogeneous()),
	centerOfMass_((unsigned int)3),
	halfFootWidth_(0),
	startTime_(0),
	stepping_(false),
	magnitude_(0),
	omega_(0),
	comPeriod_(0),
	maxComGain_(10.),
	stepHeight_(0),
	timePeriod_(.005)
      {
	comGainSOUT.dependencyType = TimeDependency<int>::ALWAYS_READY;
	comReferenceSOUT.dependencyType = TimeDependency<int>::ALWAYS_READY;
	comVelocitySOUT.dependencyType = TimeDependency<int>::ALWAYS_READY;
	zmpReferenceSOUT.dependencyType = TimeDependency<int>::ALWAYS_READY;
	leftAnkleReferenceSOUT.dependencyType =
	  TimeDependency<int>::ALWAYS_READY;
	rightAnkleReferenceSOUT.dependencyType =
	  TimeDependency<int>::ALWAYS_READY;
	signalRegistration(comGainSOUT);
	signalRegistration(comReferenceSOUT);
	signalRegistration(comVelocitySOUT);
	signalRegistration(zmpReferenceSOUT);
	signalRegistration(leftAnkleReferenceSOUT);
	signalRegistration(rightAnkleReferenceSOUT);

	comGainSOUT.setFunction(boost::bind(&Stepper::computeComGain,
					    this, _1, _2));
	comReferenceSOUT.setFunction(boost::bind(&Stepper::computeComRef,
						 this, _1, _2));
	comVelocitySOUT.setFunction(boost::bind(&Stepper::computeComDot,
						this, _1, _2));
	zmpReferenceSOUT.setFunction(boost::bind(&Stepper::computeZmpRef,
						 this, _1, _2));
	leftAnkleReferenceSOUT.
	  setFunction(boost::bind(&Stepper::computeLeftAnkle, this, _1, _2));
	rightAnkleReferenceSOUT.
	  setFunction(boost::bind(&Stepper::computeRightAnkle, this, _1, _2));

	leftAnkleReferenceSOUT.addDependency(zmpReferenceSOUT);
	rightAnkleReferenceSOUT.addDependency(zmpReferenceSOUT);

	std::string docstring;
	docstring =
	  "\n"
	  "    Set position of left ankle before stepping\n"
	  "\n"
	  "      input:\n"
	  "        a vector of dimension 3\n"
	  "\n";
	addCommand("setLeftAnklePosition",
		   new dynamicgraph::command::Setter< Stepper, Matrix >
		   (*this, &Stepper::setLeftAnklePosition, docstring));
	docstring =
	  "\n"
	  "    Set position of right ankle before stepping\n"
	  "\n"
	  "      input:\n"
	  "        a vector of dimension 3\n"
	  "\n";
	addCommand("setRightAnklePosition",
		   new dynamicgraph::command::Setter<Stepper, Matrix>
		   (*this, &Stepper::setRightAnklePosition, docstring));

	docstring =
	  "\n"
	  "    Set position of left foot center before stepping\n"
	  "\n"
	  "      input:\n"
	  "        a vector of dimension 2\n"
	  "\n";
	addCommand("setLeftFootCenter",
		   new dynamicgraph::command::Setter<Stepper, Vector>
		   (*this, &Stepper::setLeftFootCenter, docstring));

	docstring =
	  "\n"
	  "    Set position of right foot center before stepping\n"
	  "\n"
	  "      input:\n"
	  "        a vector of dimension 2\n"
	  "\n";
	addCommand("setRightFootCenter",
		   new dynamicgraph::command::Setter<Stepper, Vector>
		   (*this, &Stepper::setRightFootCenter, docstring));

	docstring =
	  "\n"
	  "    Set position of center of mass before stepping\n"
	  "\n"
	  "      input:\n"
	  "        a vector of dimension 3\n"
	  "\n";
	addCommand("setCenterOfMass",
		   new dynamicgraph::command::Setter<Stepper, Vector>
		   (*this, &Stepper::setCenterOfMass, docstring));

	docstring =
	  "\n"
	  "    Set width of the feet\n"
	  "\n"
	  "      input:\n"
	  "        a floating point number\n"
	  "\n";
	addCommand("setFootWidth",
		   new dynamicgraph::command::Setter<Stepper, double>
		   (*this, &Stepper::setFootWidth, docstring));

	docstring =
	  "\n"
	  "    Start the stepping motion\n"
	  "\n"
	  "      no input:\n"
	  "\n";
	addCommand("start",
		   new command::stepper::Start(*this, docstring));

	docstring =
	  "\n"
	  "    Set step height\n"
	  "\n"
	  "      input:\n"
	  "        a floating point number\n"
	  "\n";
	addCommand("setStepHeight",
		   new dynamicgraph::command::Setter<Stepper, double>
		   (*this, &Stepper::setStepHeight, docstring));

	docstring =
	  "\n"
	  "    Get step height\n"
	  "\n"
	  "      return:\n"
	  "        a floating point number\n"
	  "\n";
	addCommand("getStepHeight",
		   new dynamicgraph::command::Getter<Stepper, double>
		   (*this, &Stepper::getStepHeight, docstring));

	docstring =
	  "\n"
	  "    Set maximal gain for center of mass task\n"
	  "\n"
	  "      input:\n"
	  "        a floating point number\n"
	  "\n";
	addCommand("setMaxComGain",
		   new dynamicgraph::command::Setter<Stepper, double>
		   (*this, &Stepper::setMaxComGain, docstring));

	docstring =
	  "\n"
	  "    Get maximal gain for center of mass task\n"
	  "\n"
	  "      return:\n"
	  "        a floating point number\n"
	  "\n";
	addCommand("getMaxComGain",
		   new dynamicgraph::command::Getter<Stepper, double>
		   (*this, &Stepper::getMaxComGain, docstring));
	docstring =
	  "\n"
	  "    Set sampling time period task\n"
	  "\n"
	  "      input:\n"
	  "        a floating point number\n"
	  "\n";
	addCommand("setTimePeriod",
		   new dynamicgraph::command::Setter<Stepper, double>
		   (*this, &Stepper::setTimePeriod, docstring));

	docstring =
	  "\n"
	  "    Get sampling time period task\n"
	  "\n"
	  "      return:\n"
	  "        a floating point number\n"
	  "\n";
	addCommand("getTimePeriod",
		   new dynamicgraph::command::Getter<Stepper, double>
		   (*this, &Stepper::getTimePeriod, docstring));
      }
      static MatrixHomogeneous toMatrixHomogeneous(const Matrix& inMatrix)
      {
	// Check that matrix is of dimension 4 by 4
	if ((inMatrix.nbRows() != 4) ||
	    (inMatrix.nbCols() != 4)) {
	  throw ExceptionAbstract(0,
				  "expected a matrix 4 by 4.");
	}
	MatrixHomogeneous mh;
	for (unsigned r = 0; r<3; r++) {
	  for (unsigned c = 0; c<4; c++) {
	    mh(r, c) = inMatrix(r, c);
	  }
	}
	return mh;
      }
      
      double Stepper::computeMagnitude(double t)
      {
	double magnitude = 0.;
	// Increase magnitude from 0 to maximal value into two periods of
	//oscillation
	if (t < 2.*comPeriod_) {
	  magnitude = t/(2.*comPeriod_);
	} else if (t <= 10.*comPeriod_) {
	  magnitude = 1.;
	} else if (t <= 12.*comPeriod_) {
	  magnitude = (12.*comPeriod_ - t)/(2.*comPeriod_);
	}
	return magnitude;
      }

      double Stepper::computeFootHeight(const Vector& inFootCenter,
					const Vector& inZmp)
      {
	double footHeight;
	double d = sqrt((inFootCenter(0)-inZmp(0))*(inFootCenter(0)-inZmp(0))+
			(inFootCenter(1)-inZmp(1))*(inFootCenter(1)-inZmp(1)));
	if (d > halfFootWidth_) {
	  footHeight = 0.;
	} else {
	  footHeight = stepHeight_*((halfFootWidth_-d)/halfFootWidth_)*
	    ((halfFootWidth_-d)/halfFootWidth_)*
	    ((halfFootWidth_-d)/halfFootWidth_);
	}
	return footHeight;
      }

      void Stepper::setLeftAnklePosition(const Matrix& inLeftAnklePosition)
      {
	leftAnklePosition_ = toMatrixHomogeneous(inLeftAnklePosition);
	leftAnkleReferenceSOUT.recompute(0);
      }

      void Stepper::setRightAnklePosition(const Matrix& inRightAnklePosition)
      {
	rightAnklePosition_ = toMatrixHomogeneous(inRightAnklePosition);
	rightAnkleReferenceSOUT.recompute(0);
      }

      void Stepper::setLeftFootCenter(const Vector& inFootPosition)
      {
	leftFootCenter_ = inFootPosition;
      }
      void Stepper::setRightFootCenter(const Vector& inFootPosition)
      {
	rightFootCenter_ = inFootPosition;
      }

      void Stepper::setCenterOfMass(const Vector& inCom)
      {
	// Check that vector is of dimension 3
	if (inCom.size() != 3)
	  throw ExceptionAbstract(0,
				  "expected vector of dimension 3");
	centerOfMass_ = inCom;
      }

      void Stepper::start()
      {
	if (!stepping_) {
	  stepping_ = true;
	  magnitude_ = 0.;
	  startTime_ = comReferenceSOUT.getTime();
	  Vector lf = leftFootCenter_;
	  Vector rf = rightFootCenter_;
	  Vector com;
	  com = (lf + rf)*.5;
	  centerOfMass_(0) = com(0);
	  centerOfMass_(1) = com(1);
	  omega_ = sqrt(gravity/centerOfMass_(2));
	  comPeriod_ = 2*M_PI/omega_;
	}
      }

      void Stepper::setFootWidth(const double& inWidth)
      {
	halfFootWidth_ = .5*inWidth;
	std::cout << "Half foot width = " << halfFootWidth_ << std::endl;
      }

      double& Stepper::computeComGain(double& comGain, const int& inTime)
      {
	if (stepping_) {
	  double t = timePeriod_*(inTime - startTime_);
	  if (t > 12*comPeriod_)
	    stepping_ = false;

	  // Increase gain from 1 to 100 into one period of oscillation
	  double a = t/comPeriod_;
	  if (a < 1.) {
	    comGain = (1. - a)*1. + a * maxComGain_;
	  } else {
	    comGain = maxComGain_;
	  }
	} else {
	  comGain = 1.0;
	}
	return comGain;
      }

      Vector& Stepper::computeComRef(Vector& comRef, const int& inTime)
      {
	if (stepping_) {
	  double t = timePeriod_*(inTime - startTime_);
	  double magnitude = computeMagnitude(t);
	  Vector lf = leftFootCenter_;
	  Vector rf = rightFootCenter_;
	  comRef = centerOfMass_ + (lf - rf)*(.25*magnitude*sin(omega_*t));
	} else {
	  comRef = centerOfMass_;
	}
	return comRef;
      }

      Vector& Stepper::computeComDot(Vector& comDot, const int& inTime)
      {
	if (stepping_) {
	  double t = timePeriod_*(inTime - startTime_);
	  double magnitude = computeMagnitude(t);
	  Vector lf = leftFootCenter_;
	  Vector rf = rightFootCenter_;
	  comDot = omega_*(lf - rf)*(.25*magnitude*cos(omega_*t));
	} else {
	  comDot = 0 * centerOfMass_;
	}
	return comDot;
      }

      Vector& Stepper::computeZmpRef(Vector& zmpRef, const int& inTime)
      {
	if (stepping_) {
	  double t = timePeriod_*(inTime - startTime_);
	  double magnitude = computeMagnitude(t);
	  Vector lf = leftFootCenter_;
	  Vector rf = rightFootCenter_;
	  
	  zmpRef = centerOfMass_ + (lf - rf)*(.5*magnitude*sin(omega_*t));
	} else {
	  zmpRef = centerOfMass_;
	}
	// Express ZMP in waist reference frame.
	zmpRef(2) = 0.;
	return zmpRef;
      }

      MatrixHomogeneous& Stepper::
      computeLeftAnkle(MatrixHomogeneous& leftAnklePosition, const int& inTime)
      {
	leftAnklePosition = leftAnklePosition_;
	if (stepping_) {
	  double t = timePeriod_*(inTime - startTime_);
	  if (t > 3*comPeriod_ && t < 9*comPeriod_) {
	    Vector rf = rightFootCenter_;
	    double footHeight = computeFootHeight(rf, zmpReferenceSOUT(inTime));
	    leftAnklePosition(2,3) = leftAnklePosition_(2,3) + footHeight;
	  }
	}
	return leftAnklePosition;
      }

      MatrixHomogeneous& Stepper::
      computeRightAnkle(MatrixHomogeneous& rightAnklePosition,
			const int& inTime)
      {
	rightAnklePosition = rightAnklePosition_;
	if (stepping_) {
	  double t = timePeriod_*(inTime - startTime_);
	  if (t > 3*comPeriod_ && t < 9*comPeriod_) {
	    Vector lf = leftFootCenter_;
	    double footHeight = computeFootHeight(lf, zmpReferenceSOUT(inTime));
	    rightAnklePosition(2,3) = rightAnklePosition_(2,3) + footHeight;
	  }
	}
	return rightAnklePosition;
      }
    } // namespace dynamic
  } // namespace sot
} // namespace dynamicgraph
