#include "Math.hpp"
#include "Propeller.hpp"
#include "PistonEngine.hpp"
#include "PropEngine.hpp"
namespace yasim {

PropEngine::PropEngine(Propeller* prop, PistonEngine* eng, float moment)
{
    // Start off at 500rpm, because the start code doesn't exist yet
    _omega = 52.3f;
    _dir[0] = 1; _dir[1] = 0; _dir[2] = 0;

    _variable = false;

    _prop = prop;
    _eng = eng;
    _moment = moment;
    _fuel = true;
}

PropEngine::~PropEngine()
{
    delete _prop;
    delete _eng;
}

void PropEngine::setMagnetos(int pos)
{
    _magnetos = pos;
}

void PropEngine::setAdvance(float advance)
{
    _advance = Math::clamp(advance, 0, 1);
}

void PropEngine::setPropPitch(float proppitch)
{
    // update Propeller property
    _prop->setPropPitch(proppitch);
}

void PropEngine::setVariableProp(float min, float max)
{
    _variable = true;
    _minOmega = min;
    _maxOmega = max;
}

bool PropEngine::isRunning()
{
    return _eng->isRunning();
}

bool PropEngine::isCranking()
{
    return _eng->isCranking();
}

float PropEngine::getOmega()
{
    return _omega;
}

void PropEngine::getThrust(float* out)
{
    int i;
    for(i=0; i<3; i++) out[i] = _thrust[i];    
}

void PropEngine::getTorque(float* out)
{
    int i;
    for(i=0; i<3; i++) out[i] = _torque[i];
}

void PropEngine::getGyro(float* out)
{
    int i;
    for(i=0; i<3; i++) out[i] = _gyro[i];
}

float PropEngine::getFuelFlow()
{
    return _fuelFlow;
}

void PropEngine::stabilize()
{
    float speed = -Math::dot3(_wind, _dir);
    _eng->setThrottle(_throttle);
    _eng->setMixture(_mixture);

    _eng->setMagnetos(3);
    _eng->setRunning(true);

    if(_variable) {
	_omega = _minOmega + _advance * (_maxOmega - _minOmega);
	_prop->modPitch(1e6); // Start at maximum pitch and move down
    } else {
	_omega = 52;
    }

    bool goingUp = false;
    float step = 10;
    while(true) {
	float ptau, dummy;
	_prop->calc(_rho, speed, _omega, &dummy, &ptau);
	_eng->calc(_pressure, _temp, _omega);
        float etau = _eng->getTorque();
	float tdiff = etau - ptau;

	if(Math::abs(tdiff/_moment) < 0.1)
	    break;

	if(tdiff > 0) {
	    if(!goingUp) step *= 0.5f;
	    goingUp = true;
 	    if(!_variable)  _omega += step;
	    else            _prop->modPitch(1+(step*0.005f));
	} else {
	    if(goingUp) step *= 0.5f;
	    goingUp = false;
 	    if(!_variable)  _omega -= step;
	    else            _prop->modPitch(1-(step*0.005f));
	}
    }

    // ...and back off
    _eng->setRunning(false);
}

void PropEngine::init()
{
    _omega = 0.01f;
    _eng->setStarter(false);
    _eng->setMagnetos(0);
}

void PropEngine::integrate(float dt)
{
    float speed = -Math::dot3(_wind, _dir);

    float propTorque, engTorque, thrust;

    _eng->setThrottle(_throttle);
    _eng->setStarter(_starter);
    _eng->setMagnetos(_magnetos);
    _eng->setMixture(_mixture);
    _eng->setFuelState(_fuel);
    
    _prop->calc(_rho, speed, _omega, &thrust, &propTorque);
    _eng->calc(_pressure, _temp, _omega);
    engTorque = _eng->getTorque();
    _fuelFlow = _eng->getFuelFlow();

    // Turn the thrust into a vector and save it
    Math::mul3(thrust, _dir, _thrust);

    // Euler-integrate the RPM.  This doesn't need the full-on
    // Runge-Kutta stuff.
    float rotacc = (engTorque-propTorque)/Math::abs(_moment);
    _omega += dt * rotacc;
    if (_omega < 0)
        _omega = 0 - _omega;    // don't allow negative RPM
                                // FIXME: introduce proper windmilling

    // Store the total angular momentum into _gyro
    Math::mul3(_omega*_moment, _dir, _gyro);

    // Accumulate the engine torque, it acts on the body as a whole.
    // (Note: engine torque, not propeller torque.  They can be
    // different, but the difference goes to accelerating the
    // rotation.  It is the engine torque that is felt at the shaft
    // and works on the body.)
    float tau = _moment < 0 ? engTorque : -engTorque;
    Math::mul3(tau, _dir, _torque);

    // Iterate the propeller governor, if we have one.  Since engine
    // torque is basically constant with RPM, we want to make the
    // propeller torque at the target RPM equal to the engine by
    // varying the pitch.  Assume the the torque goes as the square of
    // the RPM (roughly correct) and compute a "target" torque for the
    // _current_ RPM.  Seek to that.  This is sort of a continuous
    // Newton-Raphson, basically.
    if(_variable) {
	float targetOmega = _minOmega + _advance*(_maxOmega-_minOmega);
	float ratio2 = (_omega*_omega)/(targetOmega*targetOmega);
	float targetTorque = engTorque * ratio2;

	float mod = propTorque < targetTorque ? 1.04f : (1.0f/1.04f);

	// Convert to an acceleration here, so that big propellers
	// don't seek faster than small ones.
	float diff = Math::abs((propTorque - targetTorque) / _moment);
	if(diff < 10) mod = 1 + (mod-1)*(0.1f*diff);

	_prop->modPitch(mod);
    }
}

}; // namespace yasim
