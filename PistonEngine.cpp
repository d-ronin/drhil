#include "Atmosphere.hpp"
#include "Math.hpp"
#include "PistonEngine.hpp"
namespace yasim {

const static float HP2W = 745.7f;
const static float CIN2CM = 1.6387064e-5f;
const static float RPM2RADPS = 0.1047198f;

PistonEngine::PistonEngine(float power, float speed)
{
    _boost = 1;
    _running = false;
    _cranking = false;
    _fuel = true;

    // Presume a BSFC (in lb/hour per HP) of 0.45.  In SI that becomes
    // (2.2 lb/kg, 745.7 W/hp, 3600 sec/hour) 7.62e-08 kg/Ws.
    _f0 = power * 7.62e-08f;

    _power0 = power;
    _omega0 = speed;

    // We must be at sea level under standard conditions
    _rho0 = Atmosphere::getStdDensity(0);

    // Further presume that takeoff is (duh) full throttle and
    // peak-power, that means that by our efficiency function, we are
    // at 11/8 of "ideal" fuel flow.
    float realFlow = _f0 * (11.0f/8.0f);
    _mixCoeff = realFlow * 1.1f / _omega0;

    _turbo = 1;
    _maxMP = 1e6; // No waste gate on non-turbo engines.

    // Guess at reasonable values for these guys.  Displacements run
    // at about 2 cubic inches per horsepower or so, at least for
    // non-turbocharged engines.
    _compression = 8;
    _displacement = power * (2*CIN2CM/HP2W);
}

void PistonEngine::setTurboParams(float turbo, float maxMP)
{
    _turbo = turbo;
    _maxMP = maxMP;

    // This changes the "sea level" manifold air density
    float P0 = Atmosphere::getStdPressure(0);
    float P = P0 * (1 + _boost * (_turbo - 1));
    if(P > _maxMP) P = _maxMP;
    float T = Atmosphere::getStdTemperature(0) * Math::pow(P/P0, 2./7.);
    _rho0 = P / (287.1f * T);
}

void PistonEngine::setDisplacement(float d)
{
    _displacement = d;
}

void PistonEngine::setCompression(float c)
{
    _compression = c;
}

float PistonEngine::getMaxPower()
{
    return _power0;
}

void PistonEngine::setThrottle(float t)
{
    _throttle = t;
}

void PistonEngine::setRunning(bool r)
{
    _running = r;
}

void PistonEngine::setStarter(bool s)
{
    _cranking = s;
}

void PistonEngine::setMagnetos(int m)
{
    _magnetos = m;
}

void PistonEngine::setMixture(float m)
{
    _mixture = m;
}

void PistonEngine::setBoost(float boost)
{
    _boost = boost;
}

bool PistonEngine::isRunning()
{
    return _running;
}

bool PistonEngine::isCranking()
{
    return _cranking;
}

float PistonEngine::getTorque()
{
    return _torque;
}

float PistonEngine::getFuelFlow()
{
    return _fuelFlow;
}

float PistonEngine::getMP()
{
    return _mp;
}

float PistonEngine::getEGT()
{
    return _egt;
}

void PistonEngine::calc(float pressure, float temp, float speed)
{
    if(_magnetos == 0 || speed < 60*RPM2RADPS)
	_running = false;
    else if(_fuel == false)
        _running = false;
    else
	_running = true;

    // Calculate manifold pressure as ambient pressure modified for
    // turbocharging and reduced by the throttle setting.  According
    // to Dave Luff, minimum throttle at sea level corresponds to 6"
    // manifold pressure.  Assume that this means that minimum MP is
    // always 20% of ambient pressure. (But that's too much idle
    // power, so use 10% instead!) But we need to produce _zero_
    // thrust at that setting, so hold onto the "output" value
    // separately.  Ick.
    _mp = pressure * (1 + _boost*(_turbo-1)); // turbocharger
    float mp = _mp * (0.1f + 0.9f * _throttle); // throttle
    _mp *= _throttle;
    if(mp > _maxMP) mp = _maxMP;              // wastegate

    // Air entering the manifold does so rapidly, and thus the
    // pressure change can be assumed to be adiabatic.  Calculate a
    // temperature change, and use that to get the density.
    float T = temp * Math::pow(mp/pressure, 2.0/7.0);
    float rho = mp / (287.1f * T);

    // The actual fuel flow is determined only by engine RPM and the
    // mixture setting.  Not all of this will burn with the same
    // efficiency.
    _fuelFlow = _mixture * speed * _mixCoeff;
    if(_fuel == false) _fuelFlow = 0;

    // How much fuel could be burned with ideal (i.e. uncorrected!)
    // combustion.
    float burnable = _f0 * (rho/_rho0) * (speed/_omega0);

    // Calculate the fuel that actually burns to produce work.  The
    // idea is that less than 5/8 of ideal, we get complete
    // combustion.  We use up all the oxygen at 1 3/8 of ideal (that
    // is, you need to waste fuel to use all your O2).  In between,
    // interpolate.  This vaguely matches a curve I copied out of a
    // book for a single engine.  Shrug.
    float burned;
    float r = _fuelFlow/burnable;
    if     (burnable == 0) burned = 0;
    else if(r < .625)      burned = _fuelFlow;
    else if(r > 1.375)     burned = burnable;
    else
        burned = _fuelFlow + (burnable-_fuelFlow)*(r-0.625f)*(4.0f/3.0f);

    // Correct for engine control state
    if(!_running)
	burned = 0;
    if(_magnetos < 3)
	burned *= 0.9f;

    // And finally the power is just the reference power scaled by the
    // amount of fuel burned, and torque is that divided by RPM.
    float power = _power0 * burned/_f0;
    _torque = power/speed;

    // Figure that the starter motor produces 15% of the engine's
    // cruise torque.  Assuming 60RPM starter speed vs. 1800RPM cruise
    // speed on a 160HP engine, that comes out to about 160*.15/30 ==
    // 0.8 HP starter motor.  Which sounds about right to me.  I think
    // I've finally got this tuned. :)
    if(_cranking && !_running)
	_torque += 0.15f * _power0/_omega0;

    // Also, add a negative torque of 8% of cruise, to represent
    // internal friction.  Propeller aerodynamic friction is too low
    // at low RPMs to provide a good deceleration.  Interpolate it
    // away as we approach cruise RPMs (full at 50%, zero at 100%),
    // though, to prevent interaction with the power computations.
    // Ugly.
    if(speed > 0 && speed < _omega0) {
        float interp = 2 - 2*speed/_omega0;
        interp = (interp > 1) ? 1 : interp;
	_torque -= 0.08f * (_power0/_omega0) * interp;
    }

    // Now EGT.  This one gets a little goofy.  We can calculate the
    // work done by an isentropically expanding exhaust gas as the
    // mass of the gas times the specific heat times the change in
    // temperature.  The mass is just the engine displacement times
    // the manifold density, plus the mass of the fuel, which we know.
    // The change in temperature can be calculated adiabatically as a
    // function of the exhaust gas temperature and the compression
    // ratio (which we know).  So just rearrange the equation to get
    // EGT as a function of engine power.  Cool.  I'm using a value of
    // 1300 J/(kg*K) for the exhaust gas specific heat.  I found this
    // on a web page somewhere; no idea if it's accurate.  Also,
    // remember that four stroke engines do one combustion cycle every
    // TWO revolutions, so the displacement per revolution is half of
    // what we'd expect.  And diddle the work done by the gas a bit to
    // account for non-thermodynamic losses like internal friction;
    // 10% should do it.

    float massFlow = _fuelFlow + (rho * 0.5f * _displacement * speed);
    float specHeat = 1300;
    float corr = 1.0f/(Math::pow(_compression, 0.4f) - 1.0f);
    _egt = corr * (power * 1.1f) / (massFlow * specHeat);
    if(_egt < temp) _egt = temp;
}

}; // namespace yasim
