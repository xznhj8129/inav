/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

// JSBSim as an in-process SITL simulator backend. Physics runs inside the
// stepped clock: whenever SITL's simulated time moves, JSBSim runs in its
// fixed dt up to it, driven by INAV's motor and servo outputs, and its state
// goes to INAV's simulated sensors through the shared layer (simSensors.h).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <initialization/FGTrim.h>
#include <input_output/FGLog.h>
#include <input_output/FGPropertyManager.h>
#include <input_output/FGXMLElement.h>
#include <input_output/FGXMLFileRead.h>

#include "target/SITL/sim/jsbsim.h"

using JSBSim::Element;

namespace {

constexpr double FT_M = 0.3048;
constexpr double KTS_MS = 0.514444;
constexpr double G_MSS = 9.80665;
constexpr double GROUND_STRIKE_M = -0.5;   // below this the airframe is in the ground, not on its gear
constexpr double RANGEFINDER_MIN_TILT = 0.5;  // cos of the tilt beyond which the rangefinder sees no ground

// Prints JSBSim's errors except the ones the vehicle file lists as harmless.
class ErrorsOnly : public JSBSim::FGLogger {
public:
    explicit ErrorsOnly(std::vector<std::string> known) : known(std::move(known)) {}

    void SetLevel(JSBSim::LogLevel level) override
    {
        log_level = level;
        keep = level >= JSBSim::LogLevel::ERROR;
        text.clear();
    }

    void Message(const std::string &message) override
    {
        if (keep) {
            text += message;
        }
    }

    void Flush(void) override
    {
        if (keep && !text.empty()) {
            bool isKnown = false;
            for (const std::string &k : known) {
                isKnown = isKnown || text.find(k) != std::string::npos;
            }
            if (!isKnown) {
                fprintf(stderr, "[SIM][JSBSIM] %s\n", text.c_str());
            }
        }
        text.clear();
    }

private:
    std::vector<std::string> known;
    std::string text;
    bool keep = false;
};

// One INAV output driving one JSBSim property
struct Output {
    bool isMotor;
    uint8_t index;
    bool invert;
    SGPropertyNode *property;
};

struct Vehicle {
    std::unique_ptr<JSBSim::FGFDMExec> fdm;
    std::vector<Output> outputs;
    uint32_t dtUs;
    double rangefinderMaxM;     // 0: no rangefinder
    bool started;
    uint32_t startUs;
    uint64_t steps;

    SGPropertyNode *nx, *ny, *nz;
    SGPropertyNode *p, *q, *r;
    SGPropertyNode *phi, *theta, *psi;
    SGPropertyNode *lat, *lon, *hsl, *hagl;
    SGPropertyNode *vn, *ve, *vd, *vc;
};

Vehicle vehicle;

SGPropertyNode *node(const char *name)
{
    return vehicle.fdm->GetPropertyManager()->GetNode(name);
}

[[noreturn]] void fail(const std::string &what)
{
    fprintf(stderr, "[SIM][JSBSIM] %s\n", what.c_str());
    exit(1);
}

SGPath resolve(const SGPath &dir, const std::string &path)
{
    const SGPath p(path);
    return p.isAbsolute() ? p : dir / path;
}

// --- sensor model -----------------------------------------------------------
// JSBSim's body frame is forward-right-down; the sample is forward-left-up.

simSensorSample_t sensorSample(void)
{
    const Vehicle &v = vehicle;
    simSensorSample_t s;

    const double phi = v.phi->getDoubleValue();
    const double theta = v.theta->getDoubleValue();
    const double psi = v.psi->getDoubleValue();

    // N* are specific force in g, with Nz positive up
    s.specificForceFlu[0] = v.nx->getDoubleValue() * G_MSS;
    s.specificForceFlu[1] = -v.ny->getDoubleValue() * G_MSS;
    s.specificForceFlu[2] = v.nz->getDoubleValue() * G_MSS;

    s.rateFlu[0] = v.p->getDoubleValue();
    s.rateFlu[1] = -v.q->getDoubleValue();
    s.rateFlu[2] = -v.r->getDoubleValue();

    // a flat field pointing north (no dip), north rotated into the body
    s.magFlu[0] = cos(theta) * cos(psi);
    s.magFlu[1] = -(sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi));
    s.magFlu[2] = -(cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi));

    s.roll = phi;
    s.pitch = theta;
    s.yaw = psi;

    const double altMslM = v.hsl->getDoubleValue() * FT_M;
    s.latDeg = v.lat->getDoubleValue();
    s.lonDeg = v.lon->getDoubleValue();
    s.altMslM = altMslM;
    s.velNed[0] = v.vn->getDoubleValue() * FT_M;
    s.velNed[1] = v.ve->getDoubleValue() * FT_M;
    s.velNed[2] = v.vd->getDoubleValue() * FT_M;

    s.pressurePa = 101325.0 * pow(1.0 - 2.25577e-5 * altMslM, 5.25588);   // ISA
    s.airspeedMs = v.vc->getDoubleValue() * KTS_MS;

    s.rangeM = -1.0f;
    const double tilt = cos(phi) * cos(theta);
    if (v.rangefinderMaxM > 0.0 && tilt >= RANGEFINDER_MIN_TILT) {
        const double d = v.hagl->getDoubleValue() * FT_M / tilt;
        if (d <= v.rangefinderMaxM) {
            s.rangeM = d;
        }
    }

    return s;
}

void setOutputs(void)
{
    for (const Output &o : vehicle.outputs) {
        const float value = o.isMotor ? simMotorOutput(o.index) : simServoOutput(o.index);
        o.property->setDoubleValue(o.invert ? -value : value);
    }
}

bool advanceTo(uint32_t simTimeUs)
{
    Vehicle &v = vehicle;
    if (!v.started) {
        v.started = true;
        v.startUs = simTimeUs;
        return false;
    }

    const uint64_t due = (uint32_t)(simTimeUs - v.startUs) / v.dtUs;
    if (due <= v.steps) {
        return false;
    }

    setOutputs();
    while (v.steps < due) {
        v.fdm->Run();
        v.steps++;
        const double aglM = v.hagl->getDoubleValue() * FT_M;
        if (std::isnan(aglM) || aglM < GROUND_STRIKE_M) {
            fprintf(stderr, "[SIM][JSBSIM] ground strike at sim %.3f s, agl %.2f m\n", v.steps * v.dtUs / 1e6, aglM);
            exit(1);
        }
    }

    const simSensorSample_t sample = sensorSample();
    simSensorsApply(&sample, simTimeUs);
    return true;
}

void getTruth(simTruth_t *truth)
{
    const Vehicle &v = vehicle;
    truth->simTimeUs = v.startUs + (uint32_t)(v.steps * v.dtUs);
    truth->latDeg = v.lat->getDoubleValue();
    truth->lonDeg = v.lon->getDoubleValue();
    truth->altMslM = v.hsl->getDoubleValue() * FT_M;
    truth->aglM = v.hagl->getDoubleValue() * FT_M;
    truth->roll = v.phi->getDoubleValue();
    truth->pitch = v.theta->getDoubleValue();
    truth->yaw = v.psi->getDoubleValue();
    truth->velNed[0] = v.vn->getDoubleValue() * FT_M;
    truth->velNed[1] = v.ve->getDoubleValue() * FT_M;
    truth->velNed[2] = v.vd->getDoubleValue() * FT_M;
    truth->airspeedMs = v.vc->getDoubleValue() * KTS_MS;
}

const simInProcessBackend_t backend = { advanceTo, getTruth };

// --- vehicle file -------------------------------------------------------------

void loadOutputs(Element *doc, const char *tag, bool isMotor, uint8_t count)
{
    for (Element *e = doc->FindElement(tag); e; e = doc->FindNextElement(tag)) {
        const int index = (int)e->GetAttributeValueAsNumber("index");
        if (index < 0 || index >= count) {
            fail(e->ReadFrom() + "<" + tag + "> index " + std::to_string(index) + " out of range 0.." + std::to_string(count - 1));
        }
        const std::string property = e->GetDataLine();
        vehicle.outputs.push_back({isMotor, (uint8_t)index, e->GetAttributeValue("invert") == "true",
                                   vehicle.fdm->GetPropertyManager()->GetNode(property, true)});
    }
}

}  // namespace

const simInProcessBackend_t *simJsbsimInit(const char *vehicleFile, bool useImu)
{
    Vehicle &v = vehicle;
    const SGPath file = SGPath::fromLocal8Bit(vehicleFile);
    const SGPath dir = file.dirPath();

    JSBSim::FGXMLFileRead reader;
    Element *doc = reader.LoadXMLDocument(file);
    if (!doc) {
        fail("cannot read vehicle file " + file.utf8Str());
    }

    std::vector<std::string> knownErrors;
    for (Element *e = doc->FindElement("known_error"); e; e = doc->FindNextElement("known_error")) {
        knownErrors.push_back(e->GetDataLine());
    }

    Element *model = doc->FindElement("model");
    if (!model) {
        fail(file.utf8Str() + ": no <model>");
    }

    // JSBSim's constructor takes its console chatter level from JSBSIM_DEBUG
    // (default 1: banner and load reports); quiet unless the user set it
    setenv("JSBSIM_DEBUG", "0", 0);
    v.fdm = std::make_unique<JSBSim::FGFDMExec>();
    v.fdm->SetLogger(std::make_shared<ErrorsOnly>(knownErrors));
    v.fdm->SetRootDir(resolve(dir, model->GetAttributeValue("root")));
    v.fdm->SetAircraftPath(SGPath("aircraft"));
    v.fdm->SetEnginePath(SGPath("engine"));
    v.fdm->SetSystemsPath(SGPath("systems"));
    if (!v.fdm->LoadModel(model->GetDataLine())) {
        fail("JSBSim could not load model " + model->GetDataLine() + " from " + model->GetAttributeValue("root"));
    }

    const double dt = model->GetAttributeValueAsNumber("dt");
    v.fdm->Setdt(dt);
    v.dtUs = (uint32_t)lround(dt * 1e6);

    // JSBSim properties set before the start condition is applied, e.g. wind or mixture
    for (Element *e = doc->FindElement("property"); e; e = doc->FindNextElement("property")) {
        v.fdm->SetPropertyValue(e->GetAttributeValue("name"), e->GetDataAsNumber());
    }

    // The start condition is a standard JSBSim initialization file
    Element *init = doc->FindElement("initialize");
    if (!init) {
        fail(file.utf8Str() + ": no <initialize>");
    }
    auto ic = v.fdm->GetIC();
    if (!ic->Load(resolve(dir, init->GetDataLine()), false)) {
        fail("JSBSim could not load initialization file " + init->GetDataLine());
    }
    v.fdm->RunIC();
    // as JSBSim's own executable does: a trim the initialization file asks for
    if (ic->TrimRequested() != JSBSim::tNone) {
        v.fdm->DoTrim(ic->TrimRequested());
    }

    loadOutputs(doc, "motor", true, simMotorCount());
    loadOutputs(doc, "servo", false, simServoCount());

    Element *rangefinder = doc->FindElement("rangefinder");
    v.rangefinderMaxM = rangefinder ? rangefinder->GetAttributeValueAsNumber("max") : 0.0;

    Element *gps = doc->FindElement("gps");
    if (!gps) {
        fail(file.utf8Str() + ": no <gps rate=...>");
    }
    simSensorsInit(useImu, (uint16_t)gps->GetAttributeValueAsNumber("rate"));

    v.nx = node("accelerations/Nx");
    v.ny = node("accelerations/Ny");
    v.nz = node("accelerations/Nz");
    v.p = node("velocities/p-rad_sec");
    v.q = node("velocities/q-rad_sec");
    v.r = node("velocities/r-rad_sec");
    v.phi = node("attitude/phi-rad");
    v.theta = node("attitude/theta-rad");
    v.psi = node("attitude/psi-rad");
    v.lat = node("position/lat-geod-deg");
    v.lon = node("position/long-gc-deg");
    v.hsl = node("position/h-sl-ft");
    v.hagl = node("position/h-agl-ft");
    v.vn = node("velocities/v-north-fps");
    v.ve = node("velocities/v-east-fps");
    v.vd = node("velocities/v-down-fps");
    v.vc = node("velocities/vc-kts");

    const simSensorSample_t sample = sensorSample();
    simSensorsApplyBoot(&sample);

    fprintf(stderr, "[SIM][JSBSIM] %s: model %s, dt %u us, %zu outputs, agl %.1f m\n",
            file.utf8Str().c_str(), model->GetDataLine().c_str(), v.dtUs, v.outputs.size(), v.hagl->getDoubleValue() * FT_M);
    return &backend;
}
