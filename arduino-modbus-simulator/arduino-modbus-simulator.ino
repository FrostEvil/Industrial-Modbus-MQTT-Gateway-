#include <ModbusRTUSlave.h>

const int rsePin = 7;
ModbusRTUSlave modbus(Serial, rsePin);

uint16_t holdingRegisters[3];
const uint16_t numHoldingRegisters = 3;
const uint8_t slaveId = 0x01;

unsigned long lastUpdate = 0;
const unsigned long UPDATE_INTERVAL_MS = 1000;

float voltage = 230.0f;
float current = 16.0f;
float temperature = 25.0f;

const float VOLTAGE_MAX_STEP = 0.5f;
const float CURRENT_MAX_STEP = 0.2f;      // assumption, not yet confirmed
const float TEMPERATURE_MAX_STEP = 0.3f;  // assumption, not yet confirmed

const float VOLTAGE_MIN_BOUND = 225.0f, VOLTAGE_MAX_BOUND = 235.0f;
const float CURRENT_MIN_BOUND = 14.0f,  CURRENT_MAX_BOUND = 18.0f;

// Temperature deliberately gets a WIDE hard limit (0-50C) -- unlike V/I, it's
// allowed to actually wander into the STM32's alarm range once in a while,
// simulating real seasonal swings or a genuine overheating event, not just
// the scripted incidents below.
const float TEMPERATURE_MIN_BOUND = 0.0f, TEMPERATURE_MAX_BOUND = 50.0f;
const float TEMPERATURE_BASELINE = 25.0f;

// How strongly temperature gets pulled back toward TEMPERATURE_BASELINE on
// every update, proportional to how far away it currently is. This turned
// out to matter far more than expected: WITHOUT any pull-back, a plain
// random walk bounded only by [0,50] spent 73% of a simulated 3-day run
// outside the STM32's normal 20-35C band -- the complete opposite of
// "occasionally". A small pull-back like this keeps it settled near 25C
// most of the time, while still allowing real excursions during an unlucky
// (or "seasonal") streak of steps in the same direction. Bigger number =
// snaps back harder = rarer excursions.
const float TEMPERATURE_PULL_STRENGTH = 0.002f;

enum IncidentType { INCIDENT_NONE, INCIDENT_VOLTAGE_LOSS, INCIDENT_OVERCURRENT_TRIP };

IncidentType activeIncident = INCIDENT_NONE;
uint8_t incidentSamplesLeft = 0;
bool incidentFirstSample = false;

// ~0.65% chance per second while idle -> averages out to roughly once every
// 2.5 minutes (1 / 0.0065 seconds), matching "frequent enough to actually
// see during a testing session" rather than the original 1-2%/s (which
// worked out to about once a minute -- see the earlier calculation).
const float INCIDENT_CHANCE_PER_SAMPLE = 0.0065f;

float randomStep(float maxStep) {
    long magnitudeScaled = random(0, (long)(maxStep * 1000.0f) + 1);
    float magnitude = magnitudeScaled / 1000.0f;
    return (random(0, 2) == 0) ? -magnitude : magnitude;
}

float driftValue(float value, float maxStep, float minBound, float maxBound) {
    value += randomStep(maxStep);
    if (value < minBound) value = minBound;
    if (value > maxBound) value = maxBound;
    return value;
}

// Same idea as driftValue(), plus a gentle nudge back toward baseline every
// step. That nudge is what keeps this a "mostly near 25C, sometimes further"
// process instead of a directionless wander across the whole 0-50C range.
float driftValueWithPullback(float value, float maxStep, float baseline, float pullStrength, float minBound, float maxBound) {
    float towardBaseline = (baseline - value) * pullStrength;
    value += towardBaseline + randomStep(maxStep);
    if (value < minBound) value = minBound;
    if (value > maxBound) value = maxBound;
    return value;
}

void maybeStartIncident() {
    float roll = random(0, 1000001) / 1000000.0f;
    if (roll >= INCIDENT_CHANCE_PER_SAMPLE) return;

    activeIncident = random(0, 2) == 0 ? INCIDENT_VOLTAGE_LOSS : INCIDENT_OVERCURRENT_TRIP;
    incidentSamplesLeft = random(5, 13);
    incidentFirstSample = true;
}

void applyIncidentOverride(float &reportedVoltage, float &reportedCurrent) {
    if (activeIncident == INCIDENT_VOLTAGE_LOSS) {
        reportedVoltage = 0.0f;
        reportedCurrent = 0.0f;  // no supply voltage -> nothing downstream can draw current either
    } else if (activeIncident == INCIDENT_OVERCURRENT_TRIP) {
        reportedCurrent = incidentFirstSample ? 40.0f : 0.0f;  // one overload sample above 36.25A, then breaker-open silence
    }
    incidentFirstSample = false;

    incidentSamplesLeft--;
    if (incidentSamplesLeft == 0) {
        activeIncident = INCIDENT_NONE;
    }
}

void setup()
{
    modbus.configureHoldingRegisters(holdingRegisters, numHoldingRegisters);
    Serial.begin(9600);
    modbus.begin(slaveId, 9600, SERIAL_8N1);

    randomSeed(analogRead(A0));
}

void loop()
{
    if (millis() - lastUpdate >= UPDATE_INTERVAL_MS)
    {
        voltage     = driftValue(voltage, VOLTAGE_MAX_STEP, VOLTAGE_MIN_BOUND, VOLTAGE_MAX_BOUND);
        current     = driftValue(current, CURRENT_MAX_STEP, CURRENT_MIN_BOUND, CURRENT_MAX_BOUND);
        temperature = driftValueWithPullback(temperature, TEMPERATURE_MAX_STEP, TEMPERATURE_BASELINE,
                                              TEMPERATURE_PULL_STRENGTH, TEMPERATURE_MIN_BOUND, TEMPERATURE_MAX_BOUND);

        float reportedVoltage = voltage;
        float reportedCurrent = current;

        if (activeIncident == INCIDENT_NONE) {
            maybeStartIncident();
        }
        if (activeIncident != INCIDENT_NONE) {
            applyIncidentOverride(reportedVoltage, reportedCurrent);
        }

        holdingRegisters[0] = (uint16_t)(reportedVoltage * 10.0f + 0.5f);
        holdingRegisters[1] = (uint16_t)(reportedCurrent * 10.0f + 0.5f);
        holdingRegisters[2] = (uint16_t)(temperature * 10.0f + 0.5f);

        lastUpdate = millis();
    }

    modbus.poll();
}