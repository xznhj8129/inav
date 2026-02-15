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

#include "target/SITL/sim/gazebo_native.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <chrono>

#if defined(USE_GAZEBO_NATIVE)
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/fluid_pressure.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>

namespace {

struct gazeboContext_t {
    std::mutex lock;
    gazeboNativeSample_t sample;
    std::atomic<bool> hasSample;
    std::atomic<uint32_t> imuUpdates;
    std::atomic<uint32_t> baroUpdates;
    std::atomic<uint32_t> magUpdates;
    std::atomic<uint32_t> navsatUpdates;
    std::atomic<uint32_t> odomUpdates;
    std::atomic<uint32_t> poseUpdates;
    std::atomic<uint32_t> imuDtUsLast;
    std::atomic<uint32_t> imuDtUsMin;
    std::atomic<uint32_t> imuDtUsMax;
    std::atomic<uint64_t> lastImuStampUs;
    std::string modelName;
    gz::transport::Node node;
    gz::transport::Node::Publisher motorPublisherModelTopicActuators;
    gz::transport::Node::Publisher motorPublisherCompatTopicActuators;
};

static gazeboContext_t ctx;
static std::atomic<bool> initialized(false);

static uint64_t monotonicMicros()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(now).count();
}

static uint64_t simStampMicros(const gz::msgs::Header &_header)
{
    if (!_header.has_stamp()) {
        return 0;
    }

    const int64_t sec = _header.stamp().sec();
    const int64_t nsec = _header.stamp().nsec();
    if (sec < 0 || nsec < 0) {
        return 0;
    }

    return (uint64_t)sec * 1000000ULL + (uint64_t)nsec / 1000ULL;
}

static void updateImu(const gz::msgs::IMU &_msg)
{
    const uint64_t stampUs = monotonicMicros();
    const uint64_t simStampUs = simStampMicros(_msg.header());
    std::lock_guard<std::mutex> guard(ctx.lock);
    ctx.sample.hasImu = true;
    ctx.sample.accelMss[0] = (float)_msg.linear_acceleration().x();
    ctx.sample.accelMss[1] = (float)_msg.linear_acceleration().y();
    ctx.sample.accelMss[2] = (float)_msg.linear_acceleration().z();
    ctx.sample.gyroRadS[0] = (float)_msg.angular_velocity().x();
    ctx.sample.gyroRadS[1] = (float)_msg.angular_velocity().y();
    ctx.sample.gyroRadS[2] = (float)_msg.angular_velocity().z();
    ctx.sample.quatWxyz[0] = (float)_msg.orientation().w();
    ctx.sample.quatWxyz[1] = (float)_msg.orientation().x();
    ctx.sample.quatWxyz[2] = (float)_msg.orientation().y();
    ctx.sample.quatWxyz[3] = (float)_msg.orientation().z();
    const uint32_t imuSeq = ctx.sample.imuSeq + 1;
    ctx.sample.imuSeq = imuSeq;
    ctx.sample.imuSimStampUs = simStampUs;
    ctx.sample.imuStampUs = stampUs;
    ctx.hasSample.store(true);
    ctx.imuUpdates.fetch_add(1);
    const uint64_t prevStampUs = ctx.lastImuStampUs.exchange(stampUs);
    if (prevStampUs > 0 && stampUs > prevStampUs) {
        const uint32_t dtUs = (uint32_t)(stampUs - prevStampUs);
        ctx.imuDtUsLast.store(dtUs);
        uint32_t minDtUs = ctx.imuDtUsMin.load();
        if (minDtUs == 0 || dtUs < minDtUs) {
            ctx.imuDtUsMin.store(dtUs);
        }
        const uint32_t maxDtUs = ctx.imuDtUsMax.load();
        if (dtUs > maxDtUs) {
            ctx.imuDtUsMax.store(dtUs);
        }
    }
}

static void updatePressure(const gz::msgs::FluidPressure &_msg)
{
    const uint64_t stampUs = monotonicMicros();
    std::lock_guard<std::mutex> guard(ctx.lock);
    ctx.sample.hasBaro = true;
    ctx.sample.pressurePa = (float)_msg.pressure();
    ctx.sample.baroStampUs = stampUs;
    ctx.hasSample.store(true);
    ctx.baroUpdates.fetch_add(1);
}

static void updateMag(const gz::msgs::Magnetometer &_msg)
{
    const uint64_t stampUs = monotonicMicros();
    std::lock_guard<std::mutex> guard(ctx.lock);
    ctx.sample.hasMag = true;
    ctx.sample.magTesla[0] = (float)_msg.field_tesla().x();
    ctx.sample.magTesla[1] = (float)_msg.field_tesla().y();
    ctx.sample.magTesla[2] = (float)_msg.field_tesla().z();
    ctx.sample.magStampUs = stampUs;
    ctx.hasSample.store(true);
    ctx.magUpdates.fetch_add(1);
}

static void updateNavSat(const gz::msgs::NavSat &_msg)
{
    const uint64_t stampUs = monotonicMicros();
    std::lock_guard<std::mutex> guard(ctx.lock);
    ctx.sample.hasNavSat = true;
    ctx.sample.latitudeDeg = _msg.latitude_deg();
    ctx.sample.longitudeDeg = _msg.longitude_deg();
    ctx.sample.altitudeM = _msg.altitude();
    if (std::isfinite(_msg.velocity_east()) && std::isfinite(_msg.velocity_north()) && std::isfinite(_msg.velocity_up())) {
        ctx.sample.hasVelocity = true;
        ctx.sample.velocityEnuMss[0] = (float)_msg.velocity_east();
        ctx.sample.velocityEnuMss[1] = (float)_msg.velocity_north();
        ctx.sample.velocityEnuMss[2] = (float)_msg.velocity_up();
    }
    ctx.sample.navsatStampUs = stampUs;
    ctx.hasSample.store(true);
    ctx.navsatUpdates.fetch_add(1);
}

static void updateOdometry(const gz::msgs::Odometry &_msg)
{
    const uint64_t stampUs = monotonicMicros();
    std::lock_guard<std::mutex> guard(ctx.lock);
    ctx.sample.hasVelocity = true;
    ctx.sample.velocityEnuMss[0] = (float)_msg.twist().linear().x();
    ctx.sample.velocityEnuMss[1] = (float)_msg.twist().linear().y();
    ctx.sample.velocityEnuMss[2] = (float)_msg.twist().linear().z();
    ctx.sample.odomStampUs = stampUs;
    ctx.hasSample.store(true);
    ctx.odomUpdates.fetch_add(1);
}

static void updateDynamicPose(const gz::msgs::Pose_V &_msg)
{
    const uint64_t stampUs = monotonicMicros();
    std::lock_guard<std::mutex> guard(ctx.lock);
    for (int i = 0; i < _msg.pose_size(); i++) {
        const auto &pose = _msg.pose(i);
        if (pose.name() != ctx.modelName) {
            continue;
        }
        ctx.sample.hasPose = true;
        ctx.sample.poseQuatWxyz[0] = (float)pose.orientation().w();
        ctx.sample.poseQuatWxyz[1] = (float)pose.orientation().x();
        ctx.sample.poseQuatWxyz[2] = (float)pose.orientation().y();
        ctx.sample.poseQuatWxyz[3] = (float)pose.orientation().z();
        ctx.sample.poseStampUs = stampUs;
        ctx.hasSample.store(true);
        ctx.poseUpdates.fetch_add(1);
        return;
    }
}

static std::string joinTopic(const char *worldName, const char *modelName, const char *tail)
{
    return std::string("/world/") + worldName + "/model/" + modelName + tail;
}

} // namespace

extern "C" bool gazeboNativeInit(const char *worldName, const char *modelName)
{
    if (initialized.load()) {
        return true;
    }

    memset(&ctx.sample, 0, sizeof(ctx.sample));
    ctx.hasSample.store(false);
    ctx.imuUpdates.store(0);
    ctx.baroUpdates.store(0);
    ctx.magUpdates.store(0);
    ctx.navsatUpdates.store(0);
    ctx.odomUpdates.store(0);
    ctx.poseUpdates.store(0);
    ctx.imuDtUsLast.store(0);
    ctx.imuDtUsMin.store(0);
    ctx.imuDtUsMax.store(0);
    ctx.lastImuStampUs.store(0);
    ctx.modelName = modelName;

    const std::string imuTopic = joinTopic(worldName, modelName, "/link/base_link/sensor/imu_sensor/imu");
    const std::string pressureTopic = joinTopic(worldName, modelName, "/link/base_link/sensor/air_pressure_sensor/air_pressure");
    const std::string magTopic = joinTopic(worldName, modelName, "/link/base_link/sensor/magnetometer_sensor/magnetometer");
    const std::string navsatTopic = joinTopic(worldName, modelName, "/link/base_link/sensor/navsat_sensor/navsat");
    const std::string odomTopic = std::string("/model/") + modelName + "/odometry";
    const std::string dynamicPoseTopic = std::string("/world/") + worldName + "/dynamic_pose/info";
    const std::string motorTopicModel = std::string("/model/") + modelName + "/command/motor_speed";
    const std::string motorTopicCompat = std::string("/") + modelName + "/command/motor_speed";

    if (!ctx.node.Subscribe(imuTopic, &updateImu)) {
        return false;
    }

    if (!ctx.node.Subscribe(pressureTopic, &updatePressure)) {
        return false;
    }

    if (!ctx.node.Subscribe(magTopic, &updateMag)) {
        return false;
    }

    if (!ctx.node.Subscribe(navsatTopic, &updateNavSat)) {
        return false;
    }

    if (!ctx.node.Subscribe(dynamicPoseTopic, &updateDynamicPose)) {
        return false;
    }

    if (!ctx.node.Subscribe(odomTopic, &updateOdometry)) {
        fprintf(stderr, "[SIM][GZ] Odometry topic missing (%s), continuing without velocity.\n", odomTopic.c_str());
    }

    ctx.motorPublisherModelTopicActuators = ctx.node.Advertise<gz::msgs::Actuators>(motorTopicModel);
    ctx.motorPublisherCompatTopicActuators = ctx.node.Advertise<gz::msgs::Actuators>(motorTopicCompat);
    if (!ctx.motorPublisherModelTopicActuators &&
        !ctx.motorPublisherCompatTopicActuators) {
        return false;
    }

    initialized.store(true);
    return true;
}

extern "C" bool gazeboNativePublishMotors(const float *motorRadS, uint8_t motorCount)
{
    if (!initialized.load()) {
        return false;
    }

    gz::msgs::Actuators msgActuators;
    for (uint8_t i = 0; i < motorCount; i++) {
        msgActuators.add_velocity(motorRadS[i]);
    }

    bool published = false;
    if (ctx.motorPublisherModelTopicActuators) {
        published = ctx.motorPublisherModelTopicActuators.Publish(msgActuators) || published;
    }
    if (ctx.motorPublisherCompatTopicActuators) {
        published = ctx.motorPublisherCompatTopicActuators.Publish(msgActuators) || published;
    }

    return published;
}

extern "C" bool gazeboNativeGetSample(gazeboNativeSample_t *sample)
{
    if (!initialized.load() || !ctx.hasSample.load()) {
        return false;
    }

    std::lock_guard<std::mutex> guard(ctx.lock);
    *sample = ctx.sample;
    return true;
}

extern "C" bool gazeboNativeGetStats(gazeboNativeStats_t *stats)
{
    if (!initialized.load()) {
        return false;
    }

    stats->imuUpdates = ctx.imuUpdates.load();
    stats->baroUpdates = ctx.baroUpdates.load();
    stats->magUpdates = ctx.magUpdates.load();
    stats->navsatUpdates = ctx.navsatUpdates.load();
    stats->odomUpdates = ctx.odomUpdates.load();
    stats->poseUpdates = ctx.poseUpdates.load();
    stats->imuDtUsLast = ctx.imuDtUsLast.load();
    stats->imuDtUsMin = ctx.imuDtUsMin.load();
    stats->imuDtUsMax = ctx.imuDtUsMax.load();

    const uint64_t nowUs = monotonicMicros();
    std::lock_guard<std::mutex> guard(ctx.lock);
    stats->imuAgeUs = (ctx.sample.imuStampUs && nowUs > ctx.sample.imuStampUs) ? (uint32_t)(nowUs - ctx.sample.imuStampUs) : 0;
    stats->poseAgeUs = (ctx.sample.poseStampUs && nowUs > ctx.sample.poseStampUs) ? (uint32_t)(nowUs - ctx.sample.poseStampUs) : 0;
    stats->navsatAgeUs = (ctx.sample.navsatStampUs && nowUs > ctx.sample.navsatStampUs) ? (uint32_t)(nowUs - ctx.sample.navsatStampUs) : 0;
    stats->baroAgeUs = (ctx.sample.baroStampUs && nowUs > ctx.sample.baroStampUs) ? (uint32_t)(nowUs - ctx.sample.baroStampUs) : 0;
    stats->magAgeUs = (ctx.sample.magStampUs && nowUs > ctx.sample.magStampUs) ? (uint32_t)(nowUs - ctx.sample.magStampUs) : 0;
    return true;
}

extern "C" void gazeboNativeClose(void)
{
    initialized.store(false);
    ctx.hasSample.store(false);
}

#else

extern "C" bool gazeboNativeInit(const char *worldName, const char *modelName)
{
    (void)worldName;
    (void)modelName;
    return false;
}

extern "C" bool gazeboNativePublishMotors(const float *motorRadS, uint8_t motorCount)
{
    (void)motorRadS;
    (void)motorCount;
    return false;
}

extern "C" bool gazeboNativeGetSample(gazeboNativeSample_t *sample)
{
    (void)sample;
    return false;
}

extern "C" bool gazeboNativeGetStats(gazeboNativeStats_t *stats)
{
    (void)stats;
    return false;
}

extern "C" void gazeboNativeClose(void)
{
}

#endif
